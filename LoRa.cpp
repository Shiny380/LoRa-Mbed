// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#include <LoRa.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d
#define REG_RSSIVALUE            0x1B

// modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP           0x00
#define MODE_STDBY           0x01
#define MODE_TX              0x03
#define MODE_RX_CONTINUOUS   0x05
#define MODE_RX_SINGLE       0x06

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH     255
#define RSSI_OFFSET_LF     -164.0
#define RSSI_OFFSET_HF     -157.0
#define RF_MID_BAND_THRESH 525000000

LoRaPort::LoRaPort(PinName spi_mosi, PinName spi_miso, PinName spi_sclk,
                   PinName nss, PinName reset, PinName dio0)
    : _ss(nss),
      _reset(reset),
      _dio0(dio0),
      _frequency(0),
      _packetIndex(0),
      _implicitHeaderMode(0),
      _onReceive(NULL),
      _onTxDone(NULL),
      lora_thread(osPriorityRealtime, OS_STACK_SIZE, NULL, "LR-SX1276"),
      queue(32 * EVENTS_EVENT_SIZE) {
    _spi = new SPI(spi_mosi, spi_miso, spi_sclk);
    //   // SPI bus frequency
    uint32_t spi_freq = LORA_DEFAULT_SPI_FREQUENCY;

    //   // Hold chip-select high
    _ss.write(HIGH);
    _spi->format(8, 0);

#if defined(TARGET_KL25Z)
    // bus-clock frequency is halved -> double the SPI frequency to compensate
    _spi->frequency(spi_freq * 2);
#else
    // otherwise use default SPI frequency which is 8 MHz
    _spi->frequency(spi_freq);
#endif
}

LoRaPort::~LoRaPort() {
    delete _spi;
}

uint8_t LoRaPort::begin(long frequency) {
    // setup pins
    // set SS high
    _ss.write(HIGH);

    // perform reset
    _reset.write(LOW);
    wait_us(10000);
    _reset.write(HIGH);
    wait_us(10000);

    // check version
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12) {
        return 0;
    }

    // put in sleep mode
    lora_sleep();

    // set frequency
    setFrequency(frequency);

    // set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // set auto AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    setTxPower(17);

    // put in standby mode
    lora_idle();
    lora_thread.start(callback(&queue, &EventQueue::dispatch_forever));
    return 1;
}

void LoRaPort::end() {
    // put in sleep mode
    lora_sleep();

    // stop SPI
    delete _spi;
    lora_thread.terminate();
}

uint8_t LoRaPort::beginPacket(bool implicitHeader) {
    if (isTransmitting()) {
        return 0;
    }

    // put in standby mode
    lora_idle();

    if (implicitHeader) {
        implicitHeaderMode();
    } else {
        explicitHeaderMode();
    }

    // reset FIFO address and paload length
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0);

    return 1;
}

uint8_t LoRaPort::endPacket(bool async) {
    if ((async) && (_onTxDone))
        writeRegister(REG_DIO_MAPPING_1, 0x40);  // DIO0 => TXDONE

    // put in TX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if (!async) {
        // wait for TX done
        while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        }
        // clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }
    return 1;
}

bool LoRaPort::isTransmitting() {
    if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
        return true;
    }

    if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
        // clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return false;
}

int16_t LoRaPort::parsePacket(uint8_t size) {
    int8_t packetLength = 0;
    int8_t irqFlags = readRegister(REG_IRQ_FLAGS);

    if (size > 0) {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xFF);
    } else {
        explicitHeaderMode();
    }

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) &&
        (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        _packetIndex = 0;

        // read packet length
        if (_implicitHeaderMode) {
            packetLength = readRegister(REG_PAYLOAD_LENGTH);
        } else {
            packetLength = readRegister(REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        writeRegister(REG_FIFO_ADDR_PTR,
                      readRegister(REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        lora_idle();
    } else if (readRegister(REG_OP_MODE) !=
               (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        // not currently in RX mode

        // reset FIFO address
        writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int16_t LoRaPort::packetRssi() {
    return (readRegister(REG_PKT_RSSI_VALUE) -
            (_frequency < 868E6 ? 164 : 157));
}

float LoRaPort::packetSnr() {
    return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRaPort::packetFrequencyError() {
    int32_t freqError = 0;
    freqError =
        static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 7);  // B111
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

    if (readRegister(REG_FREQ_ERROR_MSB) & 8) {  // Sign bit is on //B1000
        freqError -= 524288;                     // B1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6;  // FXOSC: crystal oscillator (XTAL) frequency
                               // (2.5. Chip Specification, p. 14)
    const float fError =
        ((static_cast<float>(freqError) * (1L << 24)) / fXtal) *
        (getSignalBandwidth() / 500000.0f);  // p. 37

    return static_cast<long>(fError);
}

size_t LoRaPort::write(uint8_t byte) {
    return write(&byte, sizeof(byte));
}

size_t LoRaPort::write(const uint8_t *buffer, size_t size) {
    int8_t currentLength = readRegister(REG_PAYLOAD_LENGTH);

    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }

    // write data
    for (size_t i = 0; i < size; i++) {
        writeRegister(REG_FIFO, buffer[i]);
    }

    // update length
    writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

int16_t LoRaPort::available() {
    return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int16_t LoRaPort::read() {
    if (!available()) {
        return -1;
    }

    _packetIndex++;

    return readRegister(REG_FIFO);
}

int16_t LoRaPort::peek() {
    if (!available()) {
        return -1;
    }

    // store current FIFO address
    uint8_t currentAddress = readRegister(REG_FIFO_ADDR_PTR);

    // read
    int16_t b = readRegister(REG_FIFO);

    // restore FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

void LoRaPort::onReceive(Callback<void(uint16_t)> cb) {
    _onReceive = cb;

    if (cb) {
        _dio0.rise(queue.event(callback(this, &LoRaPort::handleDio0Rise)));
    } else {
        _dio0.rise(nullptr);
    }
}

void LoRaPort::onTxDone(Callback<void()> cb) {
    _onTxDone = cb;

    if (cb) {
        _dio0.rise(queue.event(callback(this, &LoRaPort::handleDio0Rise)));
    } else {
        _dio0.rise(nullptr);
    }
}

void LoRaPort::receive(uint8_t size) {
    writeRegister(REG_DIO_MAPPING_1, 0x00);  // DIO0 => RXDONE

    if (size > 0) {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicitHeaderMode();
    }
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
// #endif

void LoRaPort::lora_idle() {
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaPort::lora_sleep() {
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaPort::setTxPower(uint8_t level, PinName outputPin) {
    if (PA_OUTPUT_RFO_PIN == outputPin) {
        // RFO
        if (level > 14) {
            level = 14;
        }

        writeRegister(REG_PA_CONFIG, 0x70 | level);
    } else {
        // PA BOOST
        if (level > 17) {
            if (level > 20) {
                level = 20;
            }

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            writeRegister(REG_PA_DAC, 0x87);
            setOCP(140);
        } else {
            if (level < 2) {
                level = 2;
            }
            // Default value PA_HF/LF or +17dBm
            writeRegister(REG_PA_DAC, 0x84);
            setOCP(100);
        }

        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRaPort::setFrequency(long frequency) {
    _frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

uint32_t LoRaPort::getSpreadingFactor() {
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void LoRaPort::setSpreadingFactor(uint32_t sf) {
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }

    if (sf == 6) {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(
        REG_MODEM_CONFIG_2,
        (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    setLdoFlag();
    _datarate = sf;
}

long LoRaPort::getSignalBandwidth() {
    uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

    switch (bw) {
        case 0:
            return 7.8E3;
        case 1:
            return 10.4E3;
        case 2:
            return 15.6E3;
        case 3:
            return 20.8E3;
        case 4:
            return 31.25E3;
        case 5:
            return 41.7E3;
        case 6:
            return 62.5E3;
        case 7:
            return 125E3;
        case 8:
            return 250E3;
        case 9:
            return 500E3;
    }

    return -1;
}

void LoRaPort::setSignalBandwidth(uint32_t sbw) {
    uint8_t bw;

    if (sbw <= 7.8E3) {
        bw = 0;
    } else if (sbw <= 10.4E3) {
        bw = 1;
        _bandwidth = 10.4E3;
    } else if (sbw <= 15.6E3) {
        bw = 2;
        _bandwidth = 15.6E3;
    } else if (sbw <= 20.8E3) {
        bw = 3;
        _bandwidth = 20.8E3;
    } else if (sbw <= 31.25E3) {
        bw = 4;
        _bandwidth = 31.25E3;
    } else if (sbw <= 41.7E3) {
        bw = 5;
        _bandwidth = 41.7E3;
    } else if (sbw <= 62.5E3) {
        bw = 6;
        _bandwidth = 62.5E3;
    } else if (sbw <= 125E3) {
        bw = 7;
        _bandwidth = 125E3;
    } else if (sbw <= 250E3) {
        bw = 8;
        _bandwidth = 250E3;
    } else /*if (sbw <= 250E3)*/ {
        bw = 9;
        _bandwidth = 500E3;
    }

    writeRegister(REG_MODEM_CONFIG_1,
                  (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    setLdoFlag();
}

void LoRaPort::setLdoFlag() {
    // Section 4.1.1.5
    long symbolDuration =
        1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()));

    // Section 4.1.1.6
    bool ldoOn = symbolDuration > 16;

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    config3 ^= (-ldoOn ^ config3) & (1UL << 3);  // TODO: check this!
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRaPort::setCodingRate4(uint8_t denominator) {
    if (denominator < 5) {
        denominator = 5;
    } else if (denominator > 8) {
        denominator = 8;
    }

    uint8_t cr = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1,
                  (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
    _coderate = cr;
}

void LoRaPort::setPreambleLength(uint16_t length) {
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
    _preamble_len = length;
}

void LoRaPort::setSyncWord(uint8_t sw) {
    writeRegister(REG_SYNC_WORD, sw);
}

void LoRaPort::enableCrc(bool enable) {
    if (enable) {
        writeRegister(REG_MODEM_CONFIG_2,
                      readRegister(REG_MODEM_CONFIG_2) | 0x04);
    } else {
        writeRegister(REG_MODEM_CONFIG_2,
                      readRegister(REG_MODEM_CONFIG_2) & 0xfb);
    }
    _crc_on = enable ? 1 : 0;
}

void LoRaPort::enableInvertIQ(bool enable) {
    if (enable) {
        writeRegister(REG_INVERTIQ, 0x66);
        writeRegister(REG_INVERTIQ2, 0x19);
    } else {
        writeRegister(REG_INVERTIQ, 0x27);
        writeRegister(REG_INVERTIQ2, 0x1d);
    }
}

void LoRaPort::setOCP(uint8_t mA) {
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
        ocpTrim = (mA - 45) / 5;
    } else if (mA <= 240) {
        ocpTrim = (mA + 30) / 10;
    }

    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

uint32_t LoRaPort::random() {
    return readRegister(REG_RSSI_WIDEBAND);
}

void LoRaPort::setSPIFrequency(uint32_t frequency) {
    _spi->frequency(frequency);
}

void LoRaPort::explicitHeaderMode() {
    _implicitHeaderMode = 0;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaPort::implicitHeaderMode() {
    _implicitHeaderMode = 1;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

uint32_t LoRaPort::timeOnAir(uint16_t pkt_len) {
    uint32_t airTime = 0;
    // Symbol rate : time for one symbol (secs)
    float rs = _bandwidth / (1 << _datarate);
    float ts = 1 / rs;
    // time of preamble
    float tPreamble = (_preamble_len + 4.25f) * ts;
    // Symbol length of payload and time
    float tmp =
        ceil((8 * pkt_len - 4 * _datarate + 28 + 16 * _crc_on -
              (0))  // TODO: fixed len always false. need to change
             / (float)(4 * (_datarate -
                            (0))))  // low datarate optimise... just set to 0.
                                    // only needed for clock drift at SF11-12
        * (_coderate + 4);
    float tPayload = tmp * ts;
    // Time on air
    float tOnAir = tPreamble + tPayload;
    // return ms secs
    airTime = floor(tOnAir * 1000);

    return airTime;
}

bool LoRaPort::channelActive(int16_t  rssi_threshold,
                             uint32_t max_sense_time_ms) {
    bool    status = false;
    int16_t rssi = 0;

    Timer elapsed_time;
    elapsed_time.start();

    while (elapsed_time.read_ms() < (int)max_sense_time_ms) {
        rssi = getRssi();
        if (rssi > rssi_threshold) {
            status = true;
            break;
        }
    }
    return status;
}

int16_t LoRaPort::getRssi() {
    int16_t rssi = -1;
    if (_frequency > RF_MID_BAND_THRESH) {
        rssi = RSSI_OFFSET_HF + readRegister(REG_RSSIVALUE);
    } else {
        rssi = RSSI_OFFSET_LF + readRegister(REG_RSSIVALUE);
    }
    return rssi;
}

void LoRaPort::handleDio0Rise() {
    uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
            // received a packet
            _packetIndex = 0;

            // read packet length
            uint8_t packetLength = _implicitHeaderMode
                                       ? readRegister(REG_PAYLOAD_LENGTH)
                                       : readRegister(REG_RX_NB_BYTES);

            // set FIFO address to current RX address
            writeRegister(REG_FIFO_ADDR_PTR,
                          readRegister(REG_FIFO_RX_CURRENT_ADDR));

            if (_onReceive) {
                _onReceive(packetLength);
            }

            // reset FIFO address
            writeRegister(REG_FIFO_ADDR_PTR, 0);
        } else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
            if (_onTxDone) {
                _onTxDone();
            }
        }
    }
}

uint8_t LoRaPort::readRegister(uint8_t address) {
    return singleTransfer(address & 0x7f, 0x00);
}

void LoRaPort::writeRegister(uint8_t address, uint8_t value) {
    singleTransfer(address | 0x80, value);
}

int LoRaPort::singleTransfer(uint8_t address, uint8_t value) {
    int response;

    _ss.write(LOW);

    _spi->lock();
    _spi->write(address);
    response = _spi->write(value);
    _spi->unlock();

    _ss.write(HIGH);

    return response;
}
