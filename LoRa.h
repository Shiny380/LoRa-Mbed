// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

// Ported Library of https://github.com/sandeepmistry/arduino-LoRa
//  All credit to sandeeomistry

#ifndef LORA_H
#define LORA_H

#include "Callback.h"
#include "DigitalInOut.h"
#include "DigitalOut.h"
#include "InterruptIn.h"
#include "PinNames.h"
#include "SPI.h"
#include "mbed.h"
#include "mbed_wait_api.h"

#if DEVICE_LPTICKER
    #include "LowPowerTimeout.h"
    #define ALIAS_LORAWAN_TIMER mbed::LowPowerTimeout
#else
    #include "Timeout.h"
    #define ALIAS_LORAWAN_TIMER mbed::Timeout
#endif

#define LORA_DEFAULT_SPI_FREQUENCY 8E6

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

#define LOW  0
#define HIGH 1

class LoRaPort {
   public:
    // TODO: SPI pass in option
    LoRaPort(PinName spi_mosi, PinName spi_miso, PinName spi_sclk, PinName nss,
             PinName reset, PinName dio0);
    ~LoRaPort();

    uint8_t begin(long frequency);
    void    end();

    uint8_t beginPacket(bool implicitHeader = false);
    uint8_t endPacket(bool async = false);

    int16_t parsePacket(uint8_t size = 0);
    int16_t packetRssi();
    float   packetSnr();
    long    packetFrequencyError();

    // from Print
    virtual size_t write(uint8_t byte);
    virtual size_t write(const uint8_t* buffer, size_t size);

    // from Stream
    virtual int16_t available();
    virtual int16_t read();
    virtual int16_t peek();

    void onReceive(Callback<void(uint16_t)> cb);
    void onTxDone(Callback<void()> cb);

    void receive(uint8_t size = 0);

    void lora_idle();
    void lora_sleep();

    void setTxPower(uint8_t level,
                    PinName outputPin = (PinName)PA_OUTPUT_PA_BOOST_PIN);
    void setFrequency(long frequency);
    void setSpreadingFactor(uint32_t sf);
    void setSignalBandwidth(uint32_t sbw);
    void setCodingRate4(uint8_t denominator);
    void setPreambleLength(uint16_t length);
    void setSyncWord(uint8_t sw);
    void enableCrc(bool enable);
    void enableInvertIQ(bool enable);

    void setOCP(uint8_t mA);  // Over Current Protection control

    uint32_t random();

    void setSPIFrequency(uint32_t frequency);

    uint32_t timeOnAir(uint16_t pkt_len);
    bool     channelActive(int16_t rssi_threshold, uint32_t max_sense_time);

   private:
    void explicitHeaderMode();
    void implicitHeaderMode();

    void handleDio0Rise();
    bool isTransmitting();

    uint32_t getSpreadingFactor();
    long     getSignalBandwidth();
    int16_t  getRssi();

    void setLdoFlag();

    uint8_t readRegister(uint8_t address);
    void    writeRegister(uint8_t address, uint8_t value);
    int     singleTransfer(uint8_t address, uint8_t value);

    // static void onDio0Rise();

   private:
    SPI*                     _spi;
    DigitalOut               _ss;
    DigitalOut               _reset;
    InterruptIn              _dio0;
    long                     _frequency;
    float                    _bandwidth;
    uint8_t                  _datarate = 7;
    uint8_t                  _coderate = 5;
    uint16_t                 _preamble_len = 8;
    uint8_t                  _crc_on = false;
    uint16_t                 _packetIndex;
    bool                     _implicitHeaderMode;
    Callback<void(uint16_t)> _onReceive;
    Callback<void()>         _onTxDone;

    Thread     lora_thread;
    EventQueue queue;
};

#endif
