#ifndef SX1278_RADIO_H
#define SX1278_RADIO_H

#include "radio_interface.h"
#include <RadioLib.h>

class SX1278Radio : public Radio {
public:
    SX1278Radio(int csPin, int resetPin, int dio0Pin, int dio1Pin = RADIOLIB_NC);
    virtual ~SX1278Radio() = default;
    
    bool begin() override;
    bool configure(float frequency, float bandwidth, uint8_t power) override;
    bool transmit(const uint8_t* data, size_t length) override;
    int receive(uint8_t* data, size_t maxLength) override;
    bool available() override;
    void sleep() override;
    void wake() override;
    float getSNR() override;  // SX1278 has native SNR support

private:
    SX1278 radio;
    bool isInitialized;
};

#endif // SX1278_RADIO_H
