#ifndef SX1262_RADIO_H
#define SX1262_RADIO_H

#include "radio_interface.h"
#include <RadioLib.h>

class SX1262Radio : public Radio {
public:
    SX1262Radio(int csPin, int resetPin, int dio1Pin, int busyPin = RADIOLIB_NC);
    virtual ~SX1262Radio();
    
    bool begin() override;
    bool configure(float frequency, float bandwidth, uint8_t power) override;
    bool transmit(const uint8_t* data, size_t length) override;
    int receive(uint8_t* data, size_t maxLength) override;
    bool available() override;
    void sleep() override;
    void wake() override;
    float getSNR() override;  // SX1262 has native SNR support
    bool setOutputPower(float power) override;
    float getCurrentPower() override;
    float getMinimumPower() override { return -9.0f; }  // SX1262 minimum power is -9 dBm
    float getMaximumPower() override { return 22.0f; }  // SX1262 maximum power is 22 dBm
    
    // Use the default implementations from the Radio base class
    using Radio::processSnrFeedbackAndAdjustPower;
    using Radio::setAdaptivePowerParams;

private:
    Module* radio;
    SX1262* lora;
    bool isInitialized;
};

#endif // SX1262_RADIO_H
