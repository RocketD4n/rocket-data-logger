#ifndef SX1278_RADIO_H
#define SX1278_RADIO_H

#include "radio_interface.h"
#include <RadioLib.h>

class SX1278Radio : public Radio {
public:
    SX1278Radio(int csPin, int resetPin, int dio0Pin, int dio1Pin = RADIOLIB_NC);
    virtual ~SX1278Radio();
    
    bool begin() override;
    bool configure(float frequency, float bandwidth, uint8_t power) override;
    bool transmit(const uint8_t* data, size_t length) override;
    int receive(uint8_t* data, size_t maxLength) override;
    bool available() override;
    void sleep() override;
    void wake() override;
    float getSNR() override;  // SX1278 has native SNR support
    bool setOutputPower(float power) override;
    float getCurrentPower() override;
    float getMinimumPower() override { return 2.0f; }  // SX1278 minimum power is 2 dBm
    float getMaximumPower() override { return 20.0f; } // SX1278 maximum power is 20 dBm
    float getMinimumFrequency() override { return 433.0f; }  // SX1278 minimum frequency for 433MHz band
    float getMaximumFrequency() override { return 435.0f; }  // SX1278 maximum frequency for 433MHz band
    
    // Use the base class scanFrequencyRange implementation
    // and only implement the radio-specific part
    float scanFrequenciesAndFindBest(float minFreq, float maxFreq, float stepSize, 
                                     int samplesPerFreq, FreqScanResult* results, int numFreqs,
                                     unsigned long sampleDuration) override;
    
    // Get the announcement frequency for SX1278 (433 MHz band)
    float getAnnouncementFrequency() override { return 433.0f; }
    
    // Get the default operating frequency for SX1278
    float getDefaultFrequency() const override { return 433.0f; }
    
    // Use the default implementations from the Radio base class
    using Radio::processSnrFeedbackAndAdjustPower;
    using Radio::setAdaptivePowerParams;

private:
    Module* radio;
    SX1278* lora;
    bool isInitialized;
    float operatingFrequency = 433.0f; // Store the current operating frequency
};

#endif // SX1278_RADIO_H
