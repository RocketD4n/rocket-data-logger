#ifndef CC1101_RADIO_H
#define CC1101_RADIO_H

#include "radio_interface.h"
#include <RadioLib.h>

class CC1101Radio : public Radio {
public:
    CC1101Radio(int csPin, int gdo0Pin, int gdo2Pin);
    virtual ~CC1101Radio();
    
    bool begin() override;
    bool configure(float frequency, float bandwidth, uint8_t power) override;
    bool transmit(const uint8_t* data, size_t length) override;
    int receive(uint8_t* data, size_t maxLength) override;
    bool available() override;
    void sleep() override;
    void wake() override;
    float getSNR() override;  // For CC1101, we approximate SNR from RSSI
    bool setOutputPower(float power) override;
    float getCurrentPower() override;
    float getMinimumPower() override { return -30.0f; } // CC1101 minimum power is -30 dBm
    float getMaximumPower() override { return 10.0f; }  // CC1101 maximum power is 10 dBm
    float getMinimumFrequency() override { return 433.0f; }  // CC1101 minimum frequency for 433MHz band
    float getMaximumFrequency() override { return 435.0f; }  // CC1101 maximum frequency for 433MHz band
    
    // Use the base class scanFrequencyRange implementation
    // and only implement the radio-specific part
    float scanFrequenciesAndFindBest(float minFreq, float maxFreq, float stepSize, 
                                     int samplesPerFreq, FreqScanResult* results, int numFreqs,
                                     unsigned long sampleDuration) override;
    
    // Get the announcement frequency for CC1101 (433 MHz band)
    float getAnnouncementFrequency() override { return 433.0f; }
    
    // Get the default operating frequency for CC1101
    float getDefaultFrequency() const override { return 433.0f; }
    
    // Use the default implementations from the Radio base class
    using Radio::processSnrFeedbackAndAdjustPower;
    using Radio::setAdaptivePowerParams;

private:
    Module* radio;
    CC1101* cc1101;
    bool isInitialized;
    float operatingFrequency = 433.0f; // Store the current operating frequency
};

#endif // CC1101_RADIO_H
