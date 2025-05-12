#ifndef CC1101_RADIO_H
#define CC1101_RADIO_H

#include "radio_interface.h"
#include <RadioLib.h>

class CC1101Radio : public Radio {
public:
    CC1101Radio(int csPin, int gdo0Pin, int gdo2Pin);
    virtual ~CC1101Radio() = default;
    
    bool begin() override;
    bool configure(float frequency, float bandwidth, uint8_t power) override;
    bool transmit(const uint8_t* data, size_t length) override;
    int receive(uint8_t* data, size_t maxLength) override;
    bool available() override;
    void sleep() override;
    void wake() override;
    float getSNR() override;  // For CC1101, we approximate SNR from RSSI

private:
    CC1101 radio;
    bool isInitialized;
};

#endif // CC1101_RADIO_H
