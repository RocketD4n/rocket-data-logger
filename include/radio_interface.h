#ifndef RADIO_INTERFACE_H
#define RADIO_INTERFACE_H

#include <Arduino.h>

class Radio {
public:
    virtual ~Radio() = default;
    
    // Initialize and configure the radio
    virtual bool begin() = 0;
    
    // Configure radio parameters
    virtual bool configure(float frequency, float bandwidth, uint8_t power) = 0;
    
    // Send data
    virtual bool transmit(const uint8_t* data, size_t length) = 0;
    
    // Receive data
    // Returns number of bytes received, or -1 on error
    virtual int receive(uint8_t* data, size_t maxLength) = 0;
    
    // Check if data is available to read
    virtual bool available() = 0;
    
    // Put radio to sleep to save power
    virtual void sleep() = 0;
    
    // Wake up radio from sleep mode
    virtual void wake() = 0;
};

#endif // RADIO_INTERFACE_H
