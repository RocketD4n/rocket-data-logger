#ifndef RADIO_INTERFACE_H
#define RADIO_INTERFACE_H

#include <Arduino.h>
#include "rocket_telemetry_protocol.h"

class Radio {
protected:
    // Adaptive power control parameters
    float targetSnr = 15.0f;
    float snrHysteresis = 5.0f;
    float powerAdjustStep = 2.0f;
    float lastReceivedSnr = 0.0f;
    unsigned long lastPowerAdjustTime = 0;
    float currentPower = 10.0f;
    
    // Local transmitter ID
    uint32_t transmitterId = 0;
    
    // Process SNR feedback packet
    bool processSnrFeedbackPacket(uint8_t* data, int length) {
        if (length >= sizeof(SnrFeedbackPacket) && 
            data[0] == PROTOCOL_VERSION && 
            data[1] == PACKET_TYPE_FEEDBACK && 
            data[2] == FEEDBACK_SUBTYPE_SNR) {
            
            // Extract packet data
            SnrFeedbackPacket* packet = (SnrFeedbackPacket*)data;
            
            // Verify this packet is for our transmitter
            if (packet->transmitterId == transmitterId) {
                lastReceivedSnr = packet->snrValue;
                
                Serial.print("Received SNR feedback for transmitter 0x");
                Serial.print(transmitterId, HEX);
                Serial.print(": ");
                Serial.println(lastReceivedSnr);
                return true;
            } else {
                Serial.print("Ignoring SNR feedback for different transmitter 0x");
                Serial.println(packet->transmitterId, HEX);
            }
        }
        return false;
    }

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
    
    // Get Signal-to-Noise Ratio in dB
    virtual float getSNR() = 0;
    
    // Set output power level (in dBm)
    virtual bool setOutputPower(float power) = 0;
    
    // Get current output power level (in dBm)
    virtual float getCurrentPower() = 0;
    
    // Set the transmitter ID for filtering SNR feedback packets
    void setTransmitterId(uint32_t id) {
        transmitterId = id;
        Serial.print("Radio transmitter ID set to 0x");
        Serial.println(transmitterId, HEX);
    }
    
    // Process any received SNR feedback and adjust power if needed
    // Returns true if SNR feedback was received and processed
    virtual bool processSnrFeedbackAndAdjustPower(float targetSnr) {
        // Check for any SNR feedback packets
        uint8_t receivedData[MAX_PACKET_SIZE];
        int bytesRead = receive(receivedData, MAX_PACKET_SIZE);
        bool feedbackReceived = false;
        
        if (bytesRead > 0) {
            feedbackReceived = processSnrFeedbackPacket(receivedData, bytesRead);
        }
        
        // Adjust power based on received SNR feedback
        if (feedbackReceived && (millis() - lastPowerAdjustTime >= 10000)) { // Every 10 seconds
            // If SNR is too high, reduce power to save battery
            if (lastReceivedSnr > targetSnr + snrHysteresis) {
                float minPower = getMinimumPower();
                float newPower = max(currentPower - powerAdjustStep, minPower);
                if (setOutputPower(newPower)) {
                    Serial.print("Reducing TX power to ");
                    Serial.print(newPower);
                    Serial.println(" dBm");
                }
            }
            // If SNR is too low, increase power for better reliability
            else if (lastReceivedSnr < targetSnr - snrHysteresis) {
                float maxPower = getMaximumPower();
                float newPower = min(currentPower + powerAdjustStep, maxPower);
                if (setOutputPower(newPower)) {
                    Serial.print("Increasing TX power to ");
                    Serial.print(newPower);
                    Serial.println(" dBm");
                }
            }
            lastPowerAdjustTime = millis();
        }
        
        return feedbackReceived;
    }
    
    // Set the adaptive power control parameters
    virtual void setAdaptivePowerParams(float targetSnr, float hysteresis, float adjustStep) {
        this->targetSnr = targetSnr;
        this->snrHysteresis = hysteresis;
        this->powerAdjustStep = adjustStep;
    }
    
    // Get minimum power level supported by this radio (in dBm)
    virtual float getMinimumPower() = 0;
    
    // Get maximum power level supported by this radio (in dBm)
    virtual float getMaximumPower() = 0;
};

#endif // RADIO_INTERFACE_H
