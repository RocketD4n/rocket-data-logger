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
    float operatingFrequency = 433.0f; // Default operating frequency
    
    // Get the current operating frequency
    float getFrequency() const { return operatingFrequency; }
    
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
    
    // Get the default operating frequency for this radio
    virtual float getDefaultFrequency() const = 0;
    
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
    
    // Structure to store frequency scan results
    struct FreqScanResult {
        float frequency;
        float rssi;
    };
    
    // Scan frequency range to find a clear channel
    // Returns the best frequency found (in MHz)
    // minFreq and maxFreq are in MHz, stepSize is in kHz
    // samplesPerFreq: number of samples to take at each frequency
    // The function automatically calculates the proper sampling duration based on the
    // rocket's transmission duty cycle: normal mode sends altitude data every 100ms,
    // while low battery mode sends data every 10 seconds.
    virtual float scanFrequencyRange(float minFreq, float maxFreq, float stepSize, int samplesPerFreq = 5) {
        // Calculate proper sampling duration based on the rocket's duty cycle
        // In normal mode, altitude data is transmitted every 100ms
        // We only need to sample for slightly longer than that to ensure we catch a transmission
        // Add a small margin (20ms) to account for timing variations
        const unsigned long sampleDuration = 120;
        // Constrain to valid frequency range for this radio
        minFreq = max(minFreq, getMinimumFrequency());
        maxFreq = min(maxFreq, getMaximumFrequency());
        
        // Number of frequencies to scan
        int numFreqs = (int)((maxFreq - minFreq) * 1000.0f / stepSize) + 1;
        if (numFreqs <= 0) return (minFreq + maxFreq) / 2.0f;
        
        // Allocate array to store results
        FreqScanResult* results = new FreqScanResult[numFreqs];
        if (!results) return (minFreq + maxFreq) / 2.0f; // Memory allocation failed
        
        Serial.println("Scanning frequencies...");
        Serial.print("Range: ");
        Serial.print(minFreq);
        Serial.print(" - ");
        Serial.print(maxFreq);
        Serial.print(" MHz, Step: ");
        Serial.print(stepSize);
        Serial.println(" kHz");
        
        // Save current frequency to restore later
        float currentFreq = operatingFrequency;
        
        // Scan each frequency - this part is radio-specific and will be implemented by subclasses
        float bestFreq = scanFrequenciesAndFindBest(minFreq, maxFreq, stepSize, samplesPerFreq, results, numFreqs, sampleDuration);
        
        // Free memory
        delete[] results;
        
        // Restore original frequency
        configure(currentFreq, 250.0, currentPower);
        
        return bestFreq;
    }
    
    // This method must be implemented by each radio subclass to perform the actual scanning
    // It should fill the results array and return the best frequency
    virtual float scanFrequenciesAndFindBest(float minFreq, float maxFreq, float stepSize, 
                                             int samplesPerFreq, FreqScanResult* results, int numFreqs,
                                             unsigned long sampleDuration) {
        // Default implementation just returns the middle frequency
        return (minFreq + maxFreq) / 2.0f;
    }
    
    // Get the minimum frequency supported by this radio (in MHz)
    virtual float getMinimumFrequency() = 0;
    
    // Get the maximum frequency supported by this radio (in MHz)
    virtual float getMaximumFrequency() = 0;
    
    // Get the standard announcement frequency for this radio (in MHz)
    // This is used for initial communication before frequency selection
    // Each radio implementation should return its appropriate band
    virtual float getAnnouncementFrequency() = 0;
};

#endif // RADIO_INTERFACE_H
