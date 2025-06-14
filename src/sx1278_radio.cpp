#include "sx1278_radio.h"

SX1278Radio::SX1278Radio(int csPin, int resetPin, int dio0Pin, int dio1Pin)
    : radio(new Module(csPin, dio0Pin, resetPin, dio1Pin)), isInitialized(false) {
    currentPower = 20.0f; // Set initial power to maximum
}

bool SX1278Radio::begin() {
    // Initialize the SX1278 module
    lora = new SX1278(radio);
    int state = lora->begin();
    isInitialized = (state == RADIOLIB_ERR_NONE);
    
    if (isInitialized) {
        // Configure LoRa for long range
        lora->setCodingRate(8);           // 4/8 coding rate - better error correction
        lora->setSpreadingFactor(10);     // SF10 gives good range while keeping reasonable data rate
        lora->setPreambleLength(12);      // Longer preamble for better reception
        lora->setSyncWord(0x12);          // Common LoRa sync word
        
        // Enable CRC for better reliability
        lora->setCRC(true);
        
        // Set maximum power output (20 dBm = 100 mW)
        lora->setOutputPower(currentPower);
        
        // Enable high power mode on the PA_BOOST pin
        lora->setGain(1);
    }
    
    return isInitialized;
}

bool SX1278Radio::configure(float frequency, float bandwidth, uint8_t power) {
    if (!isInitialized) {
        Serial.println("[SX1278] Error: Radio not initialized");
        return false;
    }
    
    int state = lora->setFrequency(frequency);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX1278] Failed to set frequency ");
        Serial.print(String(frequency));
        Serial.print(" error: ");
        Serial.println(state);
        return false;
    }
    
    // Store the current frequency
    operatingFrequency = frequency;
    
    // Set bandwidth - must be one of: 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, or 500 kHz
    // Round to nearest valid bandwidth if needed
    float validBandwidths[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
    float closestBandwidth = 125.0; // Default to 125 kHz if no good match
    float minDiff = 1000.0;
    
    // Find closest valid bandwidth
    for (size_t i = 0; i < sizeof(validBandwidths)/sizeof(validBandwidths[0]); i++) {
        float diff = abs(validBandwidths[i] - bandwidth);
        if (diff < minDiff) {
            minDiff = diff;
            closestBandwidth = validBandwidths[i];
        }
    }
    
    // If the requested bandwidth is more than 10% different, log a warning
    if (minDiff > (bandwidth * 0.1)) {
        Serial.print("[SX1278] Warning: Bandwidth ");
        Serial.print(bandwidth);
        Serial.print(" kHz is not a standard value. Using closest valid bandwidth: ");
        Serial.print(closestBandwidth);
        Serial.println(" kHz");
    }
    
    // Set bandwidth (already in kHz, no conversion needed)
    state = lora->setBandwidth(closestBandwidth);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX1278] Failed to set bandwidth, error: ");
        Serial.println(state);
        return false;
    }
    
    // Set spreading factor (default to 10 for good range/performance balance)
    state = lora->setSpreadingFactor(10);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX1278] Warning: Failed to set spreading factor, error: ");
        Serial.println(state);
    }
    
    // Set coding rate (4/8 for better error correction)
    state = lora->setCodingRate(8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX1278] Warning: Failed to set coding rate, error: ");
        Serial.println(state);
    }
    
    // Set output power
    state = lora->setOutputPower(power);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX1278] Failed to set output power");
        Serial.println(state);
        return false;
    }
    currentPower = power;
    // Enable CRC for better reliability
    state = lora->setCRC(true);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX1278] Warning: Failed to enable CRC, error: ");
        Serial.println(state);
    }
    
    return true;
}

bool SX1278Radio::transmit(const uint8_t* data, size_t length) {
    if (!isInitialized) return false;
    int state = lora->transmit(data, length);
    return (state == RADIOLIB_ERR_NONE);
}

int SX1278Radio::receive(uint8_t* data, size_t maxLength) {
    if (!isInitialized) return -1;
    
    int state = lora->receive(data, maxLength);
    if (state == RADIOLIB_ERR_NONE) {
        return lora->getPacketLength();
    }
    return -1;
}

bool SX1278Radio::available() {
    if (!isInitialized) return false;
    return lora->available();
}

void SX1278Radio::sleep() {
    if (isInitialized) {
        lora->sleep();
    }
}

void SX1278Radio::wake() {
    if (isInitialized) {
        lora->standby();
    }
}

float SX1278Radio::getSNR() {
    if (!isInitialized) return -200.0f;
    // SX1278 provides SNR directly in dB
    return lora->getSNR();
}

bool SX1278Radio::setOutputPower(float power) {
    if (!isInitialized) return false;
    // Constrain power to valid range (2.0 to 20.0 dBm for SX1278)
    power = constrain(power, 2.0, 20.0);
    int state = lora->setOutputPower(power);
    if (state == RADIOLIB_ERR_NONE) {
        currentPower = power;
        return true;
    }
    return false;
}

float SX1278Radio::getCurrentPower() {
    return currentPower;
}

float SX1278Radio::scanFrequenciesAndFindBest(float minFreq, float maxFreq, float stepSize, 
                                             int samplesPerFreq, FreqScanResult* results, int numFreqs,
                                             unsigned long sampleDuration) {
    if (!isInitialized) return (minFreq + maxFreq) / 2.0f; // Default to middle if not initialized
    
    // Put radio in receive mode for scanning
    lora->standby();
    
    // Scan each frequency
    int idx = 0;
    for (float freq = minFreq; freq <= maxFreq && idx < numFreqs; freq += stepSize/1000.0f) {
        // Set frequency
        int state = lora->setFrequency(freq);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print("Failed to set frequency: ");
            Serial.println(freq);
            continue;
        }
        
        // Allow radio to settle
        delay(10);
        
        // Take multiple samples over the specified duration to capture duty cycles
        float totalRssi = 0.0f;
        float maxRssi = -150.0f; // Initialize to very low value
        unsigned long startTime = millis();
        int sampleCount = 0;
        
        // Sample for the specified duration to catch intermittent transmissions
        while (millis() - startTime < sampleDuration) {
            // Sample RSSI
            float rssi = lora->getRSSI();
            totalRssi += rssi;
            // Track the maximum RSSI seen (strongest signal)
            maxRssi = max(maxRssi, rssi);
            sampleCount++;
            delay(sampleDuration / samplesPerFreq); // Distribute samples across the duration
        }
        
        // Store average RSSI for this frequency, but weight the maximum RSSI more heavily
        // This ensures we don't miss intermittent transmissions
        results[idx].frequency = freq;
        
        // If we detected a strong signal at any point, use that as the primary indicator
        if (maxRssi > -120.0f) { // Threshold for detecting a signal
            // Weight: 70% max RSSI, 30% average RSSI
            results[idx].rssi = (maxRssi * 0.7f) + ((totalRssi / sampleCount) * 0.3f);
        } else {
            // If no strong signal detected, just use the average
            results[idx].rssi = totalRssi / sampleCount;
        }
        
        Serial.print(freq);
        Serial.print(" MHz: ");
        Serial.print(results[idx].rssi);
        Serial.print(" dBm (Max: ");
        Serial.print(maxRssi);
        Serial.print(" dBm, Samples: ");
        Serial.print(sampleCount);
        Serial.println(")");
        
        idx++;
    }
    
    // Find frequency with lowest RSSI (least interference)
    float bestFreq = minFreq;
    float lowestRssi = 0.0f;
    bool firstValid = true;
    
    for (int i = 0; i < idx; i++) {
        if (firstValid || results[i].rssi < lowestRssi) {
            lowestRssi = results[i].rssi;
            bestFreq = results[i].frequency;
            firstValid = false;
        }
    }
    
    Serial.print("Best frequency found: ");
    Serial.print(bestFreq);
    Serial.print(" MHz with RSSI: ");
    Serial.print(lowestRssi);
    Serial.println(" dBm");
    
    return bestFreq;
}

// Destructor to clean up resources
SX1278Radio::~SX1278Radio() {
    if (lora != nullptr) {
        delete lora;
    }
    if (radio != nullptr) {
        delete radio;
    }
}
