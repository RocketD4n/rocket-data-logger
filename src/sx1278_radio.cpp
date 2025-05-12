#include "sx1278_radio.h"

SX1278Radio::SX1278Radio(int csPin, int resetPin, int dio0Pin, int dio1Pin)
    : radio(new Module(csPin, dio0Pin, resetPin, dio1Pin)), isInitialized(false) {
}

bool SX1278Radio::begin() {
    int state = radio.begin();
    isInitialized = (state == RADIOLIB_ERR_NONE);
    
    if (isInitialized) {
        // Configure LoRa for long range
        radio.setCodingRate(8);           // 4/8 coding rate - better error correction
        radio.setSpreadingFactor(10);     // SF10 gives good range while keeping reasonable data rate
        radio.setPreambleLength(12);      // Longer preamble for better reception
        radio.setSyncWord(0x12);          // Common LoRa sync word
        
        // Enable CRC for better reliability
        radio.setCRC(true);
        
        // Lower data rate for better sensitivity
        radio.setBitRate(9.6);
        
        // Set maximum power output (20 dBm = 100 mW)
        radio.setOutputPower(20.0);
        
        // Enable high power mode on the PA_BOOST pin
        radio.setGain(1);
    }
    
    return isInitialized;
}

bool SX1278Radio::configure(float frequency, float bandwidth, uint8_t power) {
    if (!isInitialized) return false;
    
    int state = radio.setFrequency(frequency);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // For LoRa, bandwidth options are typically 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 
    // 41.7E3, 62.5E3, 125E3, 250E3, and 500E3 Hz
    state = radio.setBandwidth(bandwidth * 1000); // Convert kHz to Hz
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // Power is already set to maximum in begin()
    // Ignore power parameter to ensure we always use max power
    
    return true;
}

bool SX1278Radio::transmit(const uint8_t* data, size_t length) {
    if (!isInitialized) return false;
    int state = radio.transmit(data, length);
    return (state == RADIOLIB_ERR_NONE);
}

int SX1278Radio::receive(uint8_t* data, size_t maxLength) {
    if (!isInitialized) return -1;
    
    int state = radio.receive(data, maxLength);
    if (state == RADIOLIB_ERR_NONE) {
        return radio.getPacketLength();
    }
    return -1;
}

bool SX1278Radio::available() {
    if (!isInitialized) return false;
    return radio.available();
}

void SX1278Radio::sleep() {
    if (isInitialized) {
        radio.sleep();
    }
}

void SX1278Radio::wake() {
    if (isInitialized) {
        radio.standby();
    }
}

float SX1278Radio::getSNR() {
    if (!isInitialized) return -200.0f;
    // SX1278 provides SNR directly in dB
    return radio.getSNR();
}
