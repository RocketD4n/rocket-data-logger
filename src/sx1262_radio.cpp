#include "sx1262_radio.h"

SX1262Radio::SX1262Radio(int csPin, int resetPin, int dio1Pin, int busyPin)
    : radio(new Module(csPin, dio1Pin, resetPin, busyPin)), isInitialized(false) {
    currentPower = 22.0f; // Set initial power to maximum
}

bool SX1262Radio::begin() {
    // Initialize the SX1262 module
    lora = new SX1262(radio);
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
        
        // Set maximum power output (22 dBm = 158 mW)
        lora->setOutputPower(currentPower);
        
        // Set optimized parameters for 863MHz band
        lora->setRfSwitchPins(RADIOLIB_NC, RADIOLIB_NC);
    }
    
    return isInitialized;
}

bool SX1262Radio::configure(float frequency, float bandwidth, uint8_t power) {
    if (!isInitialized) return false;
    
    int state = lora->setFrequency(frequency);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // For LoRa, bandwidth options are typically 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 
    // 41.7E3, 62.5E3, 125E3, 250E3, and 500E3 Hz
    state = lora->setBandwidth(bandwidth * 1000); // Convert kHz to Hz
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // Set power according to parameter
    state = lora->setOutputPower(power);
    if (state != RADIOLIB_ERR_NONE) return false;
    currentPower = power;
    
    return true;
}

bool SX1262Radio::transmit(const uint8_t* data, size_t length) {
    if (!isInitialized) return false;
    int state = lora->transmit(data, length);
    return (state == RADIOLIB_ERR_NONE);
}

int SX1262Radio::receive(uint8_t* data, size_t maxLength) {
    if (!isInitialized) return -1;
    
    int state = lora->receive(data, maxLength);
    if (state == RADIOLIB_ERR_NONE) {
        return lora->getPacketLength();
    }
    return -1;
}

bool SX1262Radio::available() {
    if (!isInitialized) return false;
    return lora->available();
}

void SX1262Radio::sleep() {
    if (isInitialized) {
        lora->sleep();
    }
}

void SX1262Radio::wake() {
    if (isInitialized) {
        lora->standby();
    }
}

float SX1262Radio::getSNR() {
    if (!isInitialized) return -200.0f;
    // SX1262 provides SNR directly in dB
    return lora->getSNR();
}

bool SX1262Radio::setOutputPower(float power) {
    if (!isInitialized) return false;
    // Constrain power to valid range (-9.0 to 22.0 dBm for SX1262)
    power = constrain(power, -9.0, 22.0);
    int state = lora->setOutputPower(power);
    if (state == RADIOLIB_ERR_NONE) {
        currentPower = power;
        return true;
    }
    return false;
}

float SX1262Radio::getCurrentPower() {
    return currentPower;
}

// Destructor to clean up resources
SX1262Radio::~SX1262Radio() {
    if (lora != nullptr) {
        delete lora;
    }
    if (radio != nullptr) {
        delete radio;
    }
}
