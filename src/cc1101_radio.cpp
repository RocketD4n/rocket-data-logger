#include "cc1101_radio.h"

CC1101Radio::CC1101Radio(int csPin, int gdo0Pin, int gdo2Pin)
    : radio(new Module(csPin, gdo0Pin, gdo2Pin, RADIOLIB_NC)), isInitialized(false) {
    currentPower = 10.0f; // Set initial power to default
}

bool CC1101Radio::begin() {
    // Initialize the CC1101 module
    cc1101 = new CC1101(radio);
    int state = cc1101->begin();
    isInitialized = (state == RADIOLIB_ERR_NONE);
    return isInitialized;
}

bool CC1101Radio::configure(float frequency, float bandwidth, uint8_t power) {
    if (!isInitialized) return false;
    
    int state = cc1101->setFrequency(frequency);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // Set RX bandwidth
    state = cc1101->setRxBandwidth(bandwidth);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    state = cc1101->setOutputPower(power);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // Store the current power level
    currentPower = power;
    
    return true;
}

bool CC1101Radio::transmit(const uint8_t* data, size_t length) {
    if (!isInitialized) return false;
    int state = cc1101->transmit(data, length);
    return (state == RADIOLIB_ERR_NONE);
}

int CC1101Radio::receive(uint8_t* data, size_t maxLength) {
    if (!isInitialized) return -1;
    
    int state = cc1101->receive(data, maxLength);
    if (state == RADIOLIB_ERR_NONE) {
        return cc1101->getPacketLength();
    }
    return -1;
}

bool CC1101Radio::available() {
    if (!isInitialized) return false;
    return cc1101->available();
}

void CC1101Radio::sleep() {
    if (isInitialized) {
        cc1101->sleep();
    }
}

void CC1101Radio::wake() {
    if (isInitialized) {
        cc1101->standby();
    }
}

float CC1101Radio::getSNR() {
    if (!isInitialized) return -200.0f;
    // For CC1101, we approximate SNR from RSSI
    // RSSI above -60dBm is considered very good (high SNR)
    // RSSI below -100dBm is considered very poor (low SNR)
    // Map this range to a reasonable SNR range
    float rssi = cc1101->getRSSI();
    if (rssi > -60) return 30.0f;  // Maximum SNR
    if (rssi < -100) return 0.0f;   // Minimum SNR
    return (rssi + 100.0f) * 0.75f; // Linear mapping from -100..-60 to 0..30
}

bool CC1101Radio::setOutputPower(float power) {
    if (!isInitialized) return false;
    // CC1101 power levels range from -30 to 10 dBm
    // Constrain to valid range
    power = constrain(power, -30.0, 10.0);
    int state = cc1101->setOutputPower(power);
    if (state == RADIOLIB_ERR_NONE) {
        currentPower = power;
        return true;
    }
    return false;
}

float CC1101Radio::getCurrentPower() {
    return currentPower;
}

// Destructor to clean up resources
CC1101Radio::~CC1101Radio() {
    if (cc1101 != nullptr) {
        delete cc1101;
    }
    if (radio != nullptr) {
        delete radio;
    }
}
