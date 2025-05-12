#include "cc1101_radio.h"

CC1101Radio::CC1101Radio(int csPin, int gdo0Pin, int gdo2Pin)
    : radio(new Module(csPin, gdo0Pin, gdo2Pin, RADIOLIB_NC)), isInitialized(false) {
}

bool CC1101Radio::begin() {
    int state = radio.begin();
    isInitialized = (state == RADIOLIB_ERR_NONE);
    return isInitialized;
}

bool CC1101Radio::configure(float frequency, float bandwidth, uint8_t power) {
    if (!isInitialized) return false;
    
    int state = radio.setFrequency(frequency);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // Set RX bandwidth
    state = radio.setRxBandwidth(bandwidth);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    state = radio.setOutputPower(power);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    return true;
}

bool CC1101Radio::transmit(const uint8_t* data, size_t length) {
    if (!isInitialized) return false;
    int state = radio.transmit(data, length);
    return (state == RADIOLIB_ERR_NONE);
}

int CC1101Radio::receive(uint8_t* data, size_t maxLength) {
    if (!isInitialized) return -1;
    
    int state = radio.receive(data, maxLength);
    if (state == RADIOLIB_ERR_NONE) {
        return radio.getPacketLength();
    }
    return -1;
}

bool CC1101Radio::available() {
    if (!isInitialized) return false;
    return radio.available();
}

void CC1101Radio::sleep() {
    if (isInitialized) {
        radio.sleep();
    }
}

void CC1101Radio::wake() {
    if (isInitialized) {
        radio.standby();
    }
}

float CC1101Radio::getSNR() {
    if (!isInitialized) return -200.0f;
    // For CC1101, we approximate SNR from RSSI
    // RSSI above -60dBm is considered very good (high SNR)
    // RSSI below -100dBm is considered very poor (low SNR)
    // Map this range to a reasonable SNR range
    float rssi = radio.getRSSI();
    if (rssi > -60) return 30.0f;  // Maximum SNR
    if (rssi < -100) return 0.0f;   // Minimum SNR
    return (rssi + 100.0f) * 0.75f; // Linear mapping from -100..-60 to 0..30
}
