#pragma once

#include <stdint.h>

// Communication protocol constants
#define PROTOCOL_VERSION 1
#define MAX_PACKET_SIZE 64
#define GPS_DATA_PACKET 0x01
#define ALTITUDE_PACKET 0x02
#define SNR_FEEDBACK_PACKET 0xFF

inline uint8_t calculateChecksum(uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length - 1; i++) { // -1 to exclude the checksum byte
        checksum ^= data[i];
    }
    return checksum;
}

// Packet structures
struct GpsDataPacket {
    uint8_t version;
    uint8_t packetType;
    uint32_t timestamp;
    uint32_t latitude;
    uint32_t longitude;
    uint16_t altitude;
    uint16_t batteryMillivolts;
    uint8_t batteryPercent;
    int8_t txPower;      // Current transmission power in dBm
    uint8_t checksum;
};

struct AltitudePacket {
    uint8_t version;
    uint8_t packetType;
    uint32_t timestamp;
    uint16_t currentAltitude;
    uint16_t maxAltitude;
    uint16_t temperature;
    uint16_t maxG;
    uint8_t launchState;
    int8_t txPower;      // Current transmission power in dBm
    uint8_t checksum;
};

// Packet sent from receiver to logger with SNR feedback
struct SnrFeedbackPacket {
    uint8_t version;
    uint8_t packetType;   // 0xFF for SNR feedback
    uint8_t subType;      // 0x01 for SNR data
    float snrValue;       // Current SNR value measured by receiver
    uint8_t checksum;
};
