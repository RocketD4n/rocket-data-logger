#pragma once

#include <stdint.h>

// Communication protocol constants
#define PROTOCOL_VERSION 1
#define MAX_PACKET_SIZE 64
#define GPS_DATA_PACKET 0x01
#define ALTITUDE_PACKET 0x02

uint8_t calculateChecksum(uint8_t* data, size_t length) {
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
    uint8_t checksum;
};
