#pragma once

#include <stdint.h>

// Communication protocol constants
#define PROTOCOL_VERSION 1
#define MAX_PACKET_SIZE 64
#define GPS_DATA_PACKET 0x01
#define ALTITUDE_PACKET 0x02

// Packet structures
struct GpsDataPacket {
    uint8_t version;
    uint8_t packetType;
    uint32_t timestamp;
    uint32_t latitude;
    uint32_t longitude;
    uint16_t altitude;
    uint8_t checksum;
};

struct AltitudePacket {
    uint8_t version;
    uint8_t packetType;
    uint32_t timestamp;
    uint16_t currentAltitude;
    uint16_t maxAltitude;
    uint8_t checksum;
};
