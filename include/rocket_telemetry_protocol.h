#pragma once

#include <stdint.h>

// Communication protocol constants
#define PROTOCOL_VERSION 0x01
#define MAX_PACKET_SIZE 64

// Packet types
#define PACKET_TYPE_GPS 0x01
#define PACKET_TYPE_ALTITUDE 0x02
#define PACKET_TYPE_SYSTEM 0x03
#define PACKET_TYPE_FEEDBACK 0xFF

// Feedback subtypes
#define FEEDBACK_SUBTYPE_SNR 0x01

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
    uint32_t transmitterId; // Unique ID of the transmitter
    uint32_t latitude;
    uint32_t longitude;
    uint16_t altitude;
    uint8_t checksum;
};

struct AltitudePacket {
    uint8_t version;
    uint8_t packetType;
    uint32_t transmitterId; // Unique ID of the transmitter
    uint16_t currentAltitude;
    uint16_t maxAltitude;
    uint16_t temperature;
    uint16_t maxG;
    uint8_t launchState;
    uint8_t checksum;
};

// New system data packet containing battery, uptime, and transmission power info
struct SystemDataPacket {
    uint8_t version;
    uint8_t packetType;     // 0x03 for system data
    uint32_t transmitterId; // Unique ID of the transmitter
    uint32_t uptime;        // Milliseconds since boot
    uint16_t batteryMillivolts;
    uint8_t batteryPercent;
    int8_t txPower;         // Current transmission power in dBm
    uint8_t checksum;
};

// Packet sent from receiver to logger with SNR feedback
struct SnrFeedbackPacket {
    uint8_t version;
    uint8_t packetType;   // 0xFF for SNR feedback
    uint8_t subType;      // 0x01 for SNR data
    uint32_t transmitterId; // Transmitter ID this feedback is for
    float snrValue;       // Current SNR value measured by receiver
    uint8_t checksum;
};
