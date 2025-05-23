#pragma once

#include <stdint.h>

// Communication protocol constants
#define PROTOCOL_VERSION 0x01
#define MAX_PACKET_SIZE 64

// Packet types
#define PACKET_TYPE_GPS 0x01
#define PACKET_TYPE_ALTITUDE 0x02
#define PACKET_TYPE_SYSTEM 0x03
#define PACKET_TYPE_FREQ_ANNOUNCE 0x04
#define PACKET_TYPE_FREQ_ACK 0x05
#define PACKET_TYPE_COMMAND 0xFE
#define PACKET_TYPE_FEEDBACK 0xFF

// Feedback subtypes
#define FEEDBACK_SUBTYPE_SNR 0x01

// Command subtypes
#define COMMAND_SUBTYPE_BUZZER 0x01
#define COMMAND_SUBTYPE_ABORT 0x02  // Emergency parachute deployment

// Launch state enum
enum LaunchState {
    LAUNCH_STATE_WAITING = 0,  // Pre-launch, on the pad
    LAUNCH_STATE_LAUNCHED = 1, // In flight
    LAUNCH_STATE_LANDED = 2    // Flight complete, on the ground
};

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
    int16_t accelVelocity;   // Velocity from accelerometer integration (cm/s)
    int16_t baroVelocity;    // Velocity from barometric calculation (cm/s)
    int8_t orientationX;     // Current orientation vector X component * 127 (normalized)
    int8_t orientationY;     // Current orientation vector Y component * 127 (normalized)
    int8_t orientationZ;     // Current orientation vector Z component * 127 (normalized)
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
    uint32_t bootTime;      // Boot time in seconds since epoch (from GPS)
    uint8_t launchState;    // Current launch state (waiting, launched, landed)
    uint8_t tiltAngle;      // Tilt angle from vertical in degrees * 2 (0.5 degree resolution)
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

// Command packet sent from receiver to logger
struct CommandPacket {
    uint8_t version;
    uint8_t packetType;   // 0xFE for command packets
    uint8_t subType;      // Command type (see command subtypes)
    uint32_t transmitterId; // Transmitter ID this command is for
    uint8_t commandParam; // Command parameter (1=on, 0=off for buzzer)
    uint8_t checksum;
};

// Frequency announcement packet sent from logger to receiver
struct FrequencyAnnouncePacket {
    uint8_t version;
    uint8_t packetType;     // 0x04 for frequency announcement
    uint32_t transmitterId; // Unique ID of the transmitter
    uint32_t frequency;     // Frequency in kHz (e.g., 863000 for 863.0 MHz)
    uint8_t radioType;      // 0=CC1101, 1=SX1278, 2=SX1262
    uint8_t checksum;
};

// Frequency acknowledgment packet sent from receiver to logger
struct FrequencyAckPacket {
    uint8_t version;
    uint8_t packetType;     // 0x05 for frequency acknowledgment
    uint32_t transmitterId; // Transmitter ID this ack is for
    uint32_t frequency;     // Acknowledged frequency in kHz
    uint8_t checksum;
};
