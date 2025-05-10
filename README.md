# Rocket Data Logger

This project is a rocket telemetry system that logs GPS data, altitude, and IMU sensor readings, and transmits this data wirelessly using a CC1101 radio module.

## Hardware Requirements

- NodeMCU v2 (ESP8266)
- CC1101 433MHz Transmitter
- GPS Module
- BMP180/BMP085 Pressure Sensor
- MPU6050 IMU
- SD Card Module
- Relay Module (for deployment)

## Pin Configuration

### NodeMCU Pins
| Function | NodeMCU Pin | GPIO |
|----------|-------------|------|
| CC1101 CS | D0 | 16 |
| CC1101 GDO0 | D5 | 14 |
| CC1101 GDO2 | D8 | 15 |
| GPS RX | D1 | 5 |
| GPS TX | D2 | 4 |
| SD Card CS | D8 | 15 |
| MPU6050 SDA | D2 | 4 |
| MPU6050 SCL | D1 | 5 |
| BMP180 SDA | D2 | 4 |
| BMP180 SCL | D1 | 5 |
| Primary Relay | D6 | 12 |
| Backup Relay | D7 | 13 |
| LED | D4 | 2 |

## Communication Protocol

### Packet Structure

#### GPS Data Packet (0x01)
```c
struct GpsDataPacket {
    uint8_t version;        // Protocol version
    uint8_t packetType;     // 0x01 for GPS data
    uint32_t timestamp;     // Microseconds since startup
    uint32_t latitude;      // Fixed point (x10,000,000)
    uint32_t longitude;     // Fixed point (x10,000,000)
    uint16_t altitude;      // Meters
    uint8_t checksum;       // Packet checksum
};
```

#### Altitude Packet (0x02)
```c
struct AltitudePacket {
    uint8_t version;        // Protocol version
    uint8_t packetType;     // 0x02 for altitude data
    uint32_t timestamp;     // Microseconds since startup
    uint16_t currentAltitude; // Current altitude in meters
    uint16_t maxAltitude;   // Maximum recorded altitude
    uint8_t checksum;       // Packet checksum
};
```

### Protocol Features
- Fixed packet size (64 bytes)
- Error detection via checksum
- Protocol versioning
- Timestamps for data synchronization
- Separate packet types for different data
- CRC enabled for additional error detection

## Data Transmission

- GPS data transmitted every 1 second
- Altitude data transmitted every 100ms
- Radio configuration:
  - Frequency: 433.92 MHz
  - Power: 15 dBm
  - Modulation: FSK
  - Bit rate: 2400 bps
  - Preamble length: 4 bytes
  - Sync word: 0x55
  - Address: 0x01

## Features

1. GPS Data Logging
   - Latitude/Longitude
   - Altitude
   - Timestamps

2. Altitude Tracking
   - Current altitude
   - Maximum recorded altitude
   - Pressure-based altitude calculation

3. IMU Data
   - Accelerometer readings
   - Gyroscope readings

4. Wireless Transmission
   - 433MHz CC1101 radio
   - Error detection
   - Timestamped data

5. Deployment System
   - Primary relay activation
   - Backup relay system
   - Altitude-based deployment

## Setup Instructions

1. Install PlatformIO
2. Clone this repository
3. Connect hardware according to pin configuration
4. Upload code to NodeMCU
5. Configure GPS module for 9600 baud
6. Insert SD card for data logging

## Dependencies

- Adafruit MPU6050
- Adafruit BMP085 Unified
- TinyGPSPlus
- CC1101
- SPI
- Wire
- SD
