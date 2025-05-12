# Rocket Data Logger

This project is a rocket telemetry system that logs GPS data, altitude, and IMU sensor readings, and transmits this data wirelessly using either a CC1101 or SX1278 (LoRa) radio module. It also logs to a local SD card.

## Hardware Requirements

### Data Logger (Rocket)
- NodeMCU v2 (ESP8266) and power supply
- Either:
  - CC1101 433MHz Transmitter, or
  - SX1278 433MHz LoRa Transmitter
- GPS Module
- BMP180/BMP085 Pressure and Temperature Sensor
- MPU6050 IMU (Accelerometer and Gyroscope)
- SD Card Module
- 2* Relay Modules (for parachute deployment)
- MAX17043 LiPo Battery Fuel Gauge

### Telemetry Receiver (Ground Station)
- ESP32 Dev Board and power supply
- Either:
  - CC1101 433MHz Receiver, or
  - SX1278 433MHz LoRa Receiver
- 2.8" TFT Display (ILI9341)

## Display Layout

The ground station's 320x240 pixel display shows real-time telemetry data in an easy-to-read format:

```
+------------------------------------------------------+
|                                                       |
| Alt:      123.4m               Battery: 3.70V 85%     |
| Max Alt:  456.7m               Launch:  Waiting...    |
| GPS:      12.34567,-12.34567   Temp:    25.1C        |
| Staleness: 5s                  Max-G:   3.2g         |
| Stats: 123 packets, 12 errors, SNR: 15.1dB           |
|                                                       |
+------------------------------------------------------+
```

## Pin Configuration

### Data Logger (NodeMCU) Pins
| Function | NodeMCU Pin | GPIO |
|----------|-------------|------|
| Radio CS | D0 | 16 |
| Radio DIO0/GDO0 | D5 | 14 |
| Radio RST/DIO2 | D8 | 15 |
| GPS RX | D1 | 5 |
| GPS TX | D2 | 4 |
| SD Card CS | D8 | 15 |
| MPU6050 SDA | D2 | 4 |
| MPU6050 SCL | D1 | 5 |
| BMP180 SDA | D2 | 4 |
| BMP180 SCL | D1 | 5 |
| MAX17043 SDA | D2 | 4 |
| MAX17043 SCL | D1 | 5 |
| Primary Relay | D6 | 12 |
| Backup Relay | D7 | 13 |
| LED | D4 | 2 |

## Telemetry Data

The system transmits and displays the following data:
- Current and maximum altitude
- GPS coordinates and fix status
- Battery voltage and charge percentage (via MAX17043 fuel gauge)
- Temperature readings
- Maximum G-force experienced
- Launch state detection
- Packet statistics and signal quality (SNR)

## Radio Options

### CC1101
- Simple FSK/OOK transceiver
- Good for shorter range (up to ~100m with clear line of sight)
- Higher data rates possible
- Lower power consumption
- Simpler to configure

### SX1278 (LoRa)
- Long-range spread spectrum modulation
- Optimized for 2km+ range with clear line of sight
- Configuration:
  - Spreading Factor: 10 (balance of range and speed)
  - Coding Rate: 4/8 (better error correction)
  - Preamble Length: 12 symbols
  - Maximum Power: 20 dBm (100 mW)

Important notes for SX1278 maximum power operation:
1. Must use PA_BOOST pin (not RFO) to achieve 20 dBm output
2. Power supply should handle ~120mA current draw during transmission
3. Check local regulations for maximum allowed power output
4. Consider heat dissipation at maximum power if transmitting frequently
5. Expected performance:
   - Range: 2-5km+ with line of sight
   - Data Rate: ~0.5-1 kbps effective throughput
   - Latency: ~100-200ms per transmission

## Communication Protocol

### Packet Structure

#### GPS Data Packet (0x01)
```c
struct GpsDataPacket {
    uint8_t version;        // Protocol version
    uint8_t packetType;     // 0x01 for GPS data
    uint32_t timestamp;     // Milliseconds since startup
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
    uint32_t timestamp;     // Milliseconds since startup
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
   - 433MHz radio
   - Error detection
   - Timestamped data

5. Deployment System
   - Primary relay activation
   - Backup relay system
   - Altitude-based deployment

## Setup Instructions

### Data Logger Setup
1. Install PlatformIO
2. Clone this repository
3. Connect hardware according to pin configuration
4. Upload code using: `pio run -e rocket_logger -t upload`
5. Configure GPS module for 9600 baud
6. Insert SD card for data logging

### Telemetry Receiver Setup
1. Connect hardware according to pin configuration below
2. Upload code using: `pio run -e rocket_receiver -t upload`

### Receiver Pin Configuration (ESP32)
| Function | ESP32 Pin | GPIO |
|----------|-----------|------|
| TFT MISO | MISO | 19 |
| TFT MOSI | MOSI | 23 |
| TFT SCLK | SCK | 18 |
| TFT CS | CS | 15 |
| TFT DC | DC | 2 |
| TFT RST | RST | 4 |
| Radio CS | CS | 5 |
| Radio DIO0/GDO0 | DIO0 | 4 |
| Radio RST/DIO2 | RST | 16 |

## Dependencies

### Data Logger
- Adafruit MPU6050
- Adafruit BMP085 Unified
- TinyGPSPlus
- RadioLib 
- SPI
- Wire
- SD

### Telemetry Receiver
- TFT_eSPI
- RadioLib 
- SPI
