# Rocket Data Logger

This project is a rocket telemetry system that logs GPS data, altitude, and IMU sensor readings, and transmits this data wirelessly using either a CC1101 or SX1278 (LoRa) radio module. It also logs to a local SD card.

## Key Features

- **Real-time Telemetry**: Transmits altitude, GPS, battery, and sensor data in real-time using optimized packet structures
- **Multi-transmitter Support**: Automatically detects and allows selection between multiple rocket transmitters using unique ESP8266 chip IDs
- **Adaptive Power Management**: Adjusts transmission power based on signal strength feedback to optimize battery life
- **Data Visualization**: Multiple graph pages showing altitude, speed, and transmission power over time
- **Touchscreen Interface**: Easy navigation between data pages and transmitter selection
- **Power-saving Mode**: Automatically reduces transmission frequency when battery is low (<45%)

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

The ground station features a multi-page touchscreen display with navigation buttons in the top corners. Users can switch between pages by tapping the left (<) or right (>) buttons.

### Page 1: Main Data Display

The main page shows real-time telemetry data in an easy-to-read format:

```
+------------------------------------------------------+
| <                                                  > |
| Alt:      123.4m               Battery: 3.70V 85%     |
| Max Alt:  456.7m               Launch:  Waiting...    |
| GPS:      12.34567,-12.34567   Temp:    25.1C        |
| Staleness: 5s                  Max-G:   3.2g         |
|                                TX Power: 10 dBm      |
| Stats: 123 packets, 12 errors, SNR: 15.1dB           |
|                                                       |
+------------------------------------------------------+
```

### Page 2: Altitude Graph

The second page displays a real-time graph of altitude versus time, allowing users to visualize the rocket's flight profile:

```
+------------------------------------------------------+
| <           Altitude vs Time                       > |
|                                                       |
|    ^                                                  |
|    |                                                  |
| A  |               /\                                 |
| l  |              /  \                                |
| t  |             /    \                               |
|    |            /      \                              |
|    |           /        \___                          |
|    +---------------------------------------->         |
|                      Time (s)                         |
+------------------------------------------------------+
```

### Page 3: Speed Graph

The third page shows a real-time graph of vertical speed versus time, calculated from altitude changes:

```
+------------------------------------------------------+
| <           Speed vs Time                          > |
|                                                       |
|    ^                                                  |
|    |                                                  |
| S  |               /\                                 |
| p  |              /  \                                |
| e  |             /    \                               |
| e  |            /      \                              |
| d  |___________/        \___                          |
|    +---------------------------------------->         |
|                      Time (s)                         |
+------------------------------------------------------+
```

### Page 4: TX Power Graph

The fourth page displays the transmission power over time, showing how the adaptive power management system adjusts power levels:

```
+------------------------------------------------------+
| <           TX Power vs Time                       > |
|                                                       |
|    ^                                                  |
|    |                                                  |
| P  |                                                  |
| o  |    _______________                               |
| w  |   /                \___                          |
| e  |  /                     \___                      |
| r  | /                          \___                  |
|    +---------------------------------------->         |
|                      Time (s)                         |
+------------------------------------------------------+
```

### Page 5: Transmitter Selection

The fifth page allows the user to select which transmitter to monitor when multiple rockets are in range:

```
+------------------------------------------------------+
| <           Select Transmitter                     > |
|                                                       |
|    Transmitter ID: 0xA1B2C3D4 (selected)             |
|    Uptime: 00:15:32                                  |
|                                                       |
|    Transmitter ID: 0xE5F6G7H8                        |
|    Uptime: 00:05:12                                  |
|                                                       |
|    Transmitter ID: 0xI9J0K1L2                        |
|    Uptime: 00:23:45                                  |
|                                                       |
|    Tap an ID to select that transmitter              |
|                                                       |
+------------------------------------------------------+
```

## Telemetry Protocol

The telemetry system uses a custom protocol with three distinct packet types to optimize data transmission:

### GPS Data Packet (Type 0x01)
```c
struct GpsDataPacket {
    uint8_t version;        // Protocol version (0x01)
    uint8_t packetType;     // 0x01 for GPS data
    uint32_t transmitterId; // Unique ID of the transmitter
    uint32_t latitude;      // Fixed-point latitude (degrees * 1,000,000)
    uint32_t longitude;     // Fixed-point longitude (degrees * 1,000,000)
    uint16_t altitude;      // Altitude in meters
    uint8_t checksum;       // XOR checksum of all previous bytes
};
```

### Altitude Packet (Type 0x02)
```c
struct AltitudePacket {
    uint8_t version;        // Protocol version (0x01)
    uint8_t packetType;     // 0x02 for altitude data
    uint32_t transmitterId; // Unique ID of the transmitter
    uint16_t currentAltitude; // Current altitude in meters
    uint16_t maxAltitude;   // Maximum recorded altitude in meters
    uint16_t temperature;   // Temperature in degrees C * 10
    uint16_t maxG;          // Maximum G-force * 10
    uint8_t launchState;    // 0=waiting, 1=launched, 2=landed
    uint8_t checksum;       // XOR checksum of all previous bytes
};
```

### System Data Packet (Type 0x03)
```c
struct SystemDataPacket {
    uint8_t version;        // Protocol version (0x01)
    uint8_t packetType;     // 0x03 for system data
    uint32_t transmitterId; // Unique ID of the transmitter
    uint32_t uptime;        // Milliseconds since boot
    uint16_t batteryMillivolts; // Battery voltage in millivolts
    uint8_t batteryPercent; // Battery percentage (0-100)
    int8_t txPower;         // Current transmission power in dBm
    uint8_t checksum;       // XOR checksum of all previous bytes
};
```

### SNR Feedback Packet (Type 0xFF)
```c
struct SnrFeedbackPacket {
    uint8_t version;        // Protocol version (0x01)
    uint8_t packetType;     // 0xFF for feedback packets
    uint8_t subType;        // 0x01 for SNR data
    uint32_t transmitterId; // Transmitter ID this feedback is for
    float snrValue;         // Current SNR value measured by receiver
    uint8_t checksum;       // XOR checksum of all previous bytes
};
```

## Pin Configuration

### Data Logger (NodeMCU) Pins
| Function | NodeMCU Pin | GPIO |
|----------|-------------|------|
| Radio CS | D0 | 16 |
| Radio DIO0/GDO0 | D5 | 14 |
| Radio RST/DIO2 | D8 | 15 |
| GPS RX | RX | 3 |
| GPS TX | TX | 1 |
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

## Additional Information

### Performance Expectations

- Range: 2-5km+ with line of sight
- Data Rate: ~0.5-1 kbps effective throughput
- Latency: ~100-200ms per transmission

### Protocol Features
- Fixed packet size (64 bytes)
- Error detection via checksum
- Protocol versioning
- Timestamps for data synchronization
- Separate packet types for different data
- CRC enabled for additional error detection
- Adaptive power management using SNR feedback

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

1. **Dual Radio Support**
   - CC1101 Radio: Simple, low-power FSK radio
   - SX1278 (LoRa) Radio: Long-range, high-reliability radio
   - Automatic detection and configuration of whichever radio is connected

2. **Multi-Transmitter Support**
   - Each transmitter has a unique ID based on the ESP8266 chip ID
   - Receiver automatically detects all transmitters in range
   - User can select which transmitter to monitor via the touchscreen
   - Allows multiple rockets to operate simultaneously on the same frequency

3. **GPS Data Logging**
   - Latitude/Longitude
   - Altitude
   - Timestamps

4. **Altitude Tracking**
   - Current altitude
   - Maximum recorded altitude
   - Pressure-based altitude calculation

5. **IMU Data**
   - Accelerometer readings
   - Gyroscope readings

6. **Wireless Transmission**
   - 433MHz radio
   - Error detection
   - Adaptive power management

## Adaptive Power Management

The system features an adaptive power management system that dynamically adjusts the transmission power based on signal quality feedback:

1. **SNR Feedback Mechanism**
   - Receiver measures Signal-to-Noise Ratio (SNR) of incoming packets
   - SNR feedback is sent back to the transmitter every 5 seconds
   - Transmitter adjusts power based on received SNR feedback

2. **Power Adjustment Algorithm**
   - Target SNR: 15 dB (configurable)
   - Hysteresis: 5 dB (prevents rapid power changes)
   - If SNR > (target + hysteresis), power is reduced to save battery
   - If SNR < (target - hysteresis), power is increased for better reliability
   - Power adjustments occur at most every 10 seconds

3. **Radio-Specific Power Ranges**
   - SX1278 (LoRa): 2 dBm to 20 dBm
   - CC1101: -30 dBm to 10 dBm

4. **Benefits**
   - Extended battery life through optimized power usage
   - Maintains reliable communication at maximum possible distance
   - Reduces interference with other systems
   - Real-time power level displayed on receiver
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
