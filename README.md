# Rocket Data Logger

This project is a rocket telemetry system that logs GPS data, altitude, and IMU sensor readings, and transmits this data wirelessly using either a CC1101 or SX1278 (LoRa) radio module. It also logs to a local SD card.

## Key Features

- **Real-time Telemetry**: Transmits altitude, GPS, battery, and sensor data in real-time using optimized packet structures
- **Multi-transmitter Support**: Automatically detects and allows selection between multiple rocket transmitters using unique ESP8266 chip IDs
- **Adaptive Power Management**: Adjusts transmission power based on signal strength feedback to optimize battery life
- **Data Visualization**: Multiple graph pages showing altitude, speed, and transmission power over time
- **Touchscreen Interface**: Easy navigation between data pages and transmitter selection
- **Power-saving Mode**: Automatically reduces transmission frequency when battery is low (<45%)
- **SD Card Logging**: Automatically logs all telemetry data to CSV files on the receiver's SD card when installed
- **Recovery Buzzer**: Remote-activated audio beacon to help locate the rocket after landing
- **Dual Velocity Calculation**: Computes velocity using both accelerometer integration and barometric altitude changes for redundancy
- **Orientation Estimation**: Calibrates accelerometer with orientation detection to account for non-vertical mounting
- **Calibration Logging**: Records detailed calibration data to SD card for post-flight analysis

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
- 2.8" TFT Display (ILI9341) with SD card slot
- MicroSD card for data logging

## Display Layout

The ground station features a multi-page touchscreen display with navigation buttons in the top corners. Users can switch between pages by tapping the left (<) or right (>) buttons.

### Page 1: Main Data Display

The main page shows real-time telemetry data in an easy-to-read format:

```
+------------------------------------------------------+
| <              [===|] 85%                         > |
| Alt:      123.4m               Battery: 3.70V 85%     |
| Max Alt:  456.7m               Launch:  Waiting...    |
| GPS:      12.34567,-12.34567   Temp:    25.1C        |
| Staleness: 5s                  Max-G:   3.2g         |
| Accel-V:  45.6 m/s             Baro-V:  44.2 m/s     |
|                                TX Power: 10 dBm      |
| Stats: 123 pkts, 12 errs, SNR: 15.1dB               |
+------------------------------------------------------+
```

### Page 2: Altitude Graph

The second page displays a real-time graph of altitude versus time, allowing users to visualize the rocket's flight profile:

```
+------------------------------------------------------+
| <           [===|] 85%                            > |
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
| <           [===|] 85%                            > |
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
|    Accel-V: ---- Baro-V: ----                         |
+------------------------------------------------------+
```

### Page 4: TX Power Graph

The fourth page displays the transmission power over time, showing how the adaptive power management system adjusts power levels:

```
+------------------------------------------------------+
| <           [===|] 85%                            > |
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

### Page 5: Orientation Data

The fifth page shows the rocket's orientation and tilt information with a dynamic visualization of the rocket's current orientation:

```
+------------------------------------------------------+
| <           [===|] 85%                            > |
|             Rocket Orientation                       |
|                                                       |
|    Tilt from vertical: 12.5°                         |
|                                                       |
|                    /\                                 |
|                   /  \                                |
|                  /    \                               |
|                 /      \                              |
|                /        \                             |
|               /          \                            |
|              /__________\                             |
|                                                       |
|    X-axis: 0.12   Y-axis: -0.24   Z-axis: 0.96        |
|    Staleness: 5s                                     |
+------------------------------------------------------+
```

### Page 6: Last Known Positions

The sixth page displays a list of last known GPS coordinates for all tracked transmitters:

```
+------------------------------------------------------+
| <           [===|] 85%                            > |
|            Last Known Positions                      |
|                                                       |
|    Rocket Alpha                                      |
|    Lat: 37.123456, Lng: -122.543210       [Clear]    |
|    Dist: 1.2km, Bearing: 245° (SW)                   |
|                                                       |
|    Rocket Beta                                       |
|    Lat: 37.234567, Lng: -122.654321       [Clear]    |
|    Dist: 350m, Bearing: 78° (E)                      |
|                                                       |
|    Rocket Gamma                                      |
|    Lat: 37.345678, Lng: -122.765432       [Clear]    |
|    Dist: 2.5km, Bearing: 180° (S)                    |
|                                                       |
+------------------------------------------------------+
```

### Page 7: Transmitter Selection

The seventh page allows the user to select which transmitter to monitor when multiple rockets are in range:

```
+------------------------------------------------------+
| <           [===|] 85%                            > |
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
    uint8_t version;
    uint8_t packetType;
    uint32_t transmitterId; // Unique ID of the transmitter
    uint16_t currentAltitude;
    uint16_t maxAltitude;
    uint16_t temperature;
    uint16_t maxG;
    int16_t accelVelocity;  // Velocity from accelerometer in cm/s
    int16_t baroVelocity;   // Velocity from barometer in cm/s
    int8_t orientationX;    // Current orientation vector X component * 127 (normalized)
    int8_t orientationY;    // Current orientation vector Y component * 127 (normalized)
    int8_t orientationZ;    // Current orientation vector Z component * 127 (normalized)
    uint8_t checksum;
};
```

### System Data Packet (Type 0x03)
```c
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

## Velocity Calculation

The system uses two complementary methods to calculate velocity for redundancy and accuracy:

### Accelerometer-based Velocity

1. **Orientation-aware Integration**: 
   - Acceleration is projected onto the rocket's vertical axis determined during calibration
   - Compensates for non-vertical mounting of the sensor
   - Integrates acceleration over time to derive velocity

2. **Bias Correction**:
   - Calibration process measures and removes sensor bias
   - Gravity compensation is applied based on flight state (in-flight vs. stationary)

3. **Drift Mitigation**:
   - Velocity is reset to zero when stationary and not in flight
   - Gradual decay applied after landing to correct for integration drift

### Barometric-based Velocity

1. **Altitude Differentiation**:
   - Calculates velocity as the rate of change in altitude
   - Uses time-delta between altitude readings for accurate differentiation

2. **Noise Filtering**:
   - Low-pass filter applied to reduce noise in barometric readings
   - Large time steps are ignored to prevent inaccurate calculations

Both velocity values are logged and transmitted, allowing for post-flight comparison and analysis.

## Accelerometer Calibration

The accelerometer undergoes a comprehensive calibration process at startup:

1. **Bias Measurement**:
   - Collects multiple samples (100) while the rocket is stationary
   - Calculates average readings for each axis to determine electronic bias

2. **Orientation Estimation**:
   - Measures gravity vector in the sensor frame
   - Calculates the rocket's vertical axis (opposite to gravity)
   - Determines tilt angle from vertical in degrees

3. **Calibration Data Logging**:
   - Creates a calibration file with timestamp (e.g., `20250517_120000_cal.txt`)
   - Records raw accelerometer data for all samples
   - Stores calculated bias values, gravity vector, rocket axis, and tilt angle
   - Provides detailed data for post-flight analysis and troubleshooting

## SD Card Logging

The receiver automatically logs all telemetry data to CSV files on the SD card when one is inserted in the TFT display's SD card slot. The logging system creates three separate CSV files for each rocket transmitter:

1. **GPS Data Log** (`[transmitter_name/id]_[rocket_boot_time]_gps.csv`)
   - Records latitude, longitude, and altitude data

2. **Altitude Data Log** (`[transmitter_name/id]_[rocket_boot_time]_altitude.csv`)
   - Records current altitude, maximum altitude, temperature, maximum G-force, and launch state

3. **System Data Log** (`[transmitter_name/id]_[rocket_boot_time]_system.csv`)
   - Records uptime, battery voltage, battery percentage, transmission power, and boot time

To minimize SD card wear and optimize storage, data is only written to all three log files when the altitude changes by at least 1 meter. This ensures that critical flight data is captured while avoiding excessive writes during periods of minimal altitude change (e.g., before launch or after landing).

The system creates several types of log files on the SD card:

### 1. Telemetry Log Files

- **Format**: CSV files with timestamped entries
- **Filename Pattern**: `YYYYMMDD_HHMMSS.csv` (based on GPS time at startup)
- **Contents**: All telemetry data including altitude, GPS coordinates, velocity, acceleration, temperature, and system status
- **Each entry includes**:
  - Timestamp
  - Altitude (current and maximum)
  - GPS coordinates
  - Accelerometer-based velocity
  - Barometer-based velocity
  - Temperature
  - Maximum G-force
  - Battery status
  - Launch state

### 2. Calibration Log Files

- **Format**: Text files with detailed calibration data
- **Filename Pattern**: `YYYYMMDD_HHMMSS_cal.txt` (matching the telemetry log)
- **Contents**:
  - Raw accelerometer readings for all calibration samples
  - Average values for each axis
  - Calculated gravity vector
  - Rocket axis orientation
  - Tilt angle from vertical
  - Bias values for each axis

### 3. Receiver Log Files

- **Format**: CSV files with receiver-side data
- **Filename Pattern**: `RX_YYYYMMDD_HHMMSS.csv`
- **Contents**: Signal quality metrics, packet statistics, and received telemetry

Each log entry includes a timestamp, allowing for accurate time-series analysis. The filenames include the rocket's power-on date and time obtained from GPS, making it easy to organize data from multiple flights.

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
| Buzzer | D10 | 1 |
| LED | D4 | 2 |

### Telemetry Receiver (ESP32) Pins
| Function | ESP32 Pin | GPIO |
|----------|-----------|------|
| Radio CS | 5 | 5 |
| Radio GDO0 | 4 | 4 |
| Display DC | 2 | 2 |
| Display CS | 15 | 15 |
| Display Reset | 4 | 4 |
| Touch CS | 21 | 21 |
| SD Card CS | 33 | 33 |
| MAX17043 SDA | 32 | 32 |
| MAX17043 SCL | 22 | 22 |

### Battery Monitoring and Charging

The receiver uses a MAX17043 battery fuel gauge connected via I2C to monitor the LiPo battery level. This is paired with a TC4056 LiPo battery charger module for safe charging.

#### TC4056 Connections:
- OUT+ → ESP32 VIN
- OUT+ → MAX17043 CELL+
- OUT- → ESP32 GND
- OUT- → MAX17043 CELL-
- B+ → LiPo Battery +
- B- → LiPo Battery -
- IN+ → USB 5V or external power +
- IN- → USB GND or external power -

#### MAX17043 Connections:
- VCC → 3.3V
- GND → GND
- SDA → GPIO32
- SCL → GPIO22
- ALERT → Not connected (optional interrupt pin)


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

The system includes an adaptive power management system that optimizes transmission power based on signal quality feedback:

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

## Recovery Buzzer System

The rocket includes a remotely activated buzzer to help locate it after landing:

1. **Buzzer Activation**
   - Available in both pre-launch and landed states
   - Controlled via a button on the main data display page
   - Commands are periodically resent every 5 seconds for reliability

2. **Power-Efficient Operation**
   - Buzzer beeps intermittently (100ms every 2 seconds) to conserve battery
   - Uses GPIO1 (D10) which minimizes conflicts with other components
   - Automatically deactivates when the receiver sends an OFF command

3. **Command Protocol**
   - Uses the command packet type with buzzer subtype
   - Includes transmitter ID to ensure commands only affect the selected rocket
   - Provides visual feedback on the receiver display (red when active, green when inactive)
   - SX1278 (LoRa): 2 dBm to 20 dBm
   - CC1101: -30 dBm to 10 dBm

4. **Benefits**
   - Extended battery life through optimized power usage
   - Maintains reliable communication at maximum possible distance
   - Reduces interference with other systems
   - Real-time power level displayed on receiver
   - Timestamped data

## Emergency Abort System

The system includes a remote emergency abort capability that allows the ground station operator to trigger parachute deployment in case of an emergency:

1. **Abort Activation**
   - Available only when the rocket is in flight (LAUNCH_STATE_LAUNCHED)
   - Controlled via a prominent red button on the main data display page
   - Features a two-step confirmation dialog to prevent accidental triggering
   - Commands are continuously resent every 100ms for maximum reliability

2. **Abort Mechanism**
   - Immediately activates both primary and backup parachute deployment relays
   - Forces the rocket state to LANDED to indicate flight termination
   - Activates the recovery buzzer automatically
   - Sets the LED to red to visually indicate abort status

3. **Safety Features**
   - Two-step confirmation dialog with clear visual warnings
   - Command parameter validation on the rocket side
   - Transmitter ID verification ensures commands only affect the selected rocket
   - Multiple transmissions increase reliability in noisy environments
   - Button is disabled after activation to prevent duplicate commands

4. **Use Cases**
   - Rocket flying off course or outside of designated area
   - Visible structural issues during flight
   - Sudden weather changes requiring mission termination
   - Any situation requiring immediate and safe recovery of the rocket

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
