/*
 *  TODO:
 *  - receiver to offer binding options
 *  - LVGL graphics on reciever screen:
Using LVGL (Light and Versatile Graphics Library) for rocket telemetry display could offer significant advantages over current TFT_eSPI implementation. 
Advantages of LVGL
1. Modern UI Components: LVGL provides ready-made widgets like buttons, charts, gauges, and tables that would enhance your telemetry display.
2. Event-Driven Architecture: Instead of polling for touch events, LVGL uses an event-driven system that's more efficient.
3. Animations and Transitions: Smooth transitions between pages and animated elements would make the UI feel more polished.
4. Theming Support: Easily create consistent visual styles across your application.
5. Memory Efficiency: LVGL is designed for embedded systems with limited resources.
 */
 
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085_U.h>
#include <MAX1704X.h>
#include <algorithm> // For std::max
#include "rocket_accelerometer.h"

// Radio configuration pins
#define RADIO_CS 16    // D0 on NodeMCU
#define RADIO_DIO0 14  // D5 on NodeMCU (DIO0 for SX1278, GDO0 for CC1101)
#define RADIO_DIO1 12  // D6 on NodeMCU (DIO1 for SX1262)
#define RADIO_BUSY 13  // D7 on NodeMCU (BUSY for SX1262)
#define RADIO_RST 15   // D8 on NodeMCU (RESET for SX1262/SX1278)

#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"
#include "sx1278_radio.h"
#include "sx1262_radio.h"

// Radio instance - will be initialized to SX1262 by default
Radio* radio = nullptr;

// This functionality has been moved to the Radio class

// Global variables for altitude tracking
uint16_t maxAltitude = 0;
unsigned long lastFlush = 0;
bool launchDetected = false;
bool landedDetected = false;
unsigned long landedTime = 0;
char line[256];

// Accelerometer instance
RocketAccelerometer accel;

// Buzzer control variables
bool buzzerActive = false;
unsigned long lastBuzzerToggle = 0;
const unsigned long BUZZER_CYCLE_INTERVAL = 2000; // 2 seconds between beeps
const unsigned long BUZZER_BEEP_DURATION = 100;  // 100ms beep duration

// Boot time tracking
uint32_t bootTimeEpoch = 0; // Boot time in seconds since epoch (from GPS)

// Adaptive power control parameters
const float TARGET_SNR = 15.0f;       // Target SNR for reliable communication (dB)
const float SNR_HYSTERESIS = 5.0f;    // SNR hysteresis to prevent frequent power changes
const float POWER_ADJUST_STEP = 2.0f; // Power adjustment step in dBm
float txPower = 10.0f;                // Current transmission power in dBm

// Frequency scanning parameters
const float FREQUENCY_SCAN_STEP = 100.0f;  // Step size in kHz for frequency scanning
float operatingFrequency = 863.0f;         // Default operating frequency in MHz
// Using radio->getAnnouncementFrequency() instead of hardcoded value
bool frequencyAcknowledged = false;        // Whether the selected frequency has been acknowledged
unsigned long lastFrequencyAnnounceTime = 0;
const unsigned long FREQUENCY_ANNOUNCE_INTERVAL = 2000; // Send frequency announcement every 2 seconds until acknowledged
uint8_t radioType = 2;                     // 0=CC1101, 1=SX1278, 2=SX1262 (default)

// RGB LED pins
#define LED_RED_PIN 0    // D3 on NodeMCU
#define LED_GREEN_PIN 2  // D4 on NodeMCU (Built-in LED)
#define LED_BLUE_PIN 13  // D7 on NodeMCU

// Buzzer pin - Using D10 (GPIO1/TX pin)
// Note: This will disable Serial output when buzzer is active, but that's acceptable
// for a recovery beacon that only activates after landing
#define BUZZER_PIN 1    // D10 (TX) on NodeMCU

#define PRIMARY_RELAY_PIN 5 // D1 on NodeMCU

// LED color functions
void setLedColor(bool red, bool green, bool blue) {
    digitalWrite(LED_RED_PIN, red);          // Active high (external LED)
    digitalWrite(LED_GREEN_PIN, green);      // Active high (external LED)
    digitalWrite(LED_BLUE_PIN, blue);         // Active high (external LED)
}

void setLedYellow() { setLedColor(true, true, false); }
void setLedRed() { setLedColor(true, false, false); }
void setLedGreen() { setLedColor(false, true, false); }
void setLedBlue() { setLedColor(false, false, true); }
#define BACKUP_RELAY_PIN 4 // D2 on NodeMCU
#define ALTITUDE_DROP_THRESHOLD 10.0f // meters
#define BACKUP_DELAY 1000000 // 1 second in microseconds

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
MAX1704X lipo(0.00125f);  // MAX17043 voltage resolution
File Textfile;
float seaLevelPressure = 1013.25f; // Default value, will be updated with GPS altitude

// Track altitude state
float lastAltitude = 0.0f;
bool relayActive = false;
unsigned long primaryRelayActivationTime = 0;

// Function prototypes
void setLedRed();
void setLedGreen();
void setLedBlue();
void setLedOff();
void logGpsData(const GpsDataPacket& packet);
void logAltitudeData(const AltitudePacket& packet);
void getAltitudeAndTemp(float& altitude, float& temp);
void sendGpsData();
void sendSystemData();
void sendAltitudeData();
void processSnrFeedback(uint8_t* buffer, size_t length);
void processCommandPacket(uint8_t* buffer, size_t length);
void adjustTransmissionPower(float snr);
void controlBuzzer(bool active);


// Get the unique chip ID for this transmitter
uint32_t transmitterId = 0;

// Control the buzzer
void controlBuzzer(bool active) {
  Serial.print("Buzzer ");
  Serial.println(active ? "ON" : "OFF");
  digitalWrite(BUZZER_PIN, active ? HIGH : LOW);
}

// Adjust transmission power based on SNR feedback
void adjustTransmissionPower(float snr) {
  // This function is called from processSnrFeedback
  // We're already using the Radio class's adaptive power control,
  // so we don't need to implement this separately
  Serial.print("SNR feedback received: ");
  Serial.print(snr);
  Serial.println(" dB");
  
  // The actual power adjustment is handled by the Radio class's
  // processSnrFeedbackAndAdjustPower method, which is called in the main loop
}

TinyGPSPlus gps;

char filename_base[24];

void getAltitudeAndTemp(float& altitude, float& temp) {
  sensors_event_t event;
  bmp.getEvent(&event);
  altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
  bmp.getTemperature(&temp);
}

void setup() {
  // Initialize serial communication
  Serial.begin(74880);
  Serial.println("Rocket Data Logger Initializing...");
  
  // Get ESP8266 chip ID as transmitter ID
  transmitterId = ESP.getChipId();
  Serial.print("Transmitter ID: ");
  Serial.println(transmitterId, HEX);
  
  // Initialize RGB LED pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  setLedYellow();  // Yellow during initialization
  
  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Start with buzzer off

  // Initialize radio (SX1262 by default)
  radio = new SX1262Radio(RADIO_CS, RADIO_RST, RADIO_DIO1, RADIO_BUSY);
  
  if (!radio->begin()) {
    Serial.println("Failed to initialize SX1262 radio, trying SX1278...");
    
    // Try SX1278 as fallback
    delete radio;
    radio = new SX1278Radio(RADIO_CS, RADIO_RST, RADIO_DIO0);
    radioType = 1; // SX1278
    
    if (!radio->begin()) {
      Serial.println("Failed to initialize SX1278 radio, trying CC1101...");
      
      // Try CC1101 as last resort
      delete radio;
      radio = new CC1101Radio(RADIO_CS, RADIO_DIO0, RADIOLIB_NC);
      radioType = 0; // CC1101
      
      if (!radio->begin()) {
        Serial.println("Failed to initialize any radio");
        while (true) { delay(10); }
      }
    }
  }
  
  // First configure radio for the announcement frequency
  if (!radio->configure(radio->getAnnouncementFrequency(), 250.0, 14)) {
    Serial.println("Failed to configure radio for announcements");
    while (true) { delay(10); }
  }
  
  // Scan for a clear frequency
  Serial.println("Scanning for a clear frequency...");
  float minFreq = radio->getMinimumFrequency();
  float maxFreq = radio->getMaximumFrequency();
  operatingFrequency = radio->scanFrequencyRange(minFreq, maxFreq, FREQUENCY_SCAN_STEP);
  
  Serial.print("Selected operating frequency: ");
  Serial.print(operatingFrequency);
  Serial.println(" MHz");
  
  // Configure adaptive power control
  radio->setAdaptivePowerParams(TARGET_SNR, SNR_HYSTERESIS, POWER_ADJUST_STEP);
  
  // Set the transmitter ID in the radio for SNR feedback filtering
  radio->setTransmitterId(transmitterId);
  
  Serial.println("Radio initialized with transmitter ID: 0x" + String(transmitterId, HEX));
  
  Wire.begin(D2, D1);
  Serial.begin(74880);
  

  // Initialize relay pins
  pinMode(PRIMARY_RELAY_PIN, OUTPUT);
  digitalWrite(PRIMARY_RELAY_PIN, LOW);  // Start with primary relay off
  pinMode(BACKUP_RELAY_PIN, OUTPUT);
  digitalWrite(BACKUP_RELAY_PIN, LOW);  // Start with backup relay off

  Serial.println("Initializing MPU6050...");
  if (!accel.begin()) {
    Serial.println("MPU6050 initialization failed!");
    setLedRed();  // Red indicates error
    while (1);
  }
  

  if (!bmp.begin()) {
    Serial.println("BMP180 not detected. Check wiring.");
    setLedRed();  // Red indicates error
    while (1);
  }
  Serial.println("BMP180 ready.");
  
  Serial.println("Initializing SD card");
  if (!SD.begin(15)) {
    Serial.println("Initialization failed!");
    setLedRed();  // Red indicates error
    while (1);
  }

  // Initialize MAX17043
  lipo.reset();
  lipo.quickstart();
  Serial.println("MAX17043 ready.");

  Serial.println("Waiting for GPS....");
  Serial.begin(9600);  // Hardware serial for GPS
  
  // Wait for GPS fix and altitude
  while (true) {
    while (Serial.available() > 0) {
      gps.encode(Serial.read());
    }
    
    if (gps.location.isUpdated() && gps.altitude.isValid()) {
      // Get current pressure
      sensors_event_t event;
      bmp.getEvent(&event);
      
      float currentPressure = event.pressure;
      float currentTemperature;
      bmp.getTemperature(&currentTemperature);
      
      // Calculate sea level pressure using the GPS altitude
      // P0 = P * exp(altitude / (29.3 * (T + 273.15)))
      float gpsAltitudeMeters = gps.altitude.meters();
      seaLevelPressure = currentPressure / pow(1.0 - (gpsAltitudeMeters / 44330.0), 5.255);
      
      Serial.print("Sea level pressure initialized: ");
      Serial.println(seaLevelPressure);
      Serial.print("Temperature: ");
      Serial.println(currentTemperature);
      break;
    }
    delay(1000);
  }
  
 
  // Get GPS date and time
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }
  
  // Wait for valid date and time from GPS to set boot time
  unsigned long waitStart = millis();
  while (!gps.time.isValid() || !gps.date.isValid()) {
    while (Serial.available() > 0) {
      gps.encode(Serial.read());
    }
    
    // Timeout after 30 seconds if we can't get valid time
    if (millis() - waitStart > 30000) {
      Serial.println("Timeout waiting for GPS time. Using default epoch time.");
      bootTimeEpoch = 0;
      break;
    }
    delay(100);
  }
  
  if (gps.time.isValid() && gps.date.isValid()) {
    // Use time.h to convert GPS date/time to epoch time (seconds since Jan 1, 1970)
    struct tm timeinfo;
    timeinfo.tm_year = gps.date.year() - 1900; // Years since 1900
    timeinfo.tm_mon = gps.date.month() - 1;    // Months since January (0-11)
    timeinfo.tm_mday = gps.date.day();         // Day of the month (1-31)
    timeinfo.tm_hour = gps.time.hour();        // Hours since midnight (0-23)
    timeinfo.tm_min = gps.time.minute();       // Minutes after the hour (0-59)
    timeinfo.tm_sec = gps.time.second();       // Seconds after the minute (0-59)
    timeinfo.tm_isdst = 0;                     // Daylight Saving Time flag
    
    // Convert to epoch time using mktime
    bootTimeEpoch = mktime(&timeinfo);
    
    Serial.print("Boot time set to epoch: ");
    Serial.println(bootTimeEpoch);
  }
  
  // Format filename as YYYYMMDD_HHMMSS.csv
  char filename[24];
  snprintf(filename_base, sizeof(filename_base), "%04d%02d%02d_%02d%02d%02d",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
  snprintf(filename, sizeof(filename), "%s.csv", filename_base);
  Textfile = SD.open(filename, FILE_WRITE);

  if (Textfile) {
    Serial.print("Writing header to file ");
    Serial.println(filename);
    Textfile.println("micros, XG, YG, ZG, XAc, YAc, ZAc, gps_speed, gps_altitude, altitude, temp");
    Textfile.print(micros());
    float altitude, temp;
    getAltitudeAndTemp(altitude, temp);
    Textfile.print(",0,0,0,0,0,0,");
    Textfile.print(altitude);
    Textfile.print(",");
    Textfile.println(temp);
    Textfile.flush();
    Serial.println("done");
  }

  // Set pointers to launch detection state variables
  accel.setLaunchDetectionPointers(&launchDetected, &landedDetected);
  
  // Calibrate the accelerometer at startup
  accel.calibrate(filename_base);

  Serial.println("Initialization completed");
  setLedGreen();  // Green indicates successful initialization with GPS fix
  Serial.println("Waiting for launch....");
}

void sendGpsData() {
    // Get GPS data
    while (Serial.available()) {
        gps.encode(Serial.read());
    }
    
    if (gps.location.isValid()) {
        GpsDataPacket packet;
        packet.version = PROTOCOL_VERSION;
        packet.packetType = PACKET_TYPE_GPS;
        packet.transmitterId = transmitterId;
        packet.latitude = gps.location.lat() * 10000000;
        packet.longitude = gps.location.lng() * 10000000;
        packet.altitude = gps.altitude.meters();
        packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
        
        radio->transmit((uint8_t*)&packet, sizeof(packet));
    }
}

void sendSystemData() {
  SystemDataPacket packet;
  packet.version = PROTOCOL_VERSION;
  packet.packetType = PACKET_TYPE_SYSTEM;
  packet.transmitterId = transmitterId;
  packet.uptime = millis();
  packet.batteryMillivolts = lipo.voltage() * 1000;
  packet.batteryPercent = lipo.percent();
  packet.txPower = txPower;
  packet.bootTime = bootTimeEpoch;
  
  // Set launch state using the enum values
  if (landedDetected) {
    packet.launchState = LAUNCH_STATE_LANDED;      // Landed
  } else if (launchDetected) {
    packet.launchState = LAUNCH_STATE_LAUNCHED;    // In flight
  } else {
    packet.launchState = LAUNCH_STATE_WAITING;    // Pre-launch
  }
  
  // Include tilt angle from calibration (0.5 degree resolution)
  packet.tiltAngle = accel.getTiltAngleInt();
  
  packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
  
  radio->transmit((uint8_t*)&packet, sizeof(SystemDataPacket));
}

void sendAltitudeData() {
  float altitude, temperature;
  getAltitudeAndTemp(altitude, temperature);

  if (altitude > maxAltitude) {
    maxAltitude = altitude;
  }

  AltitudePacket packet;
  packet.version = PROTOCOL_VERSION;
  packet.packetType = PACKET_TYPE_ALTITUDE;
  packet.transmitterId = transmitterId;
  packet.currentAltitude = altitude;
  packet.maxAltitude = maxAltitude;
  packet.temperature = temperature * 10; // Store with 0.1°C precision
  packet.maxG = accel.getMaxG() * 10; // Store with 0.1g precision
  packet.accelVelocity = (int16_t)(accel.getAccelVelocity() * 100); // Store in cm/s
  packet.baroVelocity = (int16_t)(accel.getBaroVelocity() * 100); // Store in cm/s
  
  // Get current orientation vector
  float orientX, orientY, orientZ;
  accel.getCurrentOrientation(orientX, orientY, orientZ);
  
  // Scale to int8_t range (-127 to 127)
  packet.orientationX = (int8_t)(orientX * 127.0f);
  packet.orientationY = (int8_t)(orientY * 127.0f);
  packet.orientationZ = (int8_t)(orientZ * 127.0f);

  packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
  
  radio->transmit((uint8_t*)&packet, sizeof(packet));
}

// Send frequency announcement packet
void sendFrequencyAnnouncement() {
  // Create frequency announcement packet
  FrequencyAnnouncePacket packet;
  packet.version = PROTOCOL_VERSION;
  packet.packetType = PACKET_TYPE_FREQ_ANNOUNCE;
  packet.transmitterId = transmitterId;
  packet.frequency = (uint32_t)(operatingFrequency * 1000.0f); // Convert MHz to kHz
  packet.radioType = radioType;
  
  // Calculate checksum
  packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
  
  // Send the packet
  if (radio->transmit((uint8_t*)&packet, sizeof(packet))) {
    Serial.print("Sent frequency announcement: ");
    Serial.print(operatingFrequency);
    Serial.println(" MHz");
  } else {
    Serial.println("Failed to send frequency announcement");
  }
}

// Process frequency acknowledgment packet
void processFrequencyAckPacket(uint8_t* buffer, size_t length) {
  // Verify packet length
  if (length < sizeof(FrequencyAckPacket)) {
    Serial.println("Frequency ack packet too short");
    return;
  }
  
  // Mark frequency as acknowledged
  frequencyAcknowledged = true;
  Serial.print("Frequency acknowledged: ");
  Serial.print(operatingFrequency);
  Serial.println(" MHz");
}

// Process SNR feedback packet
void processSnrFeedback(uint8_t* buffer, size_t length) {
  // Verify packet length
  if (length < sizeof(SnrFeedbackPacket)) {
    Serial.println("SNR feedback packet too short");
    return;
  }
  
  // Extract packet data
  SnrFeedbackPacket* packet = (SnrFeedbackPacket*)buffer;
  
  // Verify this is for our transmitter
  if (packet->transmitterId != transmitterId) {
    Serial.print("Ignoring SNR feedback for different transmitter: 0x");
    Serial.println(packet->transmitterId, HEX);
    return;
  }
  
  // Verify checksum
  uint8_t calculatedChecksum = calculateChecksum(buffer, sizeof(SnrFeedbackPacket) - 1);
  if (packet->checksum != calculatedChecksum) {
    Serial.println("SNR feedback packet has invalid checksum");
    return;
  }
  
  // Log the SNR value
  Serial.print("Received SNR feedback: ");
  Serial.print(packet->snrValue);
  Serial.println(" dB");
  
  // Adjust transmission power based on SNR feedback
  adjustTransmissionPower(packet->snrValue);
}

// Process command packet
void processCommandPacket(uint8_t* buffer, size_t length) {
  // Verify packet length
  if (length < sizeof(CommandPacket)) {
    Serial.println("Command packet too short");
    return;
  }
  
  // Extract packet data
  CommandPacket* packet = (CommandPacket*)buffer;
  
  // Verify this is for our transmitter
  if (packet->transmitterId != transmitterId) {
    Serial.print("Ignoring command for different transmitter: 0x");
    Serial.println(packet->transmitterId, HEX);
    return;
  }
  
  // Verify checksum
  uint8_t calculatedChecksum = calculateChecksum(buffer, sizeof(CommandPacket) - 1);
  if (packet->checksum != calculatedChecksum) {
    Serial.println("Command packet has invalid checksum");
    return;
  }
  
  // Process command based on subtype
  switch (packet->subType) {
    case COMMAND_SUBTYPE_BUZZER:
      Serial.print("Received buzzer command: ");
      Serial.println(packet->commandParam ? "ON" : "OFF");
      controlBuzzer(packet->commandParam != 0);
      break;
      
    case COMMAND_SUBTYPE_ABORT:
      Serial.println("RECEIVED EMERGENCY ABORT COMMAND!");
      // Trigger immediate parachute deployment
      digitalWrite(PRIMARY_RELAY_PIN, HIGH);
      digitalWrite(BACKUP_RELAY_PIN, HIGH);
      relayActive = true;
      primaryRelayActivationTime = micros();
      break;
      
    default:
      Serial.print("Unknown command subtype: ");
      Serial.println(packet->subType, HEX);
      break;
  }
}

void loop() {
  // Check if we need to send frequency announcements
  if (!frequencyAcknowledged && (millis() - lastFrequencyAnnounceTime >= FREQUENCY_ANNOUNCE_INTERVAL)) {
    // Make sure we're on the announcement frequency
    radio->configure(radio->getAnnouncementFrequency(), 250.0, 14);
    sendFrequencyAnnouncement();
    lastFrequencyAnnounceTime = millis();
    
    // Check for frequency acknowledgment
    uint8_t buffer[MAX_PACKET_SIZE];
    int bytesRead = radio->receive(buffer, MAX_PACKET_SIZE);
    
    if (bytesRead > 0) {
      // Check packet type
      if (buffer[0] == PROTOCOL_VERSION) {
        uint8_t packetType = buffer[1];
        
        switch (packetType) {
          case PACKET_TYPE_FREQ_ACK:
            processFrequencyAckPacket(buffer, bytesRead);
            break;
            
          case PACKET_TYPE_FEEDBACK:
            processSnrFeedback(buffer, bytesRead);
            break;
            
          case PACKET_TYPE_COMMAND:
            processCommandPacket(buffer, bytesRead);
            break;
        }
      }
    }
    
    // If frequency is acknowledged, switch to the operating frequency
    if (frequencyAcknowledged) {
      radio->configure(operatingFrequency, 250.0, txPower);
    }
    
    // Return early to avoid sending other packets during the announcement phase
    return;
  }
  
  // If frequency is not acknowledged yet, don't send regular telemetry
  if (!frequencyAcknowledged) {
    return;
  }
  
  // Check for incoming packets (SNR feedback, commands, etc.)
  uint8_t buffer[MAX_PACKET_SIZE];
  int bytesRead = radio->receive(buffer, MAX_PACKET_SIZE);
  
  if (bytesRead > 0) {
    // Check packet type
    if (buffer[0] == PROTOCOL_VERSION) {
      uint8_t packetType = buffer[1];
      
      switch (packetType) {
        case PACKET_TYPE_FEEDBACK:
          processSnrFeedback(buffer, bytesRead);
          break;
          
        case PACKET_TYPE_COMMAND:
          processCommandPacket(buffer, bytesRead);
          break;
      }
    }
  }
  
  // Check for SNR feedback and adjust power if needed
  radio->processSnrFeedbackAndAdjustPower(TARGET_SNR);
  
  // Get current battery percentage with hysteresis to prevent rapid mode switching
  static bool lowBatteryMode = false;
  uint8_t batteryPercent = (uint8_t)lipo.percent();
  
  // Add hysteresis: enter low power mode at 45%, exit at 55%
  if (lowBatteryMode && batteryPercent >= 55) {
    lowBatteryMode = false;
    Serial.println("Exiting low battery mode");
  } else if (!lowBatteryMode && batteryPercent <= 45) {
    lowBatteryMode = true;
    Serial.println("Entering low battery mode");
  }
  
  // Send GPS data (every 10 seconds in low battery mode, every second in normal mode)
  static unsigned long lastGpsSend = 0;
  unsigned long gpsInterval = lowBatteryMode ? 10000 : 1000;
  
  if (millis() - lastGpsSend >= gpsInterval) {
    // Wake up radio if it was sleeping
    if (lowBatteryMode) {
      radio->wake();
      delay(10); // Small delay to ensure radio is ready
    }
    
    // Send GPS data
    sendGpsData();
    sendSystemData();
    
    lastGpsSend = millis();
    
    // Put radio to sleep in low battery mode
    if (lowBatteryMode) {
      delay(50); // Allow time for the transmission to complete
      radio->sleep();
    }
  }
  
  // Send altitude data every 100ms (only in normal battery mode)
  static unsigned long lastAltSend = 0;
  if (!lowBatteryMode && millis() - lastAltSend >= 100) {
    sendAltitudeData();
    lastAltSend = millis();
  }
  
  // Process IMU data
  sensors_event_t a, g, temp;
  accel.mpu.getEvent(&a, &g, &temp);
  
  // Update velocity calculations
  accel.updateVelocity(a);
  
  float altitude, temperature;
  getAltitudeAndTemp(altitude, temperature);
  
  // Update barometric velocity calculation
  accel.updateBaroVelocity(altitude);
  
  // Check for launch and handle deployment    
  if (launchDetected) {
    if (!relayActive && altitude < lastAltitude - ALTITUDE_DROP_THRESHOLD) {
      relayActive = true;
      digitalWrite(PRIMARY_RELAY_PIN, HIGH);
      primaryRelayActivationTime = micros();
      Serial.println("Primary relay activated");
    }
    
    if (relayActive && micros() - primaryRelayActivationTime >= BACKUP_DELAY) {
      digitalWrite(BACKUP_RELAY_PIN, HIGH);
      Serial.println("Backup relay activated");
    }
    
    lastAltitude = altitude;
  }
  
  // Compute total acceleration magnitude
  float aTotal = sqrt(a.acceleration.x * a.acceleration.x 
                     + a.acceleration.y * a.acceleration.y 
                     + a.acceleration.z * a.acceleration.z);
  // maxG is now updated in the accelerometer class
  
  // Launch detection
  if (!launchDetected && aTotal > 1.7 * 9.8) { // 1.7g threshold
    Serial.println("Launch detected!");
    launchDetected = true;
    setLedBlue();  // Blue indicates launch detected
  }
  
  // Landed detection
  static unsigned long landingCheckStartTime = 0;
  static float initialLandingAltitude = 0;
  static float maxAltitudeDifference = 0;
  static bool landedDetected = false;
  
  if (launchDetected && relayActive && !landedDetected) {
    // First time we're checking for landing after parachute deployment
    if (landingCheckStartTime == 0) {
      landingCheckStartTime = millis();
      initialLandingAltitude = altitude;
    } else {
      // Track the maximum altitude difference we've seen
      float currentDifference = abs(altitude - initialLandingAltitude);
      maxAltitudeDifference = max(maxAltitudeDifference, currentDifference);
      
      // We've been monitoring for at least 10 seconds
      if (millis() - landingCheckStartTime > 10000) {
        // If max altitude difference is less than 2 meters in the last 10 seconds, consider it landed
        if (maxAltitudeDifference < 2.0) {
          landedDetected = true;
          Serial.println("Landing detected! Altitude stable near " + String(altitude) + "m");
          Serial.println("Maximum altitude variation: " + String(maxAltitudeDifference) + "m");
          setLedGreen();  // Green indicates landed
        } else {
          // Reset the check if we've seen too much variation
          landingCheckStartTime = millis();
          initialLandingAltitude = altitude;
          maxAltitudeDifference = 0;
          Serial.println("Reset landing detection, too much altitude variation: " + String(maxAltitudeDifference) + "m");
        }
      }
    }
  }
  
  // Check if backup relay should be activated
  if (relayActive) {
    unsigned long timeSincePrimary = millis() - primaryRelayActivationTime;
    if (timeSincePrimary >= BACKUP_DELAY) {
      digitalWrite(BACKUP_RELAY_PIN, HIGH);
      Serial.println("Backup relay activated");
    }
  }
  
  lastAltitude = altitude;
  
  // Determine if we should log to SD card based on flight state
  static unsigned long lastLogTime = 0;
  bool shouldLog = false;
  
  if (!launchDetected) {
    // Pre-launch: Log once every 10 seconds
    if (millis() - lastLogTime >= 10000) { // 10 seconds
      shouldLog = true;
      lastLogTime = millis();
    }
  } else if (!landedDetected) {
    // During flight: Log at full frequency
    shouldLog = true;
  } else {
    // Post-landing: Log once every 60 seconds
    if (millis() - lastLogTime >= 60000) { // 60 seconds
      shouldLog = true;
      lastLogTime = millis();
    }
  }
  
  if (shouldLog) {
    // Log data to SD card
    if (Textfile) {
      sprintf(line, "%lu,%0.6f,%0.6f,%0.1f,%0.1f,%0.1f,%0.1f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d",
              millis(),
              gps.location.lat(),
              gps.location.lng(),
              altitude,
              temperature,
              maxAltitude,
              accel.getMaxG(),
              accel.getAccelVelocity(),
              accel.getMaxAccelVelocity(),
              accel.getBaroVelocity(),
              accel.getMaxBaroVelocity(),
              launchDetected ? 1 : 0,
              landedDetected ? 1 : 0,
              (int)lipo.voltage(),
              (int)lipo.percent());
      Textfile.println(line);
    }
  }  
  // Always flush after logging in landed state to ensure data is written
  if (landedDetected || micros() - lastFlush > 1000000) { 
    Textfile.flush();
    lastFlush = micros();
  }
  
  // Handle buzzer control with power-efficient pattern if active
  static bool buzzerCurrentlyOn = false;
  static unsigned long lastBuzzerCheck = 0;
  
  // Only check buzzer state every 50ms to reduce overhead
  unsigned long currentTime = millis();
  if (currentTime - lastBuzzerCheck >= 50) {
    lastBuzzerCheck = currentTime;
    
    bool shouldBuzzerBeOn = false;
    
    if (buzzerActive) {
      unsigned long timeInCycle = currentTime % BUZZER_CYCLE_INTERVAL;
      // Buzzer should be on during the beep duration at the start of each cycle
      shouldBuzzerBeOn = (timeInCycle < BUZZER_BEEP_DURATION);
    }
    
    // Only change the buzzer state if needed
    if (shouldBuzzerBeOn != buzzerCurrentlyOn) {
      digitalWrite(BUZZER_PIN, shouldBuzzerBeOn ? HIGH : LOW);
      buzzerCurrentlyOn = shouldBuzzerBeOn;
    }
  }
  
  // Small delay to prevent CPU hogging
  delay(50);
}
