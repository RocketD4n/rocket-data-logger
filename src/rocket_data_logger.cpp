/*
 *  TODO:
 *  - receiver to send to rocket command to make sound
 *  - receiver to offer binding options
 *  - LVGL graphics on reciever screen
 */
 
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085_U.h>
#include <MAX1704X.h>
#include <algorithm> // For std::max

// CC1101 configuration
#define CC1101_CS 16  // D0 on NodeMCU
#define CC1101_GDO0 14 // D5 on NodeMCU

#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"

// Radio instance using CC1101
Radio* radio = nullptr;

// This functionality has been moved to the Radio class

// Global variables for altitude tracking
uint16_t maxAltitude = 0;
unsigned long lastFlush = 0;
bool launchDetected = false;
bool landedDetected = false;
unsigned long landedTime = 0;
char line[256];
float maxG = 0.0f;

// Boot time tracking
uint32_t bootTimeEpoch = 0; // Boot time in seconds since epoch (from GPS)

// Adaptive power control parameters
const float TARGET_SNR = 15.0f;       // Target SNR for reliable communication (dB)
const float SNR_HYSTERESIS = 5.0f;    // SNR hysteresis to prevent frequent power changes
const float POWER_ADJUST_STEP = 2.0f; // Power adjustment step in dBm
float txPower = 10.0f;                // Current transmission power in dBm

// RGB LED pins
#define LED_RED_PIN 0    // D3 on NodeMCU
#define LED_GREEN_PIN 2  // D4 on NodeMCU (Built-in LED)
#define LED_BLUE_PIN 13  // D7 on NodeMCU

#define PRIMARY_RELAY_PIN 12 // D6 on NodeMCU

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
#define BACKUP_RELAY_PIN 13 // D7 on NodeMCU
#define ALTITUDE_DROP_THRESHOLD 10.0f // meters
#define BACKUP_DELAY 1000000 // 1 second in microseconds

Adafruit_MPU6050 mpu;
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
void adjustTransmissionPower(float snr);

// Get the unique chip ID for this transmitter
uint32_t transmitterId = 0;

TinyGPSPlus gps;
// Using hardware Serial for GPS

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

  // Initialize radio (CC1101)
  radio = new CC1101Radio(CC1101_CS, CC1101_GDO0, RADIOLIB_NC);
  
  if (!radio->begin()) {
    Serial.println("Failed to initialize radio");
    while (true) { delay(10); }
  }
  
  // Configure radio parameters
  // Using 433.92 MHz frequency, 250 kHz bandwidth (adjust if needed), and 10 dBm power
  if (!radio->configure(433.92, 250.0, 10)) {
    Serial.println("Failed to configure radio");
    while (true) { delay(10); }
  }
  
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
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    setLedRed();  // Red indicates error
    while (1);
  }
  Serial.println("MPU6050 ready!");

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
  snprintf(filename, sizeof(filename), "%04d%02d%02d_%02d%02d%02d.csv",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
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
  packet.batteryMillivolts = (uint16_t)(lipo.voltage() * 1000);
  packet.batteryPercent = (uint8_t)lipo.percent();
  packet.txPower = (int8_t)radio->getCurrentPower();
  packet.bootTime = bootTimeEpoch;
  packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
  
  radio->transmit((uint8_t*)&packet, sizeof(packet));
}

void sendAltitudeData() {
  float altitude;
  float temperature;
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
  packet.maxG = maxG * 10; // Store with 0.1g precision
  packet.launchState = launchDetected ? 1 : 0;
  packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
  
  radio->transmit((uint8_t*)&packet, sizeof(packet));
}

void loop() {
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
  
  // Process GPS data
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }
  
  // Process any SNR feedback packets and adjust power if needed
  if (radio->available()) {
    radio->processSnrFeedbackAndAdjustPower(TARGET_SNR);
    // Update our local txPower variable to match the radio's current power
    txPower = radio->getCurrentPower();
  }
  
  // Process IMU data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float altitude, temperature;
  getAltitudeAndTemp(altitude, temperature);
  
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
  // Use Arduino's max function with explicit type casting
  maxG = max(maxG, (float)(aTotal / 9.8f)); // Convert to g units
  
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
  
  // Determine if we should log to SD card based on landed state
  static unsigned long lastLogTime = 0;
  bool shouldLog = false;
  
  if (!landedDetected) {
    shouldLog = true;
  } else {
    if (millis() - lastLogTime >= 60000) { // 60 seconds
      shouldLog = true;
      lastLogTime = millis();
    }
  }
  
  if (shouldLog) {
    // Log data to SD card
    if (Textfile) {
      sprintf(line, "%lu,%0.6f,%0.6f,%0.1f,%0.1f,%0.1f,%0.1f,%d,%d,%d,%d",
              millis(),
              gps.location.lat(),
              gps.location.lng(),
              altitude,
              temperature,
              maxAltitude,
              maxG,
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
  
  // Small delay to prevent CPU hogging
  delay(50);
}
