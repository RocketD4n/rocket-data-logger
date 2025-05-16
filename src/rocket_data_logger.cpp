/*
 *  TODO:
 *  - receiver to send to rocket command to make sound
 *  - receiver to offer binding options
 *  - receiver to send to rocket parameters/constants
 */
 
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <MAX1704X.h>

// CC1101 configuration
#define CC1101_CS 16  // D0 on NodeMCU
#define CC1101_GDO0 14 // D5 on NodeMCU

#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"

// Radio instance using CC1101
Radio* radio = new CC1101Radio(CC1101_CS, CC1101_GDO0, RADIOLIB_NC);

// Global variables for altitude tracking
uint16_t maxAltitude = 0;
unsigned long lastFlush = 0;
bool launchDetected = false;
char line[256];
float maxG = 0.0f;

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

TinyGPSPlus gps;
SoftwareSerial gpsSerial(5, 4); // D1 (GPIO5) for RX, D2 (GPIO4) for TX

void getAltitudeAndTemp(float& altitude, float& temp) {
  sensors_event_t event;
  bmp.getEvent(&event);
  altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
  bmp.getTemperature(&temp);
}

void setup() {
  // Initialize RGB LED pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  setLedYellow();  // Yellow during initialization

  // Initialize radio
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
  
  Serial.println("Radio initialized");
  
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
  gpsSerial.begin(9600);
  
  // Wait for GPS fix and altitude
  while (true) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    
    if (gps.location.isUpdated() && gps.altitude.isValid()) {
      // Get current pressure
      sensors_event_t event;
      bmp.getEvent(&event);
      
      // Calculate sea level pressure using GPS altitude
      float currentPressure = event.pressure;
      float currentTemperature;
      bmp.getTemperature(&currentTemperature);
      
      // Calculate sea level pressure using the current altitude and temperature
      seaLevelPressure = bmp.pressureToAltitude(currentPressure, currentPressure, currentTemperature);
      Serial.print("Sea level pressure initialized: ");
      Serial.println(seaLevelPressure);
      Serial.print("Temperature: ");
      Serial.println(currentTemperature);
      break;
    }
    delay(1000);
  }
  
 
  // Get GPS date and time
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
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
    if (Serial1.available()) {
        gps.encode(Serial1.read());
    }
    
    if (gps.location.isValid()) {
        GpsDataPacket packet;
        packet.version = PROTOCOL_VERSION;
        packet.packetType = GPS_DATA_PACKET;
        packet.timestamp = millis();
        packet.latitude = gps.location.lat() * 10000000;
        packet.longitude = gps.location.lng() * 10000000;
        packet.altitude = (uint16_t)(gps.altitude.meters() * 10.0f);
        packet.batteryMillivolts = (uint16_t)(lipo.voltage() * 1000);
        packet.batteryPercent = (uint8_t)lipo.percent();
        packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(GpsDataPacket) - 1);
        if (radio->transmit((uint8_t*)&packet, sizeof(packet)) != RADIOLIB_ERR_NONE) {
            Serial.println("Failed to send GPS data");
        } else {
            Serial.println("Sent GPS data");
        }
    }
}

void sendAltitudeData() {
    float currentAltitude;
    float temp;
    getAltitudeAndTemp(currentAltitude, temp);
    
    if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
    }
    
    AltitudePacket packet;
    packet.version = PROTOCOL_VERSION;
    packet.packetType = ALTITUDE_PACKET;
    packet.timestamp = millis();
    packet.currentAltitude = currentAltitude;
    packet.maxAltitude = maxAltitude;
    packet.temperature = temp;
    packet.maxG = maxG;
    packet.launchState = launchDetected;
    
    // Calculate checksum
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(AltitudePacket) - 1);
    
    if (radio->transmit((uint8_t*)&packet, sizeof(packet)) != RADIOLIB_ERR_NONE) {
        Serial.println("Failed to send altitude data");
    } else {
        Serial.println("Sent altitude data");
    }
}

void loop() {
        // Get current battery percentage
    uint8_t batteryPercent = (uint8_t)lipo.percent();
    bool lowBatteryMode = batteryPercent < 50;
    
    // Send GPS data (every 10 seconds in low battery mode, every second in normal mode)
    static unsigned long lastGpsSend = 0;
    unsigned long gpsInterval = lowBatteryMode ? 10000 : 1000;
    
    if (millis() - lastGpsSend >= gpsInterval) {
        // Wake up radio if it was sleeping
        if (lowBatteryMode) {
            radio->wake();
            delay(10); // Small delay to ensure radio is ready
        }
        
        sendGpsData();
        lastGpsSend = millis();
        
        // Put radio to sleep in low battery mode
        if (lowBatteryMode) {
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
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    
    // Process IMU data
    sensors_event_t a, g;
    sensors_event_t temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Log data to SD card
    if (Textfile) {
        Textfile.print(micros());
        Textfile.print(",");
        Textfile.print(a.acceleration.x);
        Textfile.print(",");
        Textfile.print(a.acceleration.y);
        Textfile.print(",");
        Textfile.print(a.acceleration.z);
        Textfile.print(",");
        Textfile.print(g.gyro.x);
        Textfile.print(",");
        Textfile.print(g.gyro.y);
        Textfile.print(",");
        Textfile.print(g.gyro.z);
        
        float altitude, temp;
        getAltitudeAndTemp(altitude, temp);
        Textfile.print(",");
        Textfile.print(altitude);
        Textfile.print(",");
        Textfile.println(temp);
        
        if (millis() - lastFlush >= 1000) {
            Textfile.flush();
            lastFlush = millis();
        }
    }
    
    // Check for launch and handle deployment    
    if (launchDetected) {
        float currentAltitude;
        float currentTemp;
        getAltitudeAndTemp(currentAltitude, currentTemp);
        
        if (!relayActive && currentAltitude < lastAltitude - ALTITUDE_DROP_THRESHOLD) {
            relayActive = true;
            digitalWrite(PRIMARY_RELAY_PIN, HIGH);
            primaryRelayActivationTime = micros();
            Serial.println("Primary relay activated");
        }
        
        if (relayActive && micros() - primaryRelayActivationTime >= BACKUP_DELAY) {
            digitalWrite(BACKUP_RELAY_PIN, HIGH);
            Serial.println("Backup relay activated");
        }
        
        lastAltitude = currentAltitude;
    }
    
    // Compute total acceleration magnitude
    float aTotal = sqrt(a.acceleration.x * a.acceleration.x 
                      + a.acceleration.y * a.acceleration.y 
                      + a.acceleration.z * a.acceleration.z);
    maxG = max(maxG, aTotal);
    if (!launchDetected && aTotal > 1.7) { 
      Serial.println("Launch detected!");
      launchDetected = true;
      setLedBlue();  // Blue indicates launch detected
    }
    
    float altitude, temperature;
    getAltitudeAndTemp(altitude, temperature);

    // Check altitude drop
    if (lastAltitude > 0.0f) {
      float altitudeChange = lastAltitude - altitude;
      if (altitudeChange >= ALTITUDE_DROP_THRESHOLD && !relayActive) {
        // Altitude has dropped by the threshold amount
        relayActive = true;
        digitalWrite(PRIMARY_RELAY_PIN, HIGH);
        Serial.println("Altitude drop detected - Primary relay activated");
        primaryRelayActivationTime = micros();
      }
    }
    
    // Check if backup relay should be activated
    if (relayActive && !digitalRead(BACKUP_RELAY_PIN)) {
    unsigned long timeSincePrimary = micros() - primaryRelayActivationTime;
    if (timeSincePrimary >= BACKUP_DELAY) {
      digitalWrite(BACKUP_RELAY_PIN, HIGH);
      Serial.println("Backup relay activated");
    }
  }
  
  lastAltitude = altitude;

  while (gpsSerial.available() > 0) 
    gps.encode(gpsSerial.read());
  
 // gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.altitude.meters()
//  gps.date.year(), month(), day(), hour(), minute(), second(), 
  snprintf(line, sizeof(line), "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", micros(), 
            g.gyro.x, g.gyro.y, g.gyro.z,  // divide by 131.0 to convert to degrees per second
            a.acceleration.x, a.acceleration.y, a.acceleration.z,  // divide by 16384.0 to convert to accel in G
            gps.speed.mps(), gps.altitude.meters(),
            altitude, temperature);
  Textfile.println(line);
  Serial.println(line);
  if (micros() - lastFlush > 1000000) { // flush to write the file from the memory buffer at least every second
    Textfile.flush();
    lastFlush = micros();
  }
  delay(50); // millis
}
