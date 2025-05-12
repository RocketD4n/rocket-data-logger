/*
 *  TODO:
 */
 
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <RadioLib.h>

// CC1101 configuration
#define CC1101_CS 16  // D0 on NodeMCU
#define CC1101_GDO0 14 // D5 on NodeMCU

#include "rocket_telemetry_protocol.h"

// CC1101 instance
Module* module = new Module(CC1101_CS, CC1101_GDO0, RADIOLIB_NC, RADIOLIB_NC);
CC1101* radio = new CC1101(module);

// Global variables for altitude tracking
uint16_t maxAltitude = 0;
unsigned long lastFlush = 0;
bool launchDetected = false;
char line[256];

#define LED_PIN 2 // Built-in LED on NodeMCU (D4)
#define PRIMARY_RELAY_PIN 12 // D6 on NodeMCU
#define BACKUP_RELAY_PIN 13 // D7 on NodeMCU
#define ALTITUDE_DROP_THRESHOLD 10.0f // meters
#define BACKUP_DELAY 1000000 // 1 second in microseconds

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
File Textfile;
Adafruit_MPU6050 mpu;
float seaLevelPressure = 1013.25f; // Default value, will be updated with GPS altitude

// Track altitude state
float lastAltitude = 0.0f;
bool relayActive = false;
unsigned long primaryRelayActivationTime = 0;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(5, 4); // D1 (GPIO5) for RX, D2 (GPIO4) for TX

int countFiles(File dir) {
  int count = 0;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      count++;
    }
    entry.close();
  }
  return count;
}

void getAltitudeAndTemp(float& altitude, float& temp) {
  sensors_event_t event;
  bmp.getEvent(&event);
  altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
  bmp.getTemperature(&temp);
}

void setup() {
  // Initialize CC1101
  if (radio->begin() != RADIOLIB_ERR_NONE) {
    Serial.println("Failed to initialize CC1101");
    while (true) { delay(10); }
  }
  
  // Set frequency to 433.92 MHz
  radio->setFrequency(433.92);
  
  // Set bit rate to 2400 bps
  radio->setBitRate(2400);
  
  // Set preamble length and quality threshold
  radio->setPreambleLength(4, 0);
  
  // Set sync word
  radio->setSyncWord(0x55, 0x55);
  
  // Set node address
  radio->setNodeAddress(0x01);

  
  Serial.println("CC1101 initialized");
  
  Wire.begin(D2, D1);
  Serial.begin(74880);
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start with LED off

  // Initialize relay pins
  pinMode(PRIMARY_RELAY_PIN, OUTPUT);
  digitalWrite(PRIMARY_RELAY_PIN, LOW);  // Start with primary relay off
  pinMode(BACKUP_RELAY_PIN, OUTPUT);
  digitalWrite(BACKUP_RELAY_PIN, LOW);  // Start with backup relay off

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    return;
  }
  Serial.println("MPU6050 ready!");

  if (!bmp.begin()) {
    Serial.println("BMP180 not detected. Check wiring.");
    return;
  }
  Serial.println("BMP180 ready.");
  
  Serial.println("Initializing SD card");
  if (!SD.begin(15)) {
    Serial.println("Initialization failed!");
    return;
  }

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
  
  Serial.println("Initialization completed");

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
        packet.altitude = gps.altitude.meters();
        
        // Calculate checksum
        uint8_t checksum = 0;
        uint8_t* ptr = (uint8_t*)&packet;
        for (uint8_t i = 0; i < sizeof(packet) - 1; i++) {
            checksum ^= ptr[i];
        }
        packet.checksum = checksum;
        
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
    
    // Calculate checksum
    uint8_t checksum = 0;
    uint8_t* ptr = (uint8_t*)&packet;
    for (uint8_t i = 0; i < sizeof(packet) - 1; i++) {
        checksum ^= ptr[i];
    }
    packet.checksum = checksum;
    
    if (radio->transmit((uint8_t*)&packet, sizeof(packet)) != RADIOLIB_ERR_NONE) {
        Serial.println("Failed to send altitude data");
    } else {
        Serial.println("Sent altitude data");
    }
}

void loop() {
    // Send GPS data every second
    static unsigned long lastGpsSend = 0;
    if (millis() - lastGpsSend >= 1000) {
        sendGpsData();
        lastGpsSend = millis();
    }
    
    // Send altitude data every 100ms
    static unsigned long lastAltSend = 0;
    if (millis() - lastAltSend >= 100) {
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
    if (!launchDetected && aTotal > 1.7) { 
      Serial.println("Launch detected!");
      launchDetected = true;
      digitalWrite(LED_PIN, HIGH);  // Turn on LED when launch is detected
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
