/*
 *  TODO:
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
#include "board_config.h"
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <RadioLib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <MAX1704X.h>
#include <PCF8574.h>

#if USE_SOFTWARE_SERIAL_GPS
#include <SoftwareSerial.h>
#endif

#include <algorithm> // For std::max
#include "rocket_accelerometer.h"

// Radio configuration pins are defined in board_config.h

#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"
#include "sx1278_radio.h"
#include "sx1262_radio.h"

MAX1704X lipo(0.00125f);  // MAX17043 voltage resolution
// Battery monitoring
float batteryLevel = 0.0f;
bool batteryMonitorAvailable = false;

// Function to safely read battery level
void updateBatteryLevel() {
  static bool initialized = false;
  static bool available = false;
  static unsigned long lastReadTime = 0;
  const unsigned long READ_INTERVAL = 60000; // Read once per minute
  
  // Only read at intervals
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime < READ_INTERVAL && lastReadTime != 0) {
    return;
  }
  lastReadTime = currentTime;
  
  // Try to initialize on first read
  if (!initialized) {
    initialized = true;
    Wire.beginTransmission(0x36);
    if (Wire.endTransmission() == 0) {
      try {
        lipo.reset();
        delay(10);
        lipo.quickstart();
        delay(10);
        available = true;
        batteryMonitorAvailable = true;
        Serial.println("MAX17043 initialized for battery monitoring");
      } catch (...) {
        available = false;
        batteryMonitorAvailable = false;
        Serial.println("Failed to initialize MAX17043");
      }
    } else {
      available = false;
      batteryMonitorAvailable = false;
    }
  }
  
  // Read battery level if available
  if (available) {
    try {
      batteryLevel = lipo.percent();
      Serial.print("Battery level: ");
      Serial.print(batteryLevel);
      Serial.println("%");
      return;
    } catch (...) {
      available = false;
      batteryMonitorAvailable = false;
      Serial.println("Error reading battery level");
    }
  }
  
  // If we get here, reading failed
  batteryLevel = 0.0f;
  batteryMonitorAvailable = false;
}

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

// PCF8574 I2C expander definitions are in board_config.h

// PCF8574 I2C expander instance
PCF8574 pcf8574(PCF8574_ADDRESS);

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
const float MAX_TX_POWER = 20.0f;     // Maximum transmission power in dBm for launch mode
const float NORMAL_TX_POWER = 10.0f;  // Normal transmission power in dBm
float savedTxPower = txPower;         // Save power level before launch

// Frequency scanning parameters
const float FREQUENCY_SCAN_STEP = 100.0f;  // Step size in kHz for frequency scanning
float operatingFrequency = 0.0f;           // Will be initialized with radio's default frequency
// Using radio->getDefaultFrequency() to get the default frequency for the current radio module
bool frequencyAcknowledged = false;        // Whether the selected frequency has been acknowledged
unsigned long lastFrequencyAnnounceTime = 0;
const unsigned long FREQUENCY_ANNOUNCE_INTERVAL = 2000; // Send frequency announcement every 2 seconds until acknowledged
uint8_t radioType = 2;                     // 0=CC1101, 1=SX1278, 2=SX1262 (default)

// LED pin is defined in board_config.h

// LED control functions using PCF8574

// LED control functions using PCF8574
void setLedOff() {
  pcf8574.digitalWrite(LED_RED_PIN, HIGH);   // Active LOW
  pcf8574.digitalWrite(LED_GREEN_PIN, HIGH); // Active LOW
  pcf8574.digitalWrite(LED_BLUE_PIN, HIGH);  // Active LOW
}

void setLedRed() {
  setLedOff(); // Turn all off first
  pcf8574.digitalWrite(LED_RED_PIN, LOW);    // Active LOW
}

void setLedGreen() {
  setLedOff(); // Turn all off first
  pcf8574.digitalWrite(LED_GREEN_PIN, LOW);  // Active LOW
}

void setLedBlue() {
  setLedOff(); // Turn all off first
  pcf8574.digitalWrite(LED_BLUE_PIN, LOW);   // Active LOW
}

void setLedYellow() {
  setLedOff(); // Turn all off first
  pcf8574.digitalWrite(LED_RED_PIN, LOW);    // Active LOW
  pcf8574.digitalWrite(LED_GREEN_PIN, LOW);  // Active LOW
}

// Backup relay pin now on PCF8574 I2C expander
#define ALTITUDE_DROP_THRESHOLD 10.0f // meters
#define BACKUP_DELAY 1000000 // 1 second in microseconds

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
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
  pcf8574.digitalWrite(BUZZER_PIN, active ? HIGH : LOW);
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

// GPS pins are defined in board_config.h

// GPS instance
TinyGPSPlus gps;

#if USE_SOFTWARE_SERIAL_GPS
// Use SoftwareSerial for GPS (ESP32-C3)
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN); // RX, TX
#define GPS_SERIAL gpsSerial
#else
// Use Hardware Serial2 for GPS (regular ESP32)
#define GPS_SERIAL Serial2
#endif

char filename_base[24];

void getAltitudeAndTemp(float& altitude, float& temp) {
  sensors_event_t event;
  bmp.getEvent(&event);
  altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
  bmp.getTemperature(&temp);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Rocket Data Logger Initializing...");
  
  // Get ESP32-C3 chip ID as transmitter ID
  transmitterId = ESP.getEfuseMac() & 0xFFFFFFFF;
  Serial.print("Transmitter ID: ");
  Serial.println(transmitterId, HEX);
  
  // Initialize LED pin
  // Initialize PCF8574 and configure LED pins
  if (!pcf8574.begin()) {
    Serial.println("Failed to initialize PCF8574 expander! No relay, buzzer or LED will be available.");
  }
  
  // Set LED pins as outputs (HIGH = OFF for common anode RGB LED)
  pcf8574.pinMode(LED_RED_PIN, OUTPUT);
  pcf8574.pinMode(LED_GREEN_PIN, OUTPUT);
  pcf8574.pinMode(LED_BLUE_PIN, OUTPUT);
  setLedOff();   // Initialize all LEDs to off
  setLedBlue();  // Blue LED on during initialization
  
  // Initialize buzzer pin
  pcf8574.pinMode(BUZZER_PIN, OUTPUT);
  pcf8574.digitalWrite(BUZZER_PIN, LOW);  // Start with buzzer off

  // Initialize radio (SX1278 as primary)
  radio = new SX1278Radio(RADIO_CS, RADIO_RST, RADIO_DIO0);
  radioType = 1; // SX1278
  
  // Perform hard reset
  Serial.println(F("Performing hard radio reset..."));
  pinMode(RADIO_RST, OUTPUT);
  digitalWrite(RADIO_RST, LOW);
  delay(10);
  digitalWrite(RADIO_RST, HIGH);
  delay(100);  // Give time for the radio to reset
  Serial.println(F("  - Reset sequence completed"));
  
  Serial.println(F("3. Initializing radio..."));
  
  if (!radio->begin()) {
    Serial.println("Failed to initialize SX1278 radio, trying SX1262...");
    
    // Try SX1262 as fallback
    delete radio;
    radio = new SX1262Radio(RADIO_CS, RADIO_RST, RADIO_DIO1, RADIO_BUSY);
    radioType = 2; // SX1262
    
    if (!radio->begin()) {
      Serial.println("Failed to initialize SX1262 radio, trying CC1101...");
      
      // Try CC1101 as last resort
      delete radio;
      radio = new CC1101Radio(RADIO_CS, RADIO_DIO0, RADIOLIB_NC);
      radioType = 0; // CC1101
      
      if (!radio->begin()) {
        Serial.println("Failed to initialize any radio");
        while (true) { delay(10); }
      } else {
        // Initialize operating frequency with CC1101's default
        operatingFrequency = radio->getDefaultFrequency();
      }
    } else {
      // Initialize operating frequency with SX1262's default
      operatingFrequency = radio->getDefaultFrequency();
    }
  } else {
    // Initialize operating frequency with SX1278's default
    operatingFrequency = radio->getDefaultFrequency();
  }
  
  // First configure radio for the announcement frequency
  Serial.println(F("\n=== Radio Configuration ==="));
  Serial.print(F("Operating frequency: "));
  Serial.println(operatingFrequency);
  Serial.print(F("Announcement frequency: "));
  float announceFreq = radio->getAnnouncementFrequency();
  Serial.println(announceFreq);
  
  // Debug print radio type
  Serial.print(F("Radio type: "));
  if (radioType == 1) Serial.println(F("SX1278"));
  else if (radioType == 2) Serial.println(F("SX1262"));
  else if (radioType == 0) Serial.println(F("CC1101"));
  
  Serial.println(F("Data bandwidth: 250"));
  Serial.println(F("Power: 14"));
  
  // Try to configure with debug info
  Serial.println(F("Configuring radio with announcement frequency..."));
  if (!radio->configure(announceFreq, 250.0, 14)) {
    Serial.println(F("Failed to configure radio for announcements"));
    Serial.println(F("Trying with operating frequency instead..."));
    if (!radio->configure(operatingFrequency, 250.0, 14)) {
      Serial.println(F("Failed to configure radio with operating frequency"));
      while (true) { delay(10); }
    } else {
      Serial.println(F("Successfully configured with operating frequency"));
    }
  } else {
    Serial.println(F("Successfully configured with announcement frequency"));
  }
  
  // Scan for a clear frequency
  Serial.println("Scanning for a clear frequency...");
  float minFreq = radio->getMinimumFrequency();
  float maxFreq = radio->getMaximumFrequency();
  operatingFrequency = radio->scanFrequencyRange(minFreq, maxFreq, FREQUENCY_SCAN_STEP);
  
  Serial.print("Selected operating frequency: ");
  Serial.print(operatingFrequency);
  Serial.println(" MHz");
  
  // Configure radio with the selected frequency
  if (!radio->configure(operatingFrequency, 250.0, 14)) {
    Serial.println("Failed to configure radio with selected frequency");
    while (true) { delay(10); }
  }
  
  // Set adaptive power control parameters
  radio->setAdaptivePowerParams(TARGET_SNR, SNR_HYSTERESIS, POWER_ADJUST_STEP);
  
  // Set transmitter ID for this device
  radio->setTransmitterId(transmitterId);
  
  Serial.println("Radio initialized with transmitter ID: 0x" + String(transmitterId, HEX));
  
  Wire.begin(I2C_SDA, I2C_SCL); // SDA and SCL pins from board_config.h
  Serial.begin(115200);
  

  // Initialize PCF8574 I2C expander for relay control
  if (pcf8574.begin()) {
    Serial.println("PCF8574 I2C expander initialized successfully");
    // Set relay pins as outputs and turn them off
    pcf8574.pinMode(PRIMARY_RELAY_PIN, OUTPUT);
    pcf8574.digitalWrite(PRIMARY_RELAY_PIN, LOW);  // Start with primary relay off
    pcf8574.pinMode(BACKUP_RELAY_PIN, OUTPUT);
    pcf8574.digitalWrite(BACKUP_RELAY_PIN, LOW);  // Start with backup relay off
  } else {
    Serial.println("ERROR: Could not initialize PCF8574 I2C expander");
  }

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
  if (!SD.begin(SD_CS_PIN)) { // SD Card CS pin from board_config.h
    Serial.println("Initialization failed!");
    setLedRed();  // Red indicates error
    while (1);
  }

  // Check for MAX17043 presence (non-blocking)
  Serial.print("Checking for MAX17043... ");
  bool batteryMonitorAvailable = false;
  
  // Simple I2C scan to check for device presence
  // Check for MAX17043 presence (non-blocking)
  Wire.beginTransmission(0x36);  // MAX17043 I2C address
  if (Wire.endTransmission() == 0) {
    Serial.println("found");
  } else {
    Serial.println("not found");
  }
  
  // Note: We'll initialize the MAX17043 on-demand when reading battery level
  // This prevents crashes during boot if the battery isn't connected

  // Initialize GPS serial 
  // Print GPS configuration
  Serial.println("GPS Configuration:");
  #if USE_SOFTWARE_SERIAL_GPS
    Serial.println("- Using SoftwareSerial");
  #else
    Serial.println("- Using Hardware Serial2");
  #endif
  
  // Try different baud rates to find the correct one
  const long baudRates[] = {9600, 115200, 57600, 38400, 19200, 4800};
  const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);
  bool foundBaudRate = false;
  
  for (int i = 0; i < numBaudRates; i++) {
    long currentBaud = baudRates[i];
    
    // Close and reopen serial port
    GPS_SERIAL.end();
    delay(100);
    
    #if !USE_SOFTWARE_SERIAL_GPS
      GPS_SERIAL.begin(currentBaud, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
    #else
      GPS_SERIAL.begin(currentBaud);
    #endif
    
    delay(100);
    
    // Clear any buffered data
    while (GPS_SERIAL.available() > 0) {
      GPS_SERIAL.read();
    }
    
    // Send a test command that should generate a response
    GPS_SERIAL.println("$PMTK605*31");
    
    // Wait for response
    unsigned long startTime = millis();
    bool gotResponse = false;
    
    while (millis() - startTime < 500) {  // Wait up to 500ms for response
      if (GPS_SERIAL.available() > 0) {
        char c = GPS_SERIAL.read();
        if (c == '$') {  // Start of NMEA sentence
          gotResponse = true;
          break;
        }
      }
    }
    
    if (gotResponse) {
      Serial.print("Found GPS at ");
      Serial.print(currentBaud);
      Serial.println(" baud");
      foundBaudRate = true;
      
      // Configure GPS module
      GPS_SERIAL.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // GGA and RMC only
      delay(100);
      GPS_SERIAL.println("$PMTK220,200*2C"); // 5Hz update rate
      delay(100);
      GPS_SERIAL.println("$PMTK313,1*2E"); // Enable SBAS
      delay(100);
      
      // Clear any remaining data
      while (GPS_SERIAL.available() > 0) {
        GPS_SERIAL.read();
      }
      
      break;
    } else {
      Serial.print("No response at ");
      Serial.print(currentBaud);
      Serial.println(" baud");
    }
  }
  
  if (!foundBaudRate) {
    Serial.println("Could not communicate with GPS module! Check wiring and power.");
    Serial.println("Tried baud rates: 9600, 115200, 57600, 38400, 19200, 4800");
    
    // Blink LED to indicate error
    while (true) {
      digitalWrite(LED_BLUE_PIN, HIGH);
      delay(100);
      digitalWrite(LED_BLUE_PIN, LOW);
      delay(100);
    }
  }
  
  // Initialize GPS serial with error checking
  bool gpsSerialStarted = false;
  
  #if USE_SOFTWARE_SERIAL_GPS
    Serial.println("Initializing SoftwareSerial for GPS...");
    GPS_SERIAL.begin(9600);  // Software Serial for GPS on ESP32-C3
    gpsSerialStarted = true;
  #else
    // Verify communication
    unsigned long startTime = millis();
    bool receivedData = false;
  
    while (millis() - startTime < 2000) { // Wait up to 2 seconds for data
      if (GPS_SERIAL.available() > 0) {
        receivedData = true;
        break;
      }
      delay(10);
    }
  
    if (receivedData) {
      Serial.println("GPS configured successfully");
      gpsSerialStarted = true;
    } 
  #endif
  
  if (!gpsSerialStarted) {
    Serial.println("ERROR: Failed to initialize GPS serial!");
    Serial.println("Please check the following:");
    Serial.println("1. GPS module is properly connected to the correct pins");
    Serial.println("2. GPS module is powered (check voltage levels)");
    Serial.println("3. Correct baud rate is used (try 9600, 57600, 115200, or 4800)");
    Serial.println("4. TX/RX pins are not swapped");
    
    while(1) {
      // Blink blue LED to indicate error
      digitalWrite(LED_BLUE_PIN, HIGH);
      delay(100);
      digitalWrite(LED_BLUE_PIN, LOW);
      delay(900);
      
      // Still try to read GPS in case it starts working
      if (GPS_SERIAL.available() > 0) {
        char c = GPS_SERIAL.read();
        if (c == '$') {  // Start of NMEA sentence
          Serial.println("\nGPS data detected! Resuming...");
          gpsSerialStarted = true;
          break;
        }
      }
    }
  } else {
    Serial.println("GPS serial initialized successfully");
  }
  
  // Wait for GPS fix and altitude
  unsigned long lastStatusTime = 0;
  byte lastSatellites = 0;
  unsigned long lastDataTime = millis();
  
  // Clear any buffered data
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    Serial.print(c);
  } 
  // Request GPS module version
 // GPS_SERIAL.println("$PMTK605*31\r\n");
  
  while (true) {
    // Process all available GPS data
    while (GPS_SERIAL.available() > 0) {
      lastDataTime = millis();
      
      // Read and process a byte
      if (gps.encode(GPS_SERIAL.read())) {
        // Got a complete NMEA sentence
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint >= 1000) {  // Update status once per second
          lastPrint = millis();
          
          // Print GPS status
          if (gps.location.isValid()) {
            Serial.print("Fix: ");
            Serial.print(gps.satellites.value());
            Serial.print(" sats, HDOP=");
            Serial.print(gps.hdop.hdop(), 1);
            Serial.print(", Lat=");
            Serial.print(gps.location.lat(), 6);
            Serial.print(", Lon=");
            Serial.print(gps.location.lng(), 6);
            Serial.print(", Alt=");
            Serial.print(gps.altitude.meters());
            Serial.println("m");
            
            // Toggle LED to show activity
            static bool ledState = false;
            digitalWrite(LED_BLUE_PIN, ledState ? HIGH : LOW);
            ledState = !ledState;
          } else {
            Serial.println("Waiting for GPS fix...");
          }
        }
      }
    }
    
    // Check for GPS timeout
    if (millis() - lastDataTime > 5000) {
      if (millis() - lastStatusTime >= 1000) {
        lastStatusTime = millis();
        Serial.print("No GPS data for ");
        Serial.print((millis() - lastDataTime) / 1000);
        Serial.println(" seconds");
      }
    }
    
    // Small delay to prevent busy-waiting
    delay(1);
    
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
      Serial.print(currentTemperature);
      Serial.print(" GPS Altitude: ");
      Serial.println(gpsAltitudeMeters);
      break;
    }
    delay(1000);
  }
  
 
  // Get GPS date and time
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }
  
  // Wait for valid date and time from GPS to set boot time
  unsigned long waitStart = millis();
  while (!gps.time.isValid() || !gps.date.isValid()) {
    while (GPS_SERIAL.available() > 0) {
      gps.encode(GPS_SERIAL.read());
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
  snprintf(filename_base, sizeof(filename_base), "/%04d%02d%02d_%02d%02d%02d",
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
    while (GPS_SERIAL.available()) {
        gps.encode(GPS_SERIAL.read());
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
  static uint8_t previousLaunchState = 0;
  if (landedDetected) {
    packet.launchState = LAUNCH_STATE_LANDED;      // Landed
    // If we've just landed, restore normal power mode
    if (previousLaunchState == LAUNCH_STATE_LAUNCHED) {
      // Restore previous power setting
      txPower = savedTxPower;
      // Reconfigure radio with the restored power
      radio->configure(operatingFrequency, 250.0, txPower);
      Serial.println("Landed: Restored normal power mode");
    }
  } else if (launchDetected) {
    packet.launchState = LAUNCH_STATE_LAUNCHED;    // In flight
    // If we've just launched, switch to maximum power
    if (previousLaunchState != LAUNCH_STATE_LAUNCHED) {
      // Save current power setting before switching to max
      savedTxPower = txPower;
      // Set to maximum power during flight
      txPower = MAX_TX_POWER;
      // Reconfigure radio with the new power
      radio->configure(operatingFrequency, 250.0, txPower);
      Serial.println("Launched: Switched to maximum power mode");
    }
  } else {
    packet.launchState = LAUNCH_STATE_WAITING;    // Pre-launch
  }
  
  // Save current state for next comparison
  previousLaunchState = packet.launchState;
  
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
    Serial.print(" MHz. Id:");
    Serial.println(transmitterId);
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
      pcf8574.digitalWrite(PRIMARY_RELAY_PIN, HIGH);
      pcf8574.digitalWrite(BACKUP_RELAY_PIN, HIGH);
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
  
  // Update battery level (safe function that handles initialization and errors)
  updateBatteryLevel();
  
  // Check for SNR feedback and adjust power if needed
  radio->processSnrFeedbackAndAdjustPower(TARGET_SNR);
  
  // Get current battery percentage with hysteresis to prevent rapid mode switching
  static bool lowBatteryMode = false;
  uint8_t batteryPercent = (uint8_t)batteryLevel;
  
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
      pcf8574.digitalWrite(PRIMARY_RELAY_PIN, HIGH);
      primaryRelayActivationTime = micros();
      Serial.println("Primary relay activated");
    }
    
    if (relayActive && micros() - primaryRelayActivationTime >= BACKUP_DELAY) {
      pcf8574.digitalWrite(BACKUP_RELAY_PIN, HIGH);
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
      pcf8574.digitalWrite(BACKUP_RELAY_PIN, HIGH);
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
      pcf8574.digitalWrite(BUZZER_PIN, shouldBuzzerBeOn ? HIGH : LOW);
      buzzerCurrentlyOn = shouldBuzzerBeOn;
    }
  }
  
  // Small delay to prevent CPU hogging
  delay(50);
}
