#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <Wire.h>
#include <MAX1704X.h>
#include <SD.h>
#include <time.h>
#include <RadioLib.h>
#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"
#include "sx1278_radio.h"
#include "sx1262_radio.h"
#include "rocket_monitor_screen.h"

// Radio configuration pins for ESP32-S3
#define RADIO_CS 5     // GPIO5
#define RADIO_DIO0 4   // GPIO4 (DIO0 for SX1278, GDO0 for CC1101)
#define RADIO_DIO1 6   // GPIO6 (DIO1 for SX1262)
#define RADIO_BUSY 3   // GPIO3 (BUSY for SX1262)
#define RADIO_RST 2    // GPIO2 (RESET for SX1262/SX1278)

// Radio instance - will be initialized to SX1262 by default
Radio* radio = nullptr;

// Direct access to SX1278 module for testing
Module* testModule = nullptr;
SX1278* testLora = nullptr;

// Battery monitor configuration for ESP32-S3
#define MAX17043_SDA 41  // SDA pin
#define MAX17043_SCL 40  // SCL pin
#define LOW_BATTERY_THRESHOLD 45.0
MAX1704X batteryMonitor(0.00125f);
bool batteryMonitorEnabled = false;

// Display configuration for ESP32-S3
#define TFT_CS    10
#define TFT_DC     9
#define TFT_MOSI  11
#define TFT_CLK   12
#define TFT_RST   14
#define TFT_MISO  13

// Touch screen configuration for ESP32-S3
#define TOUCH_CS 7
#define TOUCH_IRQ 8
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// Initialize TFT display
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

// Create the display manager instance
RocketMonitorScreen display(tft, ts);

// Forward declarations of functions
void sendSnrFeedback();
void sendBuzzerCommand(uint32_t transmitterId, bool activate);
void sendAbortCommand(uint32_t transmitterId);
void processGpsPacket(uint8_t* buffer, size_t length);
void processAltitudePacket(uint8_t* buffer, size_t length);
void processSystemPacket(uint8_t* buffer, size_t length);
void processFrequencyAnnouncePacket(uint8_t* buffer, size_t length);
void sendFrequencyAcknowledgment(uint32_t transmitterId, float frequency);
void switchToTransmitterFrequency(uint32_t transmitterId);
void lowBatteryShutdown();
void startSimulationMode();
void updateSimulation();
void testTouchScreen();

// Touch test mode
bool touchTestMode = true; // Set to true to enable touch test mode
unsigned long lastTouchCheck = 0;
const unsigned long TOUCH_CHECK_INTERVAL = 100; // Check touch every 100ms

// SNR feedback control
unsigned long lastSnrFeedbackTime = 0;
const unsigned long SNR_FEEDBACK_INTERVAL = 1000; // Send SNR feedback every 1 second

// Frequency scanning and announcement parameters
unsigned long lastFrequencyAckTime = 0;
const unsigned long FREQUENCY_ACK_INTERVAL = 500; // Send frequency ack every 500ms until confirmed
bool listeningForAnnouncements = true; // Start by listening for frequency announcements

// SD card configuration for ESP32-S3
#define SD_CS 38 // SD card CS pin for ESP32-S3
bool sdCardAvailable = false;

// Logging variables
float lastLoggedAltitude = 0.0f;
const float ALTITUDE_LOG_THRESHOLD = 1.0f; // Log when altitude changes by 1 meter

// File handles for logging
File gpsLogFile;
File altitudeLogFile;
File systemLogFile;

// Logging file paths
String gpsLogPath;
String altitudeLogPath;
String systemLogPath;

// Rocket boot time for filename generation
uint32_t rocketBootTime = 0;

// Store the latest GPS and system data for logging when altitude changes
GpsDataPacket latestGpsPacket;
bool hasNewGpsData = false;
SystemDataPacket latestSystemPacket;
bool hasNewSystemData = false;

// Function to create log files for a specific transmitter
void createLogFiles(uint32_t transmitterId, uint32_t bootTime) {
    // Close any open files first
    if (gpsLogFile) gpsLogFile.close();
    if (altitudeLogFile) altitudeLogFile.close();
    if (systemLogFile) systemLogFile.close();
    
    if (!sdCardAvailable) return;
    
    // Get transmitter name from the display class
    String transmitterName = display.getTransmitterName(transmitterId);
    
    // Format bootTime as date_time string
    char dateTimeStr[20];
    time_t bootTimeT = bootTime;
    struct tm *tm = gmtime(&bootTimeT);
    
    if (tm != NULL && bootTime > 0) {
        strftime(dateTimeStr, sizeof(dateTimeStr), "%Y%m%d_%H%M%S", tm);
    } else {
        // Fallback if bootTime is invalid
        sprintf(dateTimeStr, "unknown_time");
    }
    
    // Create filenames
    gpsLogPath = "/" + transmitterName + "_" + String(dateTimeStr) + "_gps.csv";
    altitudeLogPath = "/" + transmitterName + "_" + String(dateTimeStr) + "_altitude.csv";
    systemLogPath = "/" + transmitterName + "_" + String(dateTimeStr) + "_system.csv";
    
    // Open files and write headers
    gpsLogFile = SD.open(gpsLogPath, FILE_WRITE);
    if (gpsLogFile) {
        gpsLogFile.println("receiver_time,latitude,longitude,altitude");
        gpsLogFile.flush();
    }
    
    altitudeLogFile = SD.open(altitudeLogPath, FILE_WRITE);
    if (altitudeLogFile) {
        altitudeLogFile.println("receiver_time,current_altitude,max_altitude,temperature,max_g,launch_state");
        altitudeLogFile.flush();
    }
    
    systemLogFile = SD.open(systemLogPath, FILE_WRITE);
    if (systemLogFile) {
        systemLogFile.println("receiver_time,uptime,battery_mv,battery_percent,tx_power,boot_time");
        systemLogFile.flush();
    }
    
    Serial.println("Created log files for transmitter: " + transmitterName);
    Serial.println("GPS log: " + gpsLogPath);
    Serial.println("Altitude log: " + altitudeLogPath);
    Serial.println("System log: " + systemLogPath);
    
    // Reset altitude tracking for logging
    lastLoggedAltitude = 0.0f;
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    Serial.println("\n\nRocket Telemetry Receiver Starting...");
    
    // Initialize the display manager first (most important component)
    Serial.println("Initializing display...");
    display.begin();
    Serial.println("Display initialized");
    
    // Set a default battery percentage since we might not have a battery monitor
    display.setReceiverBatteryPercent(100);
    
    // Battery monitor is disabled to prevent crashes
    #define ENABLE_BATTERY_MONITOR 0
    
    // Set a fixed battery percentage since we can't use the battery monitor
    batteryMonitorEnabled = false;
    display.setReceiverBatteryPercent(100);
    Serial.println("Battery monitor disabled to prevent crashes");
    
    // Initialize SD card if ENABLE_SD_CARD is defined
    #define ENABLE_SD_CARD 0
    #if ENABLE_SD_CARD
        Serial.print("Initializing SD card... ");
        if (SD.begin(SD_CS)) {
            Serial.println("SD card initialized successfully");
            sdCardAvailable = true;
        } else {
            Serial.println("SD card initialization failed");
            sdCardAvailable = false;
        }
    #else
        Serial.println("SD card disabled");
        sdCardAvailable = false;
    #endif
    
    // Initialize radio if enabled
    #ifndef DISABLE_RADIO
        Serial.println(F("[Radio] Starting SX1278 radio initialization"));
        Serial.println(F("Make sure SX1278 is connected to the same SPI bus as the display:"));
        Serial.println(F("- SCK: Pin 12"));
        Serial.println(F("- MISO: Pin 13"));
        Serial.println(F("- MOSI: Pin 11"));
        Serial.print(F("Module-specific pins - CS: ")); Serial.print(RADIO_CS);
        Serial.print(F(", RST: ")); Serial.print(RADIO_RST);
        Serial.print(F(", DIO0: ")); Serial.println(RADIO_DIO0);
        
        // Create and initialize the radio
        int initAttempts = 0;
        const int MAX_INIT_ATTEMPTS = 3;
        bool radioInitialized = false;
        
        while (!radioInitialized && initAttempts < MAX_INIT_ATTEMPTS) {
            initAttempts++;
            
            // Try to initialize the SX1278 radio
            radio = new SX1278Radio(RADIO_CS, RADIO_RST, RADIO_DIO0);
            
            Serial.print(F("[Radio] Initializing SX1278 ... Attempt ")); Serial.print(initAttempts); Serial.print(F(": "));
            
            // Try to initialize with more debug info
            Serial.println(F("Attempting to initialize SX1278 radio..."));
            Serial.println(F("Checking SPI connection..."));
            
            // Test SPI connection by checking if CS pin works
            pinMode(RADIO_CS, OUTPUT);
            digitalWrite(RADIO_CS, HIGH);
            delay(10);
            digitalWrite(RADIO_CS, LOW);
            delay(10);
            digitalWrite(RADIO_CS, HIGH);
            Serial.println(F("CS pin toggled"));
            
            // Check reset pin
            Serial.println(F("Testing reset pin..."));
            pinMode(RADIO_RST, OUTPUT);
            digitalWrite(RADIO_RST, LOW);
            delay(10);
            digitalWrite(RADIO_RST, HIGH);
            delay(10);
            Serial.println(F("Reset pin toggled"));
            
            // Now try to initialize
            bool initResult = radio->begin();
            
            if (initResult) {
                radioInitialized = true;
                Serial.println(F("success!"));
                Serial.println(F("SX1278 radio initialized successfully"));
                
                // Use the SX1278Radio's configure method
                Serial.println(F("Configuring SX1278 radio"));
                
                // Set up configuration parameters
                float freq = 433.0;  // Default frequency in MHz
                float bw = 125.0;    // Bandwidth in kHz
                int8_t power = 10;   // Output power in dBm
                
                Serial.print(F("Configuring with frequency: ")); Serial.print(freq);
                Serial.print(F(" MHz, bandwidth: ")); Serial.print(bw);
                Serial.print(F(" kHz, power: ")); Serial.print(power); Serial.println(F(" dBm"));
                
                // Try to configure the radio
                bool configSuccess = radio->configure(freq, bw, power);
                
                if (configSuccess) {
                    Serial.println(F("Radio configured successfully!"));
                    Serial.print(F("Listening on ")); Serial.print(freq); Serial.println(F(" MHz"));
                } else {
                    Serial.println(F("Radio configuration failed with default parameters"));
                    
                    // Try with more conservative settings
                    bw = 62.5;  // More conservative bandwidth
                    power = 5;   // Lower power
                    
                    Serial.print(F("Trying more conservative settings - bandwidth: ")); Serial.print(bw);
                    Serial.print(F(" kHz, power: ")); Serial.print(power); Serial.println(F(" dBm"));
                    
                    configSuccess = radio->configure(freq, bw, power);
                    
                    if (configSuccess) {
                        Serial.println(F("Radio configured successfully with conservative settings!"));
                        Serial.print(F("Listening on ")); Serial.print(freq); Serial.println(F(" MHz"));
                    } else {
                        Serial.println(F("Radio configuration still failed"));
                        Serial.println(F("Will try to continue with default radio settings"));
                        
                        // Try setting just the output power as a last resort
                        if (radio->setOutputPower(power)) {
                            Serial.println(F("Output power set successfully"));
                        } else {
                            Serial.println(F("Output power setting failed"));
                        }
                    }
                }
                
                // Initialize direct access to SX1278 for testing
                Serial.println(F("Initializing direct SX1278 access for testing"));
                testModule = new Module(RADIO_CS, RADIO_DIO0, RADIO_RST, RADIOLIB_NC);
                testLora = new SX1278(testModule);
                
                int state = testLora->begin();
                if (state == RADIOLIB_ERR_NONE) {
                    Serial.println(F("Direct SX1278 initialization successful!"));
                    
                    // Configure LoRa for basic operation
                    testLora->setCodingRate(5);           // 4/5 coding rate
                    testLora->setSpreadingFactor(7);      // SF7 - fastest data rate
                    testLora->setBandwidth(62.5);         // 62.5 kHz bandwidth - more reliable
                    testLora->setPreambleLength(8);       // Standard preamble length
                    testLora->setSyncWord(0x12);          // Common LoRa sync word
                    testLora->setCRC(true);               // Enable CRC for reliability
                    testLora->setOutputPower(5);          // 5 dBm output power
                    
                    Serial.println(F("Direct SX1278 configured for testing"));
                } else {
                    Serial.print(F("Direct SX1278 initialization failed with error code: ")); Serial.println(state);
                    delete testLora;
                    delete testModule;
                    testLora = nullptr;
                    testModule = nullptr;
                }
            } else {
                Serial.println(F("failed!"));
                Serial.print(F("SX1278 initialization attempt ")); Serial.print(initAttempts); Serial.println(F(" failed"));
                
                // Clean up and try again
                delete radio;
                radio = nullptr;
                delay(500); // Wait a bit before trying again
            }
        }
        
        if (!radioInitialized) {
            Serial.println(F("Failed to initialize SX1278 radio after multiple attempts"));
            radio = nullptr;
        }
    #else
        Serial.println(F("Radio disabled in build configuration"));
    #endif
    
    // Initialize touch test mode if enabled
    if (touchTestMode) {
        Serial.println(F("Starting touch test mode"));
        testTouchScreen();
    }
    
    Serial.println(F("Setup complete!"));
    
    // Start simulation mode if no real transmitters are detected
    if (display.getNumTransmitters() == 0) {
        Serial.println("No transmitters detected, starting simulation mode");
        startSimulationMode();
    }
}

// Simulation variables
bool simulationMode = false;
unsigned long lastSimulationUpdate = 0;
const unsigned long SIMULATION_UPDATE_INTERVAL = 500; // Update every 500ms
float simulatedAltitude = 0.0f;
float simulatedSpeed = 0.0f;
float simulatedAcceleration = 0.0f;
float simulatedBatteryVoltage = 3.7f;
uint8_t simulatedBatteryPercent = 100;
int8_t simulatedTxPower = 10;
uint32_t simulatedUptime = 0;
float simulatedTemperature = 25.0f;
float simulatedMaxAltitude = 0.0f;
float simulatedMaxG = 0.0f;
float simulatedOrientX = 0.0f;
float simulatedOrientY = 0.0f;
float simulatedOrientZ = 1.0f;

// Start simulation mode
void startSimulationMode() {
    simulationMode = true;
    
    // Create a simulated transmitter
    uint32_t simulatedTransmitterId = 0x12345678;
    display.addTransmitter(simulatedTransmitterId);
    
    // Set the transmitter frequency
    display.setTransmitterFrequency(simulatedTransmitterId, 433.0f, 1); // SX1278 radio type
    display.setFrequencyAcknowledged(simulatedTransmitterId, true);
    
    // Select the transmitter using the proper setter method
    display.setSelectedTransmitterId(simulatedTransmitterId);
    display.setRocketSelected(true);
    
    Serial.println("Simulation mode started");
    Serial.println("Created simulated transmitter with ID: 0x12345678");
    Serial.println("Simulating rocket launch and flight...");
}

// Update simulation data
void updateSimulation() {
    if (!simulationMode) return;
    
    unsigned long currentTime = millis();
    if (currentTime - lastSimulationUpdate < SIMULATION_UPDATE_INTERVAL) return;
    
    lastSimulationUpdate = currentTime;
    simulatedUptime += SIMULATION_UPDATE_INTERVAL;
    
    // Simulate flight phases
    static int flightPhase = 0; // 0=pre-launch, 1=boost, 2=coast, 3=descent, 4=landed
    static unsigned long phaseStartTime = 0;
    static float maxAlt = 0.0f;
    
    if (flightPhase == 0 && simulatedUptime > 5000) {
        // Start boost phase after 5 seconds
        flightPhase = 1;
        phaseStartTime = simulatedUptime;
        Serial.println("Simulation: Boost phase started");
    }
    else if (flightPhase == 1 && (simulatedUptime - phaseStartTime > 3000)) {
        // Boost for 3 seconds
        flightPhase = 2;
        phaseStartTime = simulatedUptime;
        Serial.println("Simulation: Coast phase started");
    }
    else if (flightPhase == 2 && simulatedAltitude > maxAlt + 5.0f) {
        // Keep track of max altitude during coast
        maxAlt = simulatedAltitude;
    }
    else if (flightPhase == 2 && simulatedAltitude < maxAlt - 10.0f) {
        // Start descent when we're 10m below max altitude
        flightPhase = 3;
        phaseStartTime = simulatedUptime;
        Serial.println("Simulation: Descent phase started");
    }
    else if (flightPhase == 3 && simulatedAltitude < 1.0f) {
        // Landed
        flightPhase = 4;
        phaseStartTime = simulatedUptime;
        Serial.println("Simulation: Landed");
    }
    
    // Update simulated values based on flight phase
    switch (flightPhase) {
        case 0: // Pre-launch
            simulatedAltitude = 0.0f;
            simulatedSpeed = 0.0f;
            simulatedAcceleration = 0.0f;
            simulatedOrientZ = 1.0f;
            break;
            
        case 1: // Boost
            simulatedAcceleration = 30.0f + random(-5, 5) / 10.0f; // ~3G with small variations
            simulatedSpeed += simulatedAcceleration * (SIMULATION_UPDATE_INTERVAL / 1000.0f);
            simulatedAltitude += simulatedSpeed * (SIMULATION_UPDATE_INTERVAL / 1000.0f);
            simulatedTemperature += 0.1f; // Slowly increase temperature
            simulatedBatteryVoltage -= 0.001f; // Slowly drain battery
            simulatedOrientZ = 1.0f + random(-10, 10) / 100.0f; // Slight wobble
            break;
            
        case 2: // Coast
            simulatedAcceleration = -9.8f + random(-2, 2) / 10.0f; // Gravity with small variations
            simulatedSpeed += simulatedAcceleration * (SIMULATION_UPDATE_INTERVAL / 1000.0f);
            simulatedAltitude += simulatedSpeed * (SIMULATION_UPDATE_INTERVAL / 1000.0f);
            simulatedTemperature -= 0.05f; // Slowly decrease temperature
            simulatedBatteryVoltage -= 0.0005f; // Drain battery slower
            simulatedOrientZ = 1.0f + random(-20, 20) / 100.0f; // More wobble
            break;
            
        case 3: // Descent
            simulatedAcceleration = -9.8f + random(-2, 2) / 10.0f; // Gravity with small variations
            // Terminal velocity for parachute
            if (simulatedSpeed < -15.0f) simulatedSpeed = -15.0f;
            else simulatedSpeed += simulatedAcceleration * (SIMULATION_UPDATE_INTERVAL / 1000.0f);
            simulatedAltitude += simulatedSpeed * (SIMULATION_UPDATE_INTERVAL / 1000.0f);
            if (simulatedAltitude < 0.0f) simulatedAltitude = 0.0f;
            simulatedTemperature += 0.02f; // Slowly increase temperature as we descend
            simulatedBatteryVoltage -= 0.0005f; // Drain battery slower
            simulatedOrientZ = random(-100, 100) / 100.0f; // Random orientation during descent
            simulatedOrientX = random(-100, 100) / 100.0f;
            simulatedOrientY = random(-100, 100) / 100.0f;
            break;
            
        case 4: // Landed
            simulatedAltitude = 0.0f;
            simulatedSpeed = 0.0f;
            simulatedAcceleration = 0.0f;
            simulatedOrientZ = 0.0f;
            simulatedOrientX = 1.0f;
            simulatedBatteryVoltage -= 0.0001f; // Drain battery very slowly
            break;
    }
    
    // Update max values
    if (simulatedAltitude > simulatedMaxAltitude) simulatedMaxAltitude = simulatedAltitude;
    float gForce = abs(simulatedAcceleration / 9.8f);
    if (gForce > simulatedMaxG) simulatedMaxG = gForce;
    
    // Calculate battery percentage
    simulatedBatteryPercent = map(constrain((int)(simulatedBatteryVoltage * 100), 300, 420), 300, 420, 0, 100);
    
    // Update display with simulated data
    display.updateAltitudeData(
        simulatedAltitude,
        simulatedMaxAltitude,
        simulatedTemperature,
        simulatedMaxG,
        simulatedSpeed,
        simulatedSpeed, // Use same value for both velocity sources
        simulatedOrientX,
        simulatedOrientY,
        simulatedOrientZ
    );
    
    display.updateSystemData(
        simulatedBatteryVoltage,
        simulatedBatteryPercent,
        simulatedTxPower,
        simulatedUptime,
        0 // Tilt angle
    );
    
    // Print simulation status every 2 seconds
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint > 2000) {
        lastPrint = currentTime;
        Serial.print("Simulation: Alt="); Serial.print(simulatedAltitude);
        Serial.print("m, Speed="); Serial.print(simulatedSpeed);
        Serial.print("m/s, Accel="); Serial.print(simulatedAcceleration);
        Serial.print("m/s², Batt="); Serial.print(simulatedBatteryVoltage);
        Serial.print("V ("); Serial.print(simulatedBatteryPercent);
        Serial.println("%)");
    }
}

// Original radio initialization success message and configuration
// (moved outside setup since we're now conditionally initializing the radio)
void configureRadio() {
    if (radio == nullptr) return;
    
    Serial.println(F("Radio initialized successfully!"));
    
    // Configure radio for the announcement frequency
    if (!radio->configure(radio->getAnnouncementFrequency(), 250.0, 14)) {
        Serial.println(F("Radio configuration failed!"));
    } else {
        Serial.print(F("Listening for frequency announcements on "));
        Serial.print(radio->getAnnouncementFrequency());
        Serial.println(F(" MHz"));
    }
}

// Process GPS packet
void processGpsPacket(uint8_t* buffer, size_t length) {
    // Verify packet length
    if (length < sizeof(GpsDataPacket)) {
        Serial.println("GPS packet too short");
        display.errorCount++;
        return;
    }
    
    // Extract transmitter ID
    uint32_t transmitterId = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16) | (buffer[5] << 24);
    
    // Check if this is from our selected transmitter
    if (display.isRocketSelected() && transmitterId != display.getSelectedTransmitterId()) {
        return; // Ignore packets from other transmitters
    }
    
    // Add to known transmitters if new
    display.addTransmitter(transmitterId);
    
    // Parse GPS data
    GpsDataPacket* gpsPacket = (GpsDataPacket*)buffer;
    float lat = gpsPacket->latitude;
    float lng = gpsPacket->longitude;
    uint16_t alt = gpsPacket->altitude;
    
    // Update GPS data using the specialized method
    display.updateGpsData(lat, lng);
    
    // Store latest GPS packet for logging when altitude changes
    memcpy(&latestGpsPacket, gpsPacket, sizeof(GpsDataPacket));
    hasNewGpsData = true;
}

// Process altitude packet
void processAltitudePacket(uint8_t* buffer, size_t length) {
    if (length < sizeof(AltitudePacket)) {
        Serial.println("Altitude packet too small");
        display.errorCount++;
        return;
    }
    
    // Extract transmitter ID
    uint32_t transmitterId = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16) | (buffer[5] << 24);
    
    // Check if this is from our selected transmitter
    if (display.isRocketSelected() && transmitterId != display.getSelectedTransmitterId()) {
        return; // Ignore packets from other transmitters
    }
    
    // Add to known transmitters if new
    display.addTransmitter(transmitterId);
    
    // Parse altitude data
    AltitudePacket* packet = (AltitudePacket*)buffer;
    float altitude = packet->currentAltitude;
    float maxAlt = packet->maxAltitude;
    float temp = packet->temperature / 10.0f; // Convert back to degrees C
    float maxG = packet->maxG / 10.0f; // Convert back to g
    float accelVelocity = packet->accelVelocity / 100.0f; // Convert from cm/s to m/s
    float baroVelocity = packet->baroVelocity / 100.0f; // Convert from cm/s to m/s
    
    // Extract orientation data
    float orientX = packet->orientationX / 127.0f; // Convert from int8_t to float (-1.0 to 1.0)
    float orientY = packet->orientationY / 127.0f;
    float orientZ = packet->orientationZ / 127.0f;
    
    // Update altitude data using the specialized method
    // Now using transmitted velocity values and orientation data
    display.updateAltitudeData(altitude, maxAlt, temp, maxG, accelVelocity, baroVelocity, orientX, orientY, orientZ);
    
    // Log to SD card if available and altitude has changed by threshold amount
    if (sdCardAvailable && display.isRocketSelected() && transmitterId == display.getSelectedTransmitterId()) {
        // Check if altitude has changed enough to log
        if (abs(altitude - lastLoggedAltitude) >= ALTITUDE_LOG_THRESHOLD) {
            // Check if we need to create log files (first packet received)
            if ((!altitudeLogFile || !gpsLogFile || !systemLogFile) && rocketBootTime > 0) {
                createLogFiles(transmitterId, rocketBootTime);
            }
            
            unsigned long currentTime = millis();
            
            // Log altitude data
            if (altitudeLogFile) {
                altitudeLogFile.print(currentTime);
                altitudeLogFile.print(",");
                altitudeLogFile.print(altitude, 2);
                altitudeLogFile.print(",");
                altitudeLogFile.print(maxAlt, 2);
                altitudeLogFile.print(",");
                altitudeLogFile.print(temp, 2);
                altitudeLogFile.print(",");
                altitudeLogFile.print(maxG, 2);
                altitudeLogFile.print(",");
                altitudeLogFile.print(accelVelocity);
                altitudeLogFile.print(",");
                altitudeLogFile.println(baroVelocity);
                altitudeLogFile.flush();
            }
            
            // Log GPS data if we have it
            if (gpsLogFile && hasNewGpsData) {
                gpsLogFile.print(currentTime);
                gpsLogFile.print(",");
                gpsLogFile.print(latestGpsPacket.latitude, 6); // 6 decimal places for lat/lng
                gpsLogFile.print(",");
                gpsLogFile.print(latestGpsPacket.longitude, 6);
                gpsLogFile.print(",");
                gpsLogFile.println(latestGpsPacket.altitude);
                gpsLogFile.flush();
                hasNewGpsData = false;
            }
            
            // Log system data if we have it
            if (systemLogFile && hasNewSystemData) {
                systemLogFile.print(currentTime);
                systemLogFile.print(",");
                systemLogFile.print(latestSystemPacket.uptime);
                systemLogFile.print(",");
                systemLogFile.print(latestSystemPacket.batteryMillivolts);
                systemLogFile.print(",");
                systemLogFile.print(latestSystemPacket.batteryPercent);
                systemLogFile.print(",");
                systemLogFile.print(latestSystemPacket.txPower);
                systemLogFile.print(",");
                systemLogFile.print(latestSystemPacket.bootTime);
                systemLogFile.print(",");
                systemLogFile.println(latestSystemPacket.launchState);
                systemLogFile.flush();
                hasNewSystemData = false;
            }
            
            // Update last logged altitude
            lastLoggedAltitude = altitude;
        }
    }
}

// Process system data packet
void processSystemPacket(uint8_t* buffer, size_t length) {
    if (length < sizeof(SystemDataPacket)) {
        Serial.println("System packet too small");
        display.errorCount++;
        return;
    }
    
    // Extract transmitter ID
    uint32_t transmitterId = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16) | (buffer[5] << 24);
    
    // Check if this is from our selected transmitter
    if (display.isRocketSelected() && transmitterId != display.getSelectedTransmitterId()) {
        return; // Ignore packets from other transmitters
    }
    
    // Add to known transmitters if new
    display.addTransmitter(transmitterId);
    
    // Parse system data
    SystemDataPacket* packet = (SystemDataPacket*)buffer;
    uint32_t uptime = packet->uptime;
    float battV = packet->batteryMillivolts / 1000.0f;
    uint8_t battPct = packet->batteryPercent;
    int8_t txPwr = packet->txPower;
    uint32_t bootTime = packet->bootTime;
    uint8_t launchState = packet->launchState; // Get launch state from system packet
    uint8_t tiltAngle = packet->tiltAngle; // Get tilt angle from system packet
    
    // Store rocket boot time for log file naming
    if (rocketBootTime == 0 && bootTime > 0) {
        rocketBootTime = bootTime;
        Serial.print("Rocket boot time set to: ");
        Serial.println(rocketBootTime);
        
        // If this is the first system packet with valid boot time, create log files
        if (sdCardAvailable && display.isRocketSelected() && transmitterId == display.getSelectedTransmitterId()) {
            createLogFiles(transmitterId, rocketBootTime);
        }
    }
    
    // Update system data using the specialized method
    display.updateSystemData(battV, battPct, txPwr, uptime, tiltAngle);
    
    // Update launch state
    display.launchState = launchState;
    
    // Store latest system packet for logging when altitude changes
    memcpy(&latestSystemPacket, packet, sizeof(SystemDataPacket));
    hasNewSystemData = true;
}

// Send SNR feedback to the logger
void sendSnrFeedback() {
    if (!display.isRocketSelected()) return; // Don't send feedback if no rocket selected
    
    // Create SNR feedback packet using the proper struct
    SnrFeedbackPacket packet;
    packet.version = PROTOCOL_VERSION;
    packet.packetType = PACKET_TYPE_FEEDBACK;
    packet.subType = FEEDBACK_SUBTYPE_SNR;
    packet.transmitterId = display.getSelectedTransmitterId();
    packet.snrValue = display.lastSnr;
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    // Send packet
    radio->transmit((uint8_t*)&packet, sizeof(packet));
    
    Serial.print("Sent SNR feedback to transmitter 0x");
    Serial.print(packet.transmitterId, HEX);
    Serial.print(": ");
    Serial.println(packet.snrValue);
}

// Send buzzer command to the rocket
void sendBuzzerCommand(uint32_t transmitterId, bool activate) {
    if (transmitterId == 0) return; // Don't send if no valid transmitter ID
    
    // Create command packet
    CommandPacket packet;
    packet.version = PROTOCOL_VERSION;
    packet.packetType = PACKET_TYPE_COMMAND;
    packet.subType = COMMAND_SUBTYPE_BUZZER;
    packet.transmitterId = transmitterId;
    packet.commandParam = activate ? 1 : 0; // 1 = activate, 0 = deactivate
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    // Send packet
    radio->transmit((uint8_t*)&packet, sizeof(packet));
    
    Serial.print("Sent buzzer command to transmitter 0x");
    Serial.print(packet.transmitterId, HEX);
    Serial.print(": ");
    Serial.println(activate ? "ON" : "OFF");
    
    // Update display state to show buzzer status
    display.setBuzzerActive(activate);
}

// Send emergency abort command to the rocket
// This will trigger immediate parachute deployment
void sendAbortCommand(uint32_t transmitterId) {
    if (transmitterId == 0) return; // Don't send if no valid transmitter ID
    
    // Create command packet
    CommandPacket packet;
    packet.version = PROTOCOL_VERSION;
    packet.packetType = PACKET_TYPE_COMMAND;
    packet.subType = COMMAND_SUBTYPE_ABORT;
    packet.transmitterId = transmitterId;
    packet.commandParam = 1; // 1 = activate abort
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    // Send packet multiple times to ensure delivery
    for (int i = 0; i < 3; i++) {
        radio->transmit((uint8_t*)&packet, sizeof(packet));
        delay(100); // Short delay between transmissions
    }
    
    Serial.print("EMERGENCY ABORT COMMAND sent to transmitter 0x");
    Serial.println(packet.transmitterId, HEX);
    
    // Update display to show abort status
    display.setAbortSent(true);
}

// Process frequency announcement packet from a rocket
void processFrequencyAnnouncePacket(uint8_t* buffer, size_t length) {
    // Verify packet length
    if (length < sizeof(FrequencyAnnouncePacket)) {
        Serial.println("Frequency announcement packet too short");
        return;
    }
    
    // Extract packet data
    FrequencyAnnouncePacket* packet = (FrequencyAnnouncePacket*)buffer;
    
    // Convert frequency from kHz to MHz
    float frequency = packet->frequency / 1000.0f;
    uint32_t transmitterId = packet->transmitterId;
    uint8_t radioType = packet->radioType;
    
    Serial.print("Received frequency announcement from transmitter 0x");
    Serial.print(transmitterId, HEX);
    Serial.print(": ");
    Serial.print(frequency);
    Serial.println(" MHz");
    
    // Add transmitter to known transmitters if not already present
    display.addTransmitter(transmitterId);
    
    // Store the frequency and radio type for this transmitter
    display.setTransmitterFrequency(transmitterId, frequency, radioType);
    
    // Send acknowledgment
    sendFrequencyAcknowledgment(transmitterId, frequency);
    
    // If this is the selected transmitter, switch to its frequency
    if (display.isRocketSelected() && display.getSelectedTransmitterId() == transmitterId) {
        switchToTransmitterFrequency(transmitterId);
    }
}

// Send frequency acknowledgment to a rocket
void sendFrequencyAcknowledgment(uint32_t transmitterId, float frequency) {
    // Create frequency ack packet
    FrequencyAckPacket packet;
    packet.version = PROTOCOL_VERSION;
    packet.packetType = PACKET_TYPE_FREQ_ACK;
    packet.transmitterId = transmitterId;
    packet.frequency = (uint32_t)(frequency * 1000.0f); // Convert MHz to kHz
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    // Send the packet
    if (radio->transmit((uint8_t*)&packet, sizeof(packet))) {
        Serial.print("Sent frequency acknowledgment to transmitter 0x");
        Serial.print(transmitterId, HEX);
        Serial.print(": ");
        Serial.print(frequency);
        Serial.println(" MHz");
        
        // Mark this frequency as acknowledged
        display.setFrequencyAcknowledged(transmitterId, true);
    } else {
        Serial.println("Failed to send frequency acknowledgment");
    }
}

// Switch to the operating frequency of a transmitter
void switchToTransmitterFrequency(uint32_t transmitterId) {
    // Get the frequency for this transmitter
    float frequency = display.getTransmitterFrequency(transmitterId);
    
    // Only switch if we have a valid frequency
    if (frequency > 0.0f) {
        Serial.print("Switching to frequency ");
        Serial.print(frequency);
        Serial.print(" MHz for transmitter 0x");
        Serial.println(transmitterId, HEX);
        
        // Configure radio for this frequency
        radio->configure(frequency, 250.0, 14);
        
        // No longer need to listen for announcements
        listeningForAnnouncements = false;
    } else {
        Serial.print("No frequency information available for transmitter 0x");
        Serial.println(transmitterId, HEX);
        
        // Switch back to announcement frequency to listen for announcements
        radio->configure(radio->getAnnouncementFrequency(), 250.0, 14);
        listeningForAnnouncements = true;
    }
}

// Display low battery shutdown message and shut down
void lowBatteryShutdown() {
    // Display low battery warning using the RocketMonitorScreen class
    display.drawLowBattery();
    
    // Wait a bit to show the message
    delay(5000);
    display.sleepDisplay();
    
    // Put device to sleep
    esp_deep_sleep_start();
}

// Variables for radio testing
unsigned long lastTestPacketTime = 0;
const unsigned long TEST_PACKET_INTERVAL = 5000; // Send a test packet every 5 seconds

// Function to test touchscreen functionality
void testTouchScreen() {
    // Clear screen
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw touch test interface
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Touch Test Mode");
    
    // Print SPI configuration
    tft.setTextSize(1);
    tft.setCursor(10, 40);
    tft.println("SPI Configuration:");
    
    tft.setCursor(10, 50);
    tft.print("Display CS: ");
    tft.println(TFT_CS);
    tft.setCursor(10, 60);
    tft.print("Touch CS: ");
    tft.println(TOUCH_CS);
    tft.setCursor(10, 70);
    tft.print("Touch IRQ: ");
    tft.println(TOUCH_IRQ);
    tft.setCursor(10, 80);
    tft.print("SPI MOSI: ");
    tft.println(TFT_MOSI);
    tft.setCursor(10, 90);
    tft.print("SPI MISO: ");
    tft.println(TFT_MISO);
    tft.setCursor(10, 100);
    tft.print("SPI CLK: ");
    tft.println(TFT_CLK);
    
    // Try to reinitialize touchscreen
    Serial.println("Reinitializing touchscreen...");
    ts.begin();
    
    // Try to get initial touch reading
    delay(100);
    bool touched = ts.touched();
    TS_Point p = ts.getPoint();
    
    tft.setCursor(10, 120);
    tft.print("Touch state: ");
    tft.println(touched ? "TOUCHED" : "NOT TOUCHED");
    
    tft.setCursor(10, 130);
    tft.print("Raw values: x=");
    tft.print(p.x);
    tft.print(", y=");
    tft.print(p.y);
    tft.print(", z=");
    tft.println(p.z);
    
    // Try with different rotation settings
    Serial.println("Testing different touchscreen rotations...");
    
    for (int i = 0; i < 4; i++) {
        ts.setRotation(i);
        delay(50);
        TS_Point p = ts.getPoint();
        
        tft.setCursor(10, 140 + i*10);
        tft.print("Rotation ");
        tft.print(i);
        tft.print(": x=");
        tft.print(p.x);
        tft.print(", y=");
        tft.print(p.y);
        tft.print(", z=");
        tft.println(p.z);
        
        Serial.print("Rotation ");
        Serial.print(i);
        Serial.print(": x=");
        Serial.print(p.x);
        Serial.print(", y=");
        Serial.print(p.y);
        Serial.print(", z=");
        Serial.println(p.z);
    }
    
    // Reset rotation to default
    ts.setRotation(0);
    
    // Draw a target grid
    tft.drawLine(0, tft.height()/2, tft.width(), tft.height()/2, ILI9341_DARKGREY);
    tft.drawLine(tft.width()/2, 0, tft.width()/2, tft.height(), ILI9341_DARKGREY);
    
    // Draw touch points at the corners and center
    tft.fillCircle(20, 20, 5, ILI9341_RED);
    tft.fillCircle(tft.width()-20, 20, 5, ILI9341_RED);
    tft.fillCircle(20, tft.height()-20, 5, ILI9341_RED);
    tft.fillCircle(tft.width()-20, tft.height()-20, 5, ILI9341_RED);
    tft.fillCircle(tft.width()/2, tft.height()/2, 5, ILI9341_RED);
    
    // Instructions to exit
    tft.setCursor(10, tft.height() - 20);
    tft.println("Touch center 3 times quickly to exit");
}

void testRadio() {
    // First try with the regular Radio interface
    if (radio) {
        Serial.println(F("Testing with Radio interface..."));
        
        // Create a simple test packet
        uint8_t testPacket[10] = {0};
        testPacket[0] = 0x01; // Packet type (made up for testing)
        testPacket[1] = 0x08; // Packet length
        testPacket[2] = 0x78; // ID bytes (0x12345678)
        testPacket[3] = 0x56;
        testPacket[4] = 0x34;
        testPacket[5] = 0x12;
        testPacket[6] = 0xAA; // Test data
        testPacket[7] = 0xBB;
        testPacket[8] = 0xCC;
        testPacket[9] = 0xDD;
        
        // Try to send the packet
        Serial.println(F("Sending test packet via Radio interface..."));
        int state = radio->transmit(testPacket, 10);
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println(F("Test packet sent successfully via Radio interface!"));
        } else {
            Serial.print(F("Radio interface transmission failed with error code: ")); Serial.println(state);
        }
    } else {
        Serial.println(F("Radio interface not initialized, skipping this test"));
    }
    
    // Now try with direct SX1278 access
    if (testLora) {
        Serial.println(F("\nTesting with direct SX1278 access..."));
        
        // Create a simple test packet
        uint8_t testPacket[10] = {0};
        testPacket[0] = 0x01; // Packet type (made up for testing)
        testPacket[1] = 0x08; // Packet length
        testPacket[2] = 0x78; // ID bytes (0x12345678)
        testPacket[3] = 0x56;
        testPacket[4] = 0x34;
        testPacket[5] = 0x12;
        testPacket[6] = 0xAA; // Test data
        testPacket[7] = 0xBB;
        testPacket[8] = 0xCC;
        testPacket[9] = 0xDD;
        
        // Try to set frequency explicitly
        Serial.println(F("Setting frequency to 433.0 MHz..."));
        int freqState = testLora->setFrequency(433.0);
        
        if (freqState == RADIOLIB_ERR_NONE) {
            Serial.println(F("Frequency set successfully"));
        } else {
            Serial.print(F("Failed to set frequency, error: ")); Serial.println(freqState);
        }
        
        // Try to send the packet
        Serial.println(F("Sending test packet via direct SX1278 access..."));
        int state = testLora->transmit(testPacket, 10);
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println(F("Test packet sent successfully via direct SX1278 access!"));
            Serial.print(F("Packet data: "));
            for (int i = 0; i < 10; i++) {
                Serial.print(testPacket[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        } else {
            Serial.print(F("Direct SX1278 transmission failed with error code: ")); Serial.println(state);
            
            // Try to diagnose the issue
            if (state == RADIOLIB_ERR_TX_TIMEOUT) {
                Serial.println(F("Error: Transmission timed out"));
            } else if (state == RADIOLIB_ERR_SPI_WRITE_FAILED) {
                Serial.println(F("Error: SPI write failed - check connections"));
            } else if (state == RADIOLIB_ERR_INVALID_FREQUENCY) {
                Serial.println(F("Error: Invalid frequency"));
            }
        }
        
        // Try to receive any packets
        Serial.println(F("Setting radio to receive mode..."));
        state = testLora->startReceive();
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println(F("Radio set to receive mode successfully"));
            
            // Wait a moment in receive mode
            delay(500);
            
            // Check if any packets were received
            if (testLora->available()) {
                Serial.println(F("Packet received!"));
                
                // Read the received packet
                uint8_t rxData[64];
                size_t rxLen = sizeof(rxData);
                int rxState = testLora->readData(rxData, rxLen);
                
                if (rxState == RADIOLIB_ERR_NONE) {
                    Serial.print(F("Received packet (")); Serial.print(rxLen); Serial.println(F(" bytes):"));
                    for (size_t i = 0; i < rxLen; i++) {
                        Serial.print(rxData[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
                } else {
                    Serial.print(F("Failed to read received packet, error: ")); Serial.println(rxState);
                }
            } else {
                Serial.println(F("No packets received during listening period"));
            }
        } else {
            Serial.print(F("Failed to set radio to receive mode, error: ")); Serial.println(state);
        }
    } else {
        Serial.println(F("Direct SX1278 access not initialized, skipping this test"));
    }
}

void loop() {
    unsigned long currentTime = millis();
    
    // Simple touchscreen test mode
    if (touchTestMode) {
        static bool initialized = false;
        static unsigned long lastInfoUpdate = 0;
        
        // Initialize the test screen once
        if (!initialized) {
            tft.fillScreen(ILI9341_BLACK);
            tft.setTextColor(ILI9341_WHITE);
            tft.setTextSize(2);
            tft.setCursor(10, 10);
            tft.println("Touch Test Mode");
            
            tft.setTextSize(1);
            tft.setCursor(10, 40);
            tft.println("SPI Configuration:");
            tft.setCursor(10, 50);
            tft.print("Display CS: "); tft.println(TFT_CS);
            tft.setCursor(10, 60);
            tft.print("Touch CS: "); tft.println(TOUCH_CS);
            tft.setCursor(10, 70);
            tft.print("Touch IRQ: "); tft.println(TOUCH_IRQ);
            
            // Draw a target grid
            tft.drawLine(0, tft.height()/2, tft.width(), tft.height()/2, ILI9341_DARKGREY);
            tft.drawLine(tft.width()/2, 0, tft.width()/2, tft.height(), ILI9341_DARKGREY);
            
            // Draw touch points at the corners and center
            tft.fillCircle(20, 20, 5, ILI9341_RED);
            tft.fillCircle(tft.width()-20, 20, 5, ILI9341_RED);
            tft.fillCircle(20, tft.height()-20, 5, ILI9341_RED);
            tft.fillCircle(tft.width()-20, tft.height()-20, 5, ILI9341_RED);
            tft.fillCircle(tft.width()/2, tft.height()/2, 5, ILI9341_RED);
            
            initialized = true;
        }
        
        // Check touch status every 100ms
        if (currentTime - lastTouchCheck > TOUCH_CHECK_INTERVAL) {
            lastTouchCheck = currentTime;
            
            // Check if IRQ pin is working (if connected)
            if (TOUCH_IRQ != 255) {
                int irqState = digitalRead(TOUCH_IRQ);
                static int lastIrqState = -1;
                
                if (irqState != lastIrqState) {
                    Serial.print("Touch IRQ pin state: ");
                    Serial.println(irqState == LOW ? "ACTIVE (LOW)" : "INACTIVE (HIGH)");
                    lastIrqState = irqState;
                }
            }
            
            // Try to read touchscreen
            bool touched = ts.touched();
            TS_Point p = ts.getPoint();
            
            // Update display with touch info every second
            if (currentTime - lastInfoUpdate > 1000) {
                lastInfoUpdate = currentTime;
                
                // Clear info area
                tft.fillRect(10, 90, 220, 60, ILI9341_BLACK);
                
                // Display touch status
                tft.setCursor(10, 90);
                tft.print("Touch state: ");
                tft.println(touched ? "TOUCHED" : "NOT TOUCHED");
                
                // Display raw values
                tft.setCursor(10, 100);
                tft.print("Raw: x=");
                tft.print(p.x);
                tft.print(", y=");
                tft.print(p.y);
                tft.print(", z=");
                tft.println(p.z);
                
                // Print to serial too
                Serial.print("Touch state: ");
                Serial.println(touched ? "TOUCHED" : "NOT TOUCHED");
                Serial.print("Raw values: x=");
                Serial.print(p.x);
                Serial.print(", y=");
                Serial.print(p.y);
                Serial.print(", z=");
                Serial.println(p.z);
            }
            
            // If touched, show the point
            if (touched) {
                // Map touch coordinates to screen
                int y = map(p.x, 240, 3800, 0, tft.width());
                int x = map(p.y, 240, 3800, 0, tft.height());
                
                // Draw touch point if in bounds
                if (x >= 0 && x < tft.height() && y >= 0 && y < tft.width()) {
                    tft.fillCircle(y, x, 3, ILI9341_GREEN);
                }
            }
        }
        
        // Skip the rest of the loop in touch test mode
        return;
    }
    
    // Normal operation mode
    // Test the radio every few seconds
    if (currentTime - lastTestPacketTime > TEST_PACKET_INTERVAL) {
        lastTestPacketTime = currentTime;
        testRadio();
    }
    
    // Update simulation if in simulation mode
    if (simulationMode) {
        updateSimulation();
    }
    
    // Track buzzer state for change detection and periodic resending
    static bool lastBuzzerState = false;
    static unsigned long lastBuzzerCommandTime = 0;
    bool currentBuzzerState = display.isBuzzerActive();
    
    // Track abort state for change detection and periodic resending
    static bool lastAbortState = false;
    static unsigned long lastAbortCommandTime = 0;
    bool currentAbortState = display.isAbortSent();
    
    // Check if buzzer state has changed (button was pressed)
    if (currentBuzzerState != lastBuzzerState && display.isRocketSelected()) {
        // Send buzzer command to the rocket
        sendBuzzerCommand(display.getSelectedTransmitterId(), currentBuzzerState);
        lastBuzzerState = currentBuzzerState;
        lastBuzzerCommandTime = millis();
    }
    
    // Check if abort state has changed (abort was confirmed)
    if (currentAbortState && !lastAbortState && display.isRocketSelected()) {
        // Send abort command to the rocket
        sendAbortCommand(display.getSelectedTransmitterId());
        lastAbortState = currentAbortState;
        lastAbortCommandTime = millis();
        
        // Log the abort event
        Serial.print("EMERGENCY ABORT triggered for rocket: ");
        Serial.println(display.getTransmitterName(display.getSelectedTransmitterId()));
    }
    
    // Continuously resend the abort command if active (every 100ms)
    // This ensures the command isn't lost due to packet loss
    // Critical for emergency situations
    if (currentAbortState && display.isRocketSelected() && 
        millis() - lastAbortCommandTime >= 100) { // 100ms - much more frequent than buzzer
        // Create and send abort command packet directly instead of calling sendAbortCommand
        // to avoid the multiple transmissions within that function (we're already sending frequently)
        if (radio != nullptr) {
            CommandPacket packet;
            packet.version = PROTOCOL_VERSION;
            packet.packetType = PACKET_TYPE_COMMAND;
            packet.subType = COMMAND_SUBTYPE_ABORT;
            packet.transmitterId = display.getSelectedTransmitterId();
            packet.commandParam = 1; // 1 = activate abort
            packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
            
            // Send the packet once (we'll send again in 100ms)
            radio->transmit((uint8_t*)&packet, sizeof(packet));
        }
        lastAbortCommandTime = millis();
    }
    
    // Periodically resend the buzzer command if active (every 5 seconds)
    // This ensures the command isn't lost due to packet loss
    if (currentBuzzerState && display.isRocketSelected() && 
        millis() - lastBuzzerCommandTime >= 5000) { // 5 seconds
        if (radio != nullptr) {
            sendBuzzerCommand(display.getSelectedTransmitterId(), true);
        }
        lastBuzzerCommandTime = millis();
        Serial.println("Resending buzzer ON command");
    }
    
    // Battery monitoring is disabled to prevent crashes
    static unsigned long lastBatteryCheck = 0;
    if (millis() - lastBatteryCheck >= 10000) {
        // Just update the timestamp
        lastBatteryCheck = millis();
    }
    
    // Handle touch events
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        // Print raw touch coordinates
        Serial.print("Raw touch: x=");
        Serial.print(p.x);
        Serial.print(", y=");
        Serial.print(p.y);
        Serial.print(", z=");
        Serial.println(p.z);
        
        // Convert touch coordinates to screen coordinates
        int y = map(p.x, 240, 3800, 0, tft.width());
        int x = map(p.y, 240, 3800, 0, tft.height());
        
        // Print mapped screen coordinates
        Serial.print("Screen coordinates: x=");
        Serial.print(x);
        Serial.print(", y=");
        Serial.println(y);
        
        // Also display on screen for visual feedback
        tft.fillRect(0, 0, 100, 20, ILI9341_BLACK);
        tft.setCursor(0, 0);
        tft.setTextColor(ILI9341_GREEN);
        tft.setTextSize(1);
        tft.print("X:");
        tft.print(x);
        tft.print(" Y:");
        tft.print(y);
        
        // Handle the touch event
        display.handleTouch(x, y);
    }
    
    uint8_t data[MAX_PACKET_SIZE];
    int packetSize = 0;
    
    if (radio != nullptr) {
        packetSize = radio->receive(data, MAX_PACKET_SIZE);
    }
    
    if (packetSize > 0) {
        // Check packet version
        if (data[0] != PROTOCOL_VERSION) {
            Serial.println("Invalid protocol version");
            display.errorCount++;
        } else if (packetSize > 2) { // Minimum packet size (version + type + checksum)
            uint8_t packetType = data[1];
            
            // Process packet based on type
            switch (packetType) {
                case PACKET_TYPE_GPS:
                    processGpsPacket(data, packetSize);
                    break;
                    
                case PACKET_TYPE_ALTITUDE:
                    processAltitudePacket(data, packetSize);
                    break;
                    
                case PACKET_TYPE_SYSTEM:
                    processSystemPacket(data, packetSize);
                    break;
                    
                case PACKET_TYPE_FREQ_ANNOUNCE:
                    processFrequencyAnnouncePacket(data, packetSize);
                    break;
                    
                case PACKET_TYPE_COMMAND:
                    // Process command packets (these are sent from receiver to transmitter)
                    // We don't need to do anything here as we're the sender
                    break;
                    
                case PACKET_TYPE_FEEDBACK:
                    // Process feedback packets (SNR feedback, etc.)
                    // We don't need special handling here as the radio handles SNR feedback
                    break;
                    
                default:
                    Serial.print("Unknown packet type: ");
                    Serial.println(packetType, HEX);
                    display.errorCount++;
                    break;
            }
            
            // Get SNR from radio
            float snr = 0.0f;
            if (radio != nullptr) {
                snr = radio->getSNR();
            }
            display.lastSnr = snr;
            display.packetCount++;
            display.lastPacketTime = millis();
            
            // Send SNR feedback periodically
            if (millis() - lastSnrFeedbackTime >= SNR_FEEDBACK_INTERVAL) {
                if (radio != nullptr) {
                    sendSnrFeedback();
                }
                lastSnrFeedbackTime = millis();
            }
        }
    }
    
    // Periodically resend frequency acknowledgments for unacknowledged frequencies
    // This ensures the rocket receives the acknowledgment and can switch to the selected frequency
    static unsigned long lastFrequencyAckCheck = 0;
    if (listeningForAnnouncements && millis() - lastFrequencyAckCheck >= FREQUENCY_ACK_INTERVAL) {
        // Check all transmitters for unacknowledged frequencies
        for (int i = 0; i < display.getNumTransmitters(); i++) {
            uint32_t transmitterId = display.getTransmitterId(i);
            float frequency = display.getTransmitterFrequency(transmitterId);
            
            // If frequency is valid but not acknowledged, resend acknowledgment
            if (frequency > 0.0f && !display.isFrequencyAcknowledged(transmitterId)) {
                sendFrequencyAcknowledgment(transmitterId, frequency);
            }
        }
        lastFrequencyAckCheck = millis();
    }
    
    // Update display every second regardless of new data
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 1000) {
        display.updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    // Check for manual rescan request from the UI
    if (display.isRescanRequested()) {
        Serial.println("Manual rescan requested");
        // Switch back to announcement frequency to listen for new rockets
        radio->configure(radio->getAnnouncementFrequency(), 250.0, 14);
        listeningForAnnouncements = true;
        display.clearRescanRequest();
    }
    
    // Check for auto-rescan due to connection timeout (no data for 10 seconds)
    if (display.isRocketSelected() && !listeningForAnnouncements && display.checkConnectionTimeout()) {
        Serial.println("Auto-rescan triggered: No data received for 10 seconds");
        // Switch back to announcement frequency to listen for new rockets
        radio->configure(radio->getAnnouncementFrequency(), 250.0, 14);
        listeningForAnnouncements = true;
        
        // If we're on the main data page, switch to transmitter selection page
        if (display.getCurrentPage() != RocketMonitorScreen::PAGE_TRANSMITTER_SELECTION) {
            display.setCurrentPage(RocketMonitorScreen::PAGE_TRANSMITTER_SELECTION);
            display.updateDisplay();
        }
    }
}
