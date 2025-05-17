#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Wire.h>
#include <MAX1704X.h>
#include <SD.h>
#include <time.h>
#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"
#include "rocket_monitor_screen.h"

// CC1101 configuration for ESP32
#define CC1101_CS 5    // GPIO5
#define CC1101_GDO0 4  // GPIO4
// Radio instance using CC1101
Radio* radio = new CC1101Radio(CC1101_CS, CC1101_GDO0, RADIOLIB_NC);

// Battery monitor configuration
#define MAX17043_SDA 32
#define MAX17043_SCL 22
#define LOW_BATTERY_THRESHOLD 45.0
MAX1704X batteryMonitor(0.00125f);

// Touch screen configuration
#define TOUCH_CS 21
XPT2046_Touchscreen ts(TOUCH_CS);
// Initialize TFT display
TFT_eSPI tft = TFT_eSPI();
// Create the display manager instance
RocketMonitorScreen display(tft, ts);

// Forward declarations of functions
void sendSnrFeedback();
void sendBuzzerCommand(uint32_t transmitterId, bool activate);
void sendAbortCommand(uint32_t transmitterId);
void processGpsPacket(uint8_t* buffer, size_t length);
void processAltitudePacket(uint8_t* buffer, size_t length);
void processSystemPacket(uint8_t* buffer, size_t length);
void lowBatteryShutdown();

// SNR feedback control
unsigned long lastSnrFeedbackTime = 0;
const unsigned long SNR_FEEDBACK_INTERVAL = 1000; // Send SNR feedback every 1 second

// SD card configuration
#define SD_CS 33 // SD card CS pin on TFT shield
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
    
    // Initialize I2C for battery monitor
    Wire.begin(MAX17043_SDA, MAX17043_SCL);
    
    // Initialize battery monitor
    batteryMonitor.reset();
    batteryMonitor.quickstart();
    if (!batteryMonitor.begin()) {
        Serial.println("Failed to initialize battery monitor!");
    } else {
        Serial.println("Battery monitor initialized successfully");
        display.setReceiverBatteryPercent(batteryMonitor.percent());
        Serial.print("Initial battery: ");
        Serial.print(display.getReceiverBatteryPercent());
        Serial.println("%");
    }
    
    // Initialize the display manager
    display.begin();
    
    // Initialize SD card
    Serial.print("Initializing SD card... ");
    if (SD.begin(SD_CS)) {
        Serial.println("SD card initialized successfully");
        sdCardAvailable = true;
    } else {
        Serial.println("SD card initialization failed");
        sdCardAvailable = false;
    }
    
    // Initialize radio
    Serial.print(F("[Radio] Initializing ... "));
    if (!radio->begin()) {
        Serial.println(F("failed!"));
        while (true);
    }
    Serial.println(F("success!"));
    
    // Configure radio parameters
    // Using 433.92 MHz frequency, 250 kHz bandwidth (adjust if needed), and 10 dBm power
    if (!radio->configure(433.92, 250.0, 10)) {
        Serial.println(F("Radio configuration failed!"));
        while (true);
    }
    Serial.println(F("Radio configured successfully!"));
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
    uint8_t launchState = packet->launchState;
    
    // Update altitude data using the specialized method
    // This will also handle speed calculation and graph updates
    display.updateAltitudeData(altitude, maxAlt, temp, maxG, launchState);
    
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
                altitudeLogFile.println(launchState);
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
                systemLogFile.println(latestSystemPacket.bootTime);
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
    display.updateSystemData(battV, battPct, txPwr, uptime);
    
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

void loop() {
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
        CommandPacket packet;
        packet.version = PROTOCOL_VERSION;
        packet.packetType = PACKET_TYPE_COMMAND;
        packet.subType = COMMAND_SUBTYPE_ABORT;
        packet.transmitterId = display.getSelectedTransmitterId();
        packet.commandParam = 1; // 1 = activate abort
        packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
        
        // Send the packet once (we'll send again in 100ms)
        radio->transmit((uint8_t*)&packet, sizeof(packet));
        
        lastAbortCommandTime = millis();
    }
    
    // Periodically resend the buzzer command if active (every 5 seconds)
    // This ensures the command isn't lost due to packet loss
    if (currentBuzzerState && display.isRocketSelected() && 
        millis() - lastBuzzerCommandTime >= 5000) { // 5 seconds
        sendBuzzerCommand(display.getSelectedTransmitterId(), true);
        lastBuzzerCommandTime = millis();
        Serial.println("Resending buzzer ON command");
    }
    
    // Read battery level
    static unsigned long lastBatteryCheck = 0;
    if (millis() - lastBatteryCheck >= 10000) { // Check every 10 seconds
        uint8_t batteryLevel = batteryMonitor.percent();
        display.setReceiverBatteryPercent(batteryLevel);
        display.drawBatteryIndicator(batteryLevel); // Update battery indicator
        
        // Check for low battery condition
        if (batteryLevel < LOW_BATTERY_THRESHOLD) {
            lowBatteryShutdown();
        }
        
        lastBatteryCheck = millis();
    }
    
    // Handle touch events
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        // Convert touch coordinates to screen coordinates
        int y = map(p.x, 240, 3800, 0, tft.width());
        int x = map(p.y, 240, 3800, 0, tft.height());
        display.handleTouch(x, y);
    }
    
    uint8_t data[MAX_PACKET_SIZE];
    int packetSize = radio->receive(data, MAX_PACKET_SIZE);
    
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
                    
                default:
                    Serial.print("Unknown packet type: ");
                    Serial.println(packetType, HEX);
                    display.errorCount++;
                    break;
            }
            
            // Get SNR from radio
            float snr = radio->getSNR();
            display.lastSnr = snr;
            display.packetCount++;
            display.lastPacketTime = millis();
            
            // Send SNR feedback periodically
            if (millis() - lastSnrFeedbackTime >= SNR_FEEDBACK_INTERVAL) {
                sendSnrFeedback();
                lastSnrFeedbackTime = millis();
            }
        }
    }
    
    // Update display every second regardless of new data
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 1000) {
        display.updateDisplay();
        lastDisplayUpdate = millis();
    }
}
