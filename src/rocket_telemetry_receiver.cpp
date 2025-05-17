#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Wire.h>
#include <MAX1704X.h>
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
void processGpsPacket(uint8_t* buffer, size_t length);
void processAltitudePacket(uint8_t* buffer, size_t length);
void processSystemPacket(uint8_t* buffer, size_t length);
void lowBatteryShutdown();

// Transmitter selection
#define MAX_TRANSMITTERS 10
#define MAX_NAME_LENGTH 16
uint32_t knownTransmitters[MAX_TRANSMITTERS] = {0};
String rocketNames[MAX_TRANSMITTERS];
int numTransmitters = 0;
int selectedTransmitterIndex = -1; // -1 means no transmitter selected
uint32_t selectedTransmitterId = 0;

// Preferences for storing rocket names
Preferences preferences;

// Keyboard variables
bool showKeyboard = false;
bool editingName = false;
int editingTransmitterIndex = -1;
String currentInput = "";
const char keyboardChars[4][10] = {
    {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'},
    {'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P'},
    {'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ' '},
    {'Z', 'X', 'C', 'V', 'B', 'N', 'M', '<', ' ', '>'}
};
#define KEY_WIDTH 30
#define KEY_HEIGHT 30
#define KEYBOARD_X 10
#define KEYBOARD_Y 100

// SNR feedback control
unsigned long lastSnrFeedbackTime = 0;
const unsigned long SNR_FEEDBACK_INTERVAL = 1000; // Send SNR feedback every 1 second

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
    
    // Update GPS data using the specialized method
    display.updateGpsData(lat, lng);
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
    
    // Update system data using the specialized method
    display.updateSystemData(battV, battPct, txPwr, uptime);
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
