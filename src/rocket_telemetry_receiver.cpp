#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"

// CC1101 configuration for ESP32
#define CC1101_CS 5    // GPIO5
#define CC1101_GDO0 4  // GPIO4

// Initialize TFT display
TFT_eSPI tft = TFT_eSPI();

// Radio instance using CC1101
Radio* radio = new CC1101Radio(CC1101_CS, CC1101_GDO0, RADIOLIB_NC);

// Display layout constants
#define HEADER_HEIGHT 30
#define VALUE_HEIGHT 26
#define RIGHT_COL 180
#define LABEL_COLOR TFT_YELLOW
#define VALUE_COLOR TFT_WHITE

// Global variables for tracking data
float currentAltitude = 0.0f;
float maxAltitude = 0.0f;
float latitude = 0.0f;
float longitude = 0.0f;
float gpsAltitude = 0.0f;
unsigned long lastPacketTime = 0;
unsigned long millisAtFirstPacket = 0;
float batteryVoltage = 0.0f;
float batteryPercent = 0.0f;
float maxG = 0.0f;
bool launchState = false;
float temperature = 0.0f;

// Packet statistics
uint32_t totalPackets = 0;
uint32_t badPackets = 0;
uint32_t badChecksums = 0;

void setup() {
    Serial.begin(115200);
    
    // Initialize display
    tft.init();
    tft.setRotation(3); // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    
    // Draw labels
    tft.setTextColor(LABEL_COLOR, TFT_BLACK);
    // Left column
    tft.drawString("Alt:", 20, HEADER_HEIGHT);
    tft.drawString("Max Alt:", 20, HEADER_HEIGHT + VALUE_HEIGHT);
    tft.drawString("GPS:", 20, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    tft.drawString("Staleness:", 20, HEADER_HEIGHT + VALUE_HEIGHT * 3);
    // Right column
    tft.drawString("Battery:", RIGHT_COL, HEADER_HEIGHT);
    tft.drawString("Launch:", RIGHT_COL, HEADER_HEIGHT + VALUE_HEIGHT);
    tft.drawString("Temp:", RIGHT_COL, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    tft.drawString("Max-G:", RIGHT_COL, HEADER_HEIGHT + VALUE_HEIGHT * 3);
    
    // Stats row (full width)
    tft.drawString("Stats:", 20, HEADER_HEIGHT + VALUE_HEIGHT * 4);

    
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

void updateDisplay() {
    tft.setTextColor(VALUE_COLOR, TFT_BLACK);
    
    // Update altitude values
    tft.drawString(String(currentAltitude, 1) + "m    ", 180, HEADER_HEIGHT);
    tft.drawString(String(maxAltitude, 1) + "m    ", 180, HEADER_HEIGHT + VALUE_HEIGHT);
    
    // Update GPS coordinates (more compact format)
    String gpsStr = String(latitude, 5) + "," + String(longitude, 5);
    tft.drawString(gpsStr + "  ", 180, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    
    // Update staleness
    unsigned long timeSinceLastPacket = (millis() - millisAtFirstPacket - lastPacketTime) / 1000;
    String timeStr = String(timeSinceLastPacket) + "s    ";
    tft.drawString(timeStr, 180, HEADER_HEIGHT + VALUE_HEIGHT * 3);
    
    // Update stats and signal strength (using full width)
    float snr = radio->getSNR();
    String statsStr = String(totalPackets) + " packets, " + String(badPackets) + " errors, SNR: " + String(snr, 1) + "dB   ";
    tft.drawString(statsStr, 80, HEADER_HEIGHT + VALUE_HEIGHT * 4);
    
    // Update battery voltage and percentage
    String batteryStr = String(batteryVoltage, 2) + "V " + String(batteryPercent) + "%   ";
    tft.drawString(batteryStr, RIGHT_COL + 60, HEADER_HEIGHT);
    
    // Update launch state
    String launchStr = launchState ? "Launched    " : "Waiting...    ";
    tft.drawString(launchStr, RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT);
    
    // Update temperature
    String tempStr = String(temperature, 1) + "C    ";
    tft.drawString(tempStr, RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    
    // Update max G-force
    String gStr = String(maxG, 1) + "g    ";
    tft.drawString(gStr, RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT * 3);
}
    


bool verifyPacket(uint8_t* data, size_t length) {
    if (length < 2) { // Minimum packet size (version + type)
        badPackets++;
        return false;
    }
    
    // Verify checksum
    uint8_t expectedChecksum = calculateChecksum(data, length);
    uint8_t receivedChecksum = data[length - 1];
    if (expectedChecksum != receivedChecksum) {
        badChecksums++;
        Serial.printf("Checksum mismatch: expected 0x%02X, got 0x%02X\n", expectedChecksum, receivedChecksum);
        return false;
    }
    
    return true;
}

void processGpsPacket(uint8_t* data) {
    if (!verifyPacket(data, sizeof(GpsDataPacket))) {
        return;
    }
    
    GpsDataPacket* packet = (GpsDataPacket*)data;
    
    // Convert fixed-point coordinates back to floating point
    latitude = packet->latitude / 1000000.0f;
    longitude = packet->longitude / 1000000.0f;
    gpsAltitude = packet->altitude;
    batteryVoltage = packet->batteryMillivolts / 1000.0f;
    batteryPercent = packet->batteryPercent;

    
    if (millisAtFirstPacket == 0) {
        millisAtFirstPacket = millis();
    }   
    lastPacketTime = packet->timestamp;
    
    updateDisplay();
}

void processAltitudePacket(uint8_t* data) {
    if (!verifyPacket(data, sizeof(AltitudePacket))) {
        return;
    }
    
    AltitudePacket* packet = (AltitudePacket*)data;
    
    currentAltitude = packet->currentAltitude;
    maxAltitude = packet->maxAltitude;
    maxG = packet->maxG;
    launchState = packet->launchState;
    temperature = packet->temperature;
    
    if (millisAtFirstPacket == 0) {
        millisAtFirstPacket = millis();
    }  
    lastPacketTime = packet->timestamp;
    
    updateDisplay();
}

void loop() {
    uint8_t data[MAX_PACKET_SIZE];
    int packetSize = radio->receive(data, MAX_PACKET_SIZE);
    
    if (packetSize > 0) {
        totalPackets++;
        
        // Check packet version
        if (data[0] != PROTOCOL_VERSION) {
            Serial.println(F("Invalid protocol version"));
            badPackets++;
            return;
        }
        
        // Process packet based on type
        switch (data[1]) {
            case GPS_DATA_PACKET:
                processGpsPacket(data);
                break;
            case ALTITUDE_PACKET:
                processAltitudePacket(data);
                break;
            default:
                Serial.println(F("Unknown packet type"));
                break;
        }
    }
    
    // Update display every second regardless of new data
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 1000) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
}
