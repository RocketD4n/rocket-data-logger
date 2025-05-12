#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <RadioLib.h>
#include "rocket_telemetry_protocol.h"

// CC1101 configuration for ESP32
#define CC1101_CS 5    // GPIO5
#define CC1101_GDO0 4  // GPIO4

// Initialize TFT display
TFT_eSPI tft = TFT_eSPI();

// CC1101 instance
Module* module = new Module(CC1101_CS, CC1101_GDO0, RADIOLIB_NC, RADIOLIB_NC);
CC1101* radio = new CC1101(module);

// Display layout constants
#define HEADER_HEIGHT 30
#define VALUE_HEIGHT 25
#define LABEL_COLOR TFT_YELLOW
#define VALUE_COLOR TFT_WHITE

// Global variables for tracking data
float currentAltitude = 0.0f;
float maxAltitude = 0.0f;
float latitude = 0.0f;
float longitude = 0.0f;
float gpsAltitude = 0.0f;
unsigned long lastPacketTime = 0;

void setup() {
    Serial.begin(115200);
    
    // Initialize display
    tft.init();
    tft.setRotation(3); // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    
    // Draw static labels
    tft.setTextColor(LABEL_COLOR);
    tft.drawString("Current Alt:", 10, HEADER_HEIGHT);
    tft.drawString("Max Alt:", 10, HEADER_HEIGHT + VALUE_HEIGHT);
    tft.drawString("GPS:", 10, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    tft.drawString("Last Update:", 10, HEADER_HEIGHT + VALUE_HEIGHT * 3);
    
    // Initialize CC1101
    Serial.print(F("[CC1101] Initializing ... "));
    int state = radio->begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }
    
    // Set frequency to 433.92 MHz (same as transmitter)
    if (radio->setFrequency(433.92) == RADIOLIB_ERR_NONE) {
        Serial.println(F("Frequency set success!"));
    }
}

void updateDisplay() {
    tft.setTextColor(VALUE_COLOR, TFT_BLACK);
    
    // Update altitude values
    tft.drawString(String(currentAltitude, 1) + "m    ", 120, HEADER_HEIGHT);
    tft.drawString(String(maxAltitude, 1) + "m    ", 120, HEADER_HEIGHT + VALUE_HEIGHT);
    
    // Update GPS coordinates
    String gpsStr = String(latitude, 6) + ", " + String(longitude, 6);
    tft.drawString(gpsStr + "    ", 120, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    
    // Update last packet time
    unsigned long timeSinceLastPacket = (millis() - lastPacketTime) / 1000;
    String timeStr = String(timeSinceLastPacket) + "s ago    ";
    tft.drawString(timeStr, 120, HEADER_HEIGHT + VALUE_HEIGHT * 3);
}

void processGpsPacket(uint8_t* data) {
    GpsDataPacket* packet = (GpsDataPacket*)data;
    
    // Convert fixed-point coordinates back to floating point
    latitude = packet->latitude / 1000000.0f;
    longitude = packet->longitude / 1000000.0f;
    gpsAltitude = packet->altitude;
    
    lastPacketTime = millis();
    updateDisplay();
}

void processAltitudePacket(uint8_t* data) {
    AltitudePacket* packet = (AltitudePacket*)data;
    
    currentAltitude = packet->currentAltitude;
    maxAltitude = packet->maxAltitude;
    
    lastPacketTime = millis();
    updateDisplay();
}

void loop() {
    uint8_t data[MAX_PACKET_SIZE];
    int packetSize = radio->receive(data, MAX_PACKET_SIZE);
    
    if (packetSize > 0) {
        // Check packet version
        if (data[0] != PROTOCOL_VERSION) {
            Serial.println(F("Invalid protocol version"));
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
