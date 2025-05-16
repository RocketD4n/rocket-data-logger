#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"

// CC1101 configuration for ESP32
#define CC1101_CS 5    // GPIO5
#define CC1101_GDO0 4  // GPIO4

// Touch screen configuration
#define TOUCH_CS 21
XPT2046_Touchscreen ts(TOUCH_CS);

// Initialize TFT display
TFT_eSPI tft = TFT_eSPI();

// Radio instance using CC1101
Radio* radio = new CC1101Radio(CC1101_CS, CC1101_GDO0, RADIOLIB_NC);

// Forward declarations of functions
void drawNavigationButtons();
void drawMainPage();
void updateMainPageValues();
void drawGraphPage();
void updateGraph();
void updateDisplay();
void sendSnrFeedback();

// Display layout constants
#define HEADER_HEIGHT 30
#define VALUE_HEIGHT 26
#define RIGHT_COL 180
#define LABEL_COLOR TFT_YELLOW
#define VALUE_COLOR TFT_WHITE

// Navigation button constants
#define NAV_BUTTON_WIDTH 40
#define NAV_BUTTON_HEIGHT 40
#define NAV_BUTTON_MARGIN 5
#define NAV_BUTTON_COLOR TFT_BLUE

// Graph constants
#define GRAPH_X 50
#define GRAPH_Y 50
#define GRAPH_WIDTH 220
#define GRAPH_HEIGHT 140
#define GRAPH_COLOR TFT_GREEN
#define AXIS_COLOR TFT_WHITE
#define MAX_DATA_POINTS 100

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
int8_t txPower = 0;  // Transmission power in dBm

// Display page control
int currentPage = 0; // 0 = main data page, 1 = altitude graph page

// Altitude history for graph
float altitudeHistory[MAX_DATA_POINTS];
unsigned long timeHistory[MAX_DATA_POINTS];
int historyIndex = 0;
int historyCount = 0;

// Packet statistics
uint32_t totalPackets = 0;
uint32_t badPackets = 0;
uint32_t badChecksums = 0;

// SNR feedback control
unsigned long lastSnrFeedbackTime = 0;
const unsigned long SNR_FEEDBACK_INTERVAL = 1000; // Send SNR feedback every 1 second

void setup() {
    Serial.begin(115200);
    
    // Initialize display
    tft.init();
    tft.setRotation(3); // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    
    // Initialize touch screen
    ts.begin();
    ts.setRotation(3);
    
    // Initialize the first page
    drawMainPage();

    
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

// Draw navigation buttons
void drawNavigationButtons() {
    // Left button (back)
    tft.fillRoundRect(NAV_BUTTON_MARGIN, NAV_BUTTON_MARGIN, 
                     NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT, 5, NAV_BUTTON_COLOR);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM); // Middle center
    tft.drawString("<", NAV_BUTTON_MARGIN + NAV_BUTTON_WIDTH/2, 
                  NAV_BUTTON_MARGIN + NAV_BUTTON_HEIGHT/2, 4);
    
    // Right button (forward)
    tft.fillRoundRect(tft.width() - NAV_BUTTON_MARGIN - NAV_BUTTON_WIDTH, NAV_BUTTON_MARGIN, 
                     NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT, 5, NAV_BUTTON_COLOR);
    tft.drawString(">", tft.width() - NAV_BUTTON_MARGIN - NAV_BUTTON_WIDTH/2, 
                  NAV_BUTTON_MARGIN + NAV_BUTTON_HEIGHT/2, 4);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Draw the main data page (page 0)
void drawMainPage() {
    tft.fillScreen(TFT_BLACK);
    drawNavigationButtons();
    
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
    tft.drawString("TX Power:", RIGHT_COL, HEADER_HEIGHT + VALUE_HEIGHT * 4);
    
    // Stats row (full width)
    tft.drawString("Stats:", 20, HEADER_HEIGHT + VALUE_HEIGHT * 5);
    
    // Update values
    updateMainPageValues();
}

// Update just the values on the main page
void updateMainPageValues() {
    if (currentPage != 0) return;
    
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
    tft.drawString(statsStr, 80, HEADER_HEIGHT + VALUE_HEIGHT * 5);
    
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
    String maxGStr = String(maxG, 1) + "g    ";
    tft.drawString(maxGStr, RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT * 3);
    
    // Update TX power
    String powerStr = String(txPower) + " dBm    ";
    tft.drawString(powerStr, RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT * 4);
}

// Draw the altitude graph page (page 1)
void drawGraphPage() {
    tft.fillScreen(TFT_BLACK);
    drawNavigationButtons();
    
    // Draw page title
    tft.setTextColor(LABEL_COLOR, TFT_BLACK);
    tft.drawString("Altitude vs Time", 100, 10);
    
    // Draw graph axes
    tft.drawLine(GRAPH_X, GRAPH_Y, GRAPH_X, GRAPH_Y + GRAPH_HEIGHT, AXIS_COLOR); // Y-axis
    tft.drawLine(GRAPH_X, GRAPH_Y + GRAPH_HEIGHT, GRAPH_X + GRAPH_WIDTH, GRAPH_Y + GRAPH_HEIGHT, AXIS_COLOR); // X-axis
    
    // Draw axis labels
    tft.setTextColor(LABEL_COLOR, TFT_BLACK);
    tft.drawString("Time (s)", GRAPH_X + GRAPH_WIDTH/2 - 30, GRAPH_Y + GRAPH_HEIGHT + 10);
    
    // Rotate text for Y-axis label
    tft.setRotation(2); // Temporarily rotate to draw vertical text
    tft.drawString("Altitude (m)", tft.height() - (GRAPH_Y + GRAPH_HEIGHT/2 + 30), tft.width() - GRAPH_X + 10);
    tft.setRotation(3); // Restore rotation
    
    // Draw Y-axis scale
    if (maxAltitude > 0) {
        tft.setTextColor(AXIS_COLOR, TFT_BLACK);
        tft.drawString("0", GRAPH_X - 20, GRAPH_Y + GRAPH_HEIGHT - 5);
        tft.drawString(String(maxAltitude, 0), GRAPH_X - 25, GRAPH_Y - 5);
        tft.drawString(String(maxAltitude/2, 0), GRAPH_X - 25, GRAPH_Y + GRAPH_HEIGHT/2 - 5);
    }
    
    // Update the graph with current data
    updateGraph();
}

// Update the altitude graph
void updateGraph() {
    if (currentPage != 1 || historyCount == 0) return;
    
    // Clear graph area (not the axes)
    tft.fillRect(GRAPH_X + 1, GRAPH_Y, GRAPH_WIDTH - 1, GRAPH_HEIGHT, TFT_BLACK);
    
    // Find max time for scaling X-axis
    unsigned long maxTime = 0;
    for (int i = 0; i < historyCount; i++) {
        int idx = (historyIndex - historyCount + i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
        if (timeHistory[idx] > maxTime) {
            maxTime = timeHistory[idx];
        }
    }
    
    // Draw time scale on X-axis
    if (maxTime > 0) {
        tft.setTextColor(AXIS_COLOR, TFT_BLACK);
        tft.drawString("0", GRAPH_X - 5, GRAPH_Y + GRAPH_HEIGHT + 5);
        tft.drawString(String(maxTime/1000), GRAPH_X + GRAPH_WIDTH - 15, GRAPH_Y + GRAPH_HEIGHT + 5);
    }
    
    // Plot the data points
    if (historyCount > 1 && maxAltitude > 0 && maxTime > 0) {
        for (int i = 1; i < historyCount; i++) {
            int idx1 = (historyIndex - historyCount + i - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS;
            int idx2 = (historyIndex - historyCount + i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
            
            // Scale data points to graph dimensions
            int x1 = GRAPH_X + (timeHistory[idx1] * GRAPH_WIDTH) / maxTime;
            int y1 = GRAPH_Y + GRAPH_HEIGHT - (altitudeHistory[idx1] * GRAPH_HEIGHT) / maxAltitude;
            int x2 = GRAPH_X + (timeHistory[idx2] * GRAPH_WIDTH) / maxTime;
            int y2 = GRAPH_Y + GRAPH_HEIGHT - (altitudeHistory[idx2] * GRAPH_HEIGHT) / maxAltitude;
            
            // Draw line between points
            tft.drawLine(x1, y1, x2, y2, GRAPH_COLOR);
        }
    }
}

// Update display based on current page
void updateDisplay() {
    if (currentPage == 0) {
        updateMainPageValues();
    } else if (currentPage == 1) {
        updateGraph();
    }
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
    txPower = packet->txPower;

    
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
    txPower = packet->txPower;
    
    if (millisAtFirstPacket == 0) {
        millisAtFirstPacket = millis();
    }  
    lastPacketTime = packet->timestamp;
    
    // Store altitude data for graph
    altitudeHistory[historyIndex] = currentAltitude;
    timeHistory[historyIndex] = millis() - millisAtFirstPacket;
    historyIndex = (historyIndex + 1) % MAX_DATA_POINTS;
    if (historyCount < MAX_DATA_POINTS) {
        historyCount++;
    }
    
    updateDisplay();
}

// Check if touch is within button area
bool isTouchInButton(uint16_t x, uint16_t y, uint16_t btnX, uint16_t btnY, uint16_t btnW, uint16_t btnH) {
    return (x >= btnX && x <= btnX + btnW && y >= btnY && y <= btnY + btnH);
}

// Handle touch events
void handleTouch() {
    if (!ts.touched()) {
        return;
    }
    
    // Get touch coordinates
    TS_Point p = ts.getPoint();
    
    // Convert touch coordinates to screen coordinates
    uint16_t x = map(p.x, 0, 4095, 0, tft.width());
    uint16_t y = map(p.y, 0, 4095, 0, tft.height());
    
    // Check if left navigation button was pressed
    if (isTouchInButton(x, y, NAV_BUTTON_MARGIN, NAV_BUTTON_MARGIN, NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT)) {
        // Go to previous page
        currentPage--;
        if (currentPage < 0) currentPage = 1; // Wrap around to last page
        
        tft.fillScreen(TFT_BLACK);
        if (currentPage == 0) {
            drawMainPage();
        } else {
            drawGraphPage();
        }
    }
    
    // Check if right navigation button was pressed
    if (isTouchInButton(x, y, tft.width() - NAV_BUTTON_WIDTH - NAV_BUTTON_MARGIN, NAV_BUTTON_MARGIN, NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT)) {
        // Go to next page
        currentPage++;
        if (currentPage > 1) currentPage = 0; // Wrap around to first page
        
        tft.fillScreen(TFT_BLACK);
        if (currentPage == 0) {
            drawMainPage();
        } else {
            drawGraphPage();
        }
    }
    
    delay(50); // Debounce
}

// Send SNR feedback to the logger
void sendSnrFeedback() {
    // Only send feedback if we've received at least one packet
    if (totalPackets == 0) {
        return;
    }
    
    // Create SNR feedback packet
    SnrFeedbackPacket packet;
    packet.version = PROTOCOL_VERSION;
    packet.packetType = SNR_FEEDBACK_PACKET;
    packet.subType = 0x01; // SNR data
    
    // Get current SNR from radio
    packet.snrValue = radio->getSNR();
    
    // Calculate checksum
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(SnrFeedbackPacket) - 1);
    
    // Transmit packet
    if (radio->transmit((uint8_t*)&packet, sizeof(SnrFeedbackPacket))) {
        Serial.print("Sent SNR feedback: ");
        Serial.println(packet.snrValue);
    } else {
        Serial.println("Failed to send SNR feedback");
    }
}

void loop() {
    // Handle touch events
    handleTouch();
    
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
                // Send SNR feedback after processing GPS packets
                if (millis() - lastSnrFeedbackTime >= SNR_FEEDBACK_INTERVAL) {
                    sendSnrFeedback();
                    lastSnrFeedbackTime = millis();
                }
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
