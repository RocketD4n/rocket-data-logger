#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Wire.h>
#include <MAX1704X.h>
#include <Preferences.h>
#include "rocket_telemetry_protocol.h"
#include "cc1101_radio.h"

// CC1101 configuration for ESP32
#define CC1101_CS 5    // GPIO5
#define CC1101_GDO0 4  // GPIO4

// Touch screen configuration
#define TOUCH_CS 21
XPT2046_Touchscreen ts(TOUCH_CS);

// Battery monitor configuration
#define MAX17043_SDA 32
#define MAX17043_SCL 22
#define LOW_BATTERY_THRESHOLD 45.0
MAX1704X batteryMonitor(0.00125f);

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
void drawTransmitterSelectionPage();
void drawKeyboard();
void processKeyPress(char key);
void saveRocketName();
void loadRocketNames();

// Display layout constants
#define HEADER_HEIGHT 30
#define BATTERY_ICON_WIDTH 25
#define BATTERY_ICON_HEIGHT 12
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

float currentAltitude = 0.0f;
float maxAltitude = 0.0f;
float currentLat = 0.0f;
float currentLng = 0.0f;
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;
float receiverBatteryVoltage = 0.0f;
float receiverBatteryPercent = 0.0f;
float temperature = 0.0f;
float maxG = 0.0f;
int8_t txPower = 0;
uint8_t launchState = 0;
uint32_t rocketUptime = 0; // Rocket uptime in milliseconds

unsigned long lastPacketTime = 0;
unsigned long packetCount = 0;
unsigned long errorCount = 0;
float lastSnr = 0.0f;

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

// Display page control
int currentPage = 4; // 0 = main data page, 1-3 = graph pages, 4 = transmitter selection page
bool rocketSelected = false; // Flag to indicate if a rocket has been selected
const int NUM_PAGES = 5; // Main page + 3 graph pages + transmitter selection page

// Graph data history
float dataHistory[3][MAX_DATA_POINTS]; // 0=altitude, 1=speed, 2=txPower
unsigned long timeHistory[MAX_DATA_POINTS];
int historyIndex = 0;
int historyCount = 0;
unsigned long millisAtFirstPacket = 0; // Timestamp when first packet was received

// Speed calculation variables
float currentSpeed = 0.0f;
float lastAltitude = 0.0f;
unsigned long lastAltitudeTime = 0;

// Graph configuration
const char* graphTitles[] = {"Altitude vs Time", "Speed vs Time", "TX Power vs Time"};
const char* graphYLabels[] = {"Altitude (m)", "Speed (m/s)", "Power (dBm)"};
const uint16_t graphColors[] = {TFT_GREEN, TFT_CYAN, TFT_YELLOW};

// Packet statistics
uint32_t totalPackets = 0;
uint32_t badPackets = 0;
uint32_t badChecksums = 0;

// SNR feedback control
unsigned long lastSnrFeedbackTime = 0;
const unsigned long SNR_FEEDBACK_INTERVAL = 1000; // Send SNR feedback every 1 second

// Load rocket names from preferences storage
void loadRocketNames() {
    preferences.begin("rockets", false); // false = read/write mode
    
    // Load saved rocket names
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
        String key = String(i);
        String name = preferences.getString(key.c_str(), "");
        
        // Check if we have a transmitter ID saved for this slot
        String idKey = "id" + key;
        uint32_t savedId = preferences.getUInt(idKey.c_str(), 0);
        
        if (savedId != 0) {
            // Find if this ID is already in our known transmitters list
            bool found = false;
            for (int j = 0; j < numTransmitters; j++) {
                if (knownTransmitters[j] == savedId) {
                    rocketNames[j] = name;
                    found = true;
                    break;
                }
            }
            
            // If not found and we have space, add it
            if (!found && numTransmitters < MAX_TRANSMITTERS) {
                knownTransmitters[numTransmitters] = savedId;
                rocketNames[numTransmitters] = name;
                numTransmitters++;
            }
        }
    }
    
    preferences.end();
}

// Save rocket name to preferences storage
void saveRocketName() {
    if (editingTransmitterIndex < 0 || editingTransmitterIndex >= numTransmitters) {
        return;
    }
    
    // Trim and limit name length
    currentInput.trim();
    if (currentInput.length() > MAX_NAME_LENGTH) {
        currentInput = currentInput.substring(0, MAX_NAME_LENGTH);
    }
    
    // Save the name
    rocketNames[editingTransmitterIndex] = currentInput;
    
    // Save to preferences
    preferences.begin("rockets", false);
    String key = String(editingTransmitterIndex);
    preferences.putString(key.c_str(), currentInput);
    
    // Also save the transmitter ID
    String idKey = "id" + key;
    preferences.putUInt(idKey.c_str(), knownTransmitters[editingTransmitterIndex]);
    
    preferences.end();
    
    // Reset editing state
    editingName = false;
    showKeyboard = false;
    currentInput = "";
    editingTransmitterIndex = -1;
}

void setup() {
    Serial.begin(115200);
    
    // Initialize preferences
    preferences.begin("rockets", false);
    preferences.end();
    
    // Load saved rocket names
    loadRocketNames();
    
    // Initialize I2C for battery monitor
    Wire.begin(MAX17043_SDA, MAX17043_SCL);
    
    // Initialize battery monitor
    batteryMonitor.reset();
    batteryMonitor.quickstart();
    if (!batteryMonitor.begin()) {
        Serial.println("Failed to initialize battery monitor!");
    } else {
        Serial.println("Battery monitor initialized successfully");
        receiverBatteryPercent = batteryMonitor.percent();
        Serial.print("Initial battery: ");
        Serial.print(receiverBatteryPercent);
        Serial.println("%");
    }
    
    // Initialize display
    tft.init();
    tft.setRotation(3); // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    
    // Initialize touch screen
    ts.begin();
    ts.setRotation(3);
    
    // Initialize with the transmitter selection page
    drawTransmitterSelectionPage();

    
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

// Draw battery icon and level at the top center of the screen
void drawBatteryIndicator() {
    int centerX = tft.width() / 2;
    int batteryX = centerX - BATTERY_ICON_WIDTH / 2;
    int batteryY = 5;
    
    // Draw battery outline
    tft.drawRect(batteryX, batteryY, BATTERY_ICON_WIDTH, BATTERY_ICON_HEIGHT, TFT_WHITE);
    tft.drawRect(batteryX + BATTERY_ICON_WIDTH, batteryY + 2, 3, BATTERY_ICON_HEIGHT - 4, TFT_WHITE);
    
    // Draw battery fill based on percentage
    int fillWidth = (receiverBatteryPercent / 100.0) * (BATTERY_ICON_WIDTH - 2);
    
    // Choose color based on battery level
    uint16_t fillColor;
    if (receiverBatteryPercent > 75) {
        fillColor = TFT_GREEN;
    } else if (receiverBatteryPercent > 45) {
        fillColor = TFT_YELLOW;
    } else {
        fillColor = TFT_RED;
    }
    
    // Fill battery icon
    tft.fillRect(batteryX + 1, batteryY + 1, fillWidth, BATTERY_ICON_HEIGHT - 2, fillColor);
    
    // Draw percentage text
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(String((int)receiverBatteryPercent) + "%", centerX, batteryY + BATTERY_ICON_HEIGHT + 8, 2);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Draw navigation buttons
void drawNavigationButtons() {
    // Only show navigation buttons if a rocket has been selected
    if (!rocketSelected) {
        return;
    }
    
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
    drawBatteryIndicator();
    
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
    String gpsStr = String(currentLat, 5) + "," + String(currentLng, 5);
    tft.drawString(gpsStr + "  ", 180, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    
    // Update staleness
    unsigned long staleness = (millis() - lastPacketTime) / 1000;
    String stalenessStr = String(staleness) + "s";
    tft.drawString(stalenessStr + "  ", 180, HEADER_HEIGHT + VALUE_HEIGHT * 3);
    
    // Update battery
    String batteryStr = String(batteryVoltage, 2) + "V " + String(batteryPercent) + "%";
    tft.drawString(batteryStr + "  ", RIGHT_COL + 60, HEADER_HEIGHT);
    
    // Update launch state
    String launchStateStr;
    switch (launchState) {
        case 0: launchStateStr = "Waiting..."; break;
        case 1: launchStateStr = "Launched"; break;
        case 2: launchStateStr = "Apogee"; break;
        case 3: launchStateStr = "Landed"; break;
        default: launchStateStr = "Unknown"; break;
    }
    tft.drawString(launchStateStr + "  ", RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT);
    
    // Update temperature
    String tempStr = String(temperature, 1) + "C";
    tft.drawString(tempStr + "  ", RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT * 2);
    
    // Update max G
    String maxGStr = String(maxG, 1) + "g";
    tft.drawString(maxGStr + "  ", RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT * 3);
    
    // Update TX power
    String powerStr = String(txPower) + " dBm";
    tft.drawString(powerStr, RIGHT_COL + 60, HEADER_HEIGHT + VALUE_HEIGHT * 4);
    
    // Update packet stats
    String statsStr = String(packetCount) + " pkts, " + String(errorCount) + " errs, SNR: " + String(lastSnr, 1) + "dB";
    tft.drawString(statsStr + "  ", 10, HEADER_HEIGHT + VALUE_HEIGHT * 5);
}

// Draw a graph page (pages 1-3)
void drawGraphPage() {
    tft.fillScreen(TFT_BLACK);
    drawNavigationButtons();
    drawBatteryIndicator();
    
    // Get the graph index (0-2) from the current page (1-3)
    int graphIndex = currentPage - 1;
    
    // Draw page title
    tft.setTextColor(LABEL_COLOR, TFT_BLACK);
    tft.drawString(graphTitles[graphIndex], 100, 10);
    
    // Draw graph axes
    tft.drawLine(GRAPH_X, GRAPH_Y, GRAPH_X, GRAPH_Y + GRAPH_HEIGHT, AXIS_COLOR); // Y-axis
    tft.drawLine(GRAPH_X, GRAPH_Y + GRAPH_HEIGHT, GRAPH_X + GRAPH_WIDTH, GRAPH_Y + GRAPH_HEIGHT, AXIS_COLOR); // X-axis
    
    // Draw axis labels
    tft.setTextColor(LABEL_COLOR, TFT_BLACK);
    tft.drawString("Time (s)", GRAPH_X + GRAPH_WIDTH/2 - 30, GRAPH_Y + GRAPH_HEIGHT + 10);
    
    // Rotate text for Y-axis label
    tft.setRotation(2); // Temporarily rotate to draw vertical text
    tft.drawString(graphYLabels[graphIndex], tft.height() - (GRAPH_Y + GRAPH_HEIGHT/2 + 30), tft.width() - GRAPH_X + 10);
    tft.setRotation(3); // Restore rotation
    
    // Draw Y-axis scale based on graph type
    float maxValue = 0;
    switch (graphIndex) {
        case 0: // Altitude
            maxValue = maxAltitude;
            break;
        case 1: // Speed
            maxValue = 30.0f; // Reasonable max speed in m/s
            for (int i = 0; i < historyCount; i++) {
                int idx = (historyIndex - historyCount + i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
                if (dataHistory[1][idx] > maxValue) maxValue = dataHistory[1][idx];
            }
            break;
        case 2: // TX Power
            maxValue = 20.0f; // Max TX power in dBm
            break;
    }
    
    if (maxValue > 0) {
        tft.setTextColor(AXIS_COLOR, TFT_BLACK);
        tft.drawString("0", GRAPH_X - 20, GRAPH_Y + GRAPH_HEIGHT - 5);
        tft.drawString(String(maxValue, 0), GRAPH_X - 25, GRAPH_Y - 5);
        tft.drawString(String(maxValue/2, 0), GRAPH_X - 25, GRAPH_Y + GRAPH_HEIGHT/2 - 5);
    }
    
    // Update the graph with current data
    updateGraph();
}

// Update the current graph
void updateGraph() {
    if (currentPage < 1 || currentPage > 3 || historyCount == 0) return;
    
    // Get the graph index (0-2) from the current page (1-3)
    int graphIndex = currentPage - 1;
    
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
    
    // Get max value for Y-axis scaling
    float maxValue = 0;
    switch (graphIndex) {
        case 0: // Altitude
            maxValue = maxAltitude;
            break;
        case 1: // Speed
            maxValue = 30.0f; // Reasonable max speed in m/s
            for (int i = 0; i < historyCount; i++) {
                int idx = (historyIndex - historyCount + i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
                if (dataHistory[1][idx] > maxValue) maxValue = dataHistory[1][idx];
            }
            break;
        case 2: // TX Power
            maxValue = 20.0f; // Max TX power in dBm
            break;
    }
    
    // Plot the data points
    if (historyCount > 1 && maxValue > 0 && maxTime > 0) {
        for (int i = 1; i < historyCount; i++) {
            int idx1 = (historyIndex - historyCount + i - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS;
            int idx2 = (historyIndex - historyCount + i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
            
            // Scale data points to graph dimensions
            int x1 = GRAPH_X + (timeHistory[idx1] * GRAPH_WIDTH) / maxTime;
            int y1 = GRAPH_Y + GRAPH_HEIGHT - (dataHistory[graphIndex][idx1] * GRAPH_HEIGHT) / maxValue;
            int x2 = GRAPH_X + (timeHistory[idx2] * GRAPH_WIDTH) / maxTime;
            int y2 = GRAPH_Y + GRAPH_HEIGHT - (dataHistory[graphIndex][idx2] * GRAPH_HEIGHT) / maxValue;
            
            // Draw line between points
            tft.drawLine(x1, y1, x2, y2, graphColors[graphIndex]);
        }
    }
}

// Update display based on current page
void updateDisplay() {
    if (currentPage == 0) {
        updateMainPageValues();
    } else if (currentPage >= 1 && currentPage <= 3) {
        updateGraph();
    } else if (currentPage == 4) {
        drawTransmitterSelectionPage();
    }
}

// Draw the on-screen keyboard
void drawKeyboard() {
    // Draw keyboard background
    tft.fillRect(0, 95, tft.width(), tft.height() - 95, TFT_DARKGREY);
    
    // Draw current input field
    tft.fillRect(10, 100, tft.width() - 20, 30, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(currentInput + "_", 15, 105, 2);
    
    // Draw keyboard keys
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 10; col++) {
            int x = KEYBOARD_X + col * (KEY_WIDTH + 2);
            int y = KEYBOARD_Y + 40 + row * (KEY_HEIGHT + 2);
            
            // Special handling for function keys
            if (row == 3 && col == 7) { // Backspace key
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_BLUE);
                tft.setTextColor(TFT_WHITE);
                tft.setTextDatum(MC_DATUM);
                tft.drawString("<", x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 2);
            } 
            else if (row == 3 && col == 9) { // Enter/Done key
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_GREEN);
                tft.setTextColor(TFT_BLACK);
                tft.setTextDatum(MC_DATUM);
                tft.drawString(">", x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 2);
            }
            else if (row == 3 && col == 8) { // Space key
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_LIGHTGREY);
                tft.setTextColor(TFT_BLACK);
                tft.setTextDatum(MC_DATUM);
                tft.drawString("_", x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 2);
            }
            else {
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_LIGHTGREY);
                tft.setTextColor(TFT_BLACK);
                tft.setTextDatum(MC_DATUM);
                tft.drawString(String(keyboardChars[row][col]), x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 2);
            }
        }
    }
    
    // Draw cancel button
    tft.fillRect(10, tft.height() - 40, 100, 30, TFT_RED);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("CANCEL", 60, tft.height() - 25, 2);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Process key press on the on-screen keyboard
void processKeyPress(int x, int y) {
    // Check if cancel button was pressed
    if (y >= tft.height() - 40 && y <= tft.height() - 10 && x >= 10 && x <= 110) {
        // Cancel editing
        editingName = false;
        showKeyboard = false;
        currentInput = "";
        editingTransmitterIndex = -1;
        drawTransmitterSelectionPage();
        return;
    }
    
    // Check if a key was pressed
    if (y >= KEYBOARD_Y + 40 && y < KEYBOARD_Y + 40 + 4 * (KEY_HEIGHT + 2)) {
        int row = (y - (KEYBOARD_Y + 40)) / (KEY_HEIGHT + 2);
        int col = (x - KEYBOARD_X) / (KEY_WIDTH + 2);
        
        if (row >= 0 && row < 4 && col >= 0 && col < 10) {
            // Handle special keys
            if (row == 3 && col == 7) { // Backspace
                if (currentInput.length() > 0) {
                    currentInput = currentInput.substring(0, currentInput.length() - 1);
                }
            }
            else if (row == 3 && col == 9) { // Done/Enter
                saveRocketName();
                drawTransmitterSelectionPage();
                return;
            }
            else if (row == 3 && col == 8) { // Space
                currentInput += " ";
            }
            else { // Regular character
                if (currentInput.length() < MAX_NAME_LENGTH) {
                    currentInput += keyboardChars[row][col];
                }
            }
            
            // Redraw keyboard with updated input
            drawKeyboard();
        }
    }
}

// Draw the transmitter selection page (page 4)
void drawTransmitterSelectionPage() {
    tft.fillScreen(TFT_BLACK);
    drawBatteryIndicator();
    if (rocketSelected) {
        drawNavigationButtons();
    }
    
    tft.setTextColor(LABEL_COLOR, TFT_BLACK);
    tft.drawString("Select Transmitter:", 10, 10);
    
    // If no transmitters found yet, show searching message
    if (numTransmitters == 0) {
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Searching for rockets...", tft.width()/2, tft.height()/2, 2);
        tft.setTextDatum(TL_DATUM);
        return;
    }
    
    // If keyboard is shown, draw it and return
    if (showKeyboard && editingName) {
        drawKeyboard();
        return;
    }
    
    // Draw list of known transmitters with names and uptime
    for (int i = 0; i < numTransmitters; i++) {
        // Show either name (if available) or ID
        String displayText;
        if (rocketNames[i].length() > 0) {
            displayText = rocketNames[i];
        } else {
            displayText = "ID: 0x" + String(knownTransmitters[i], HEX);
        }
        
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(displayText, 10, 40 + i * 30);
        
        // Draw edit button
        tft.fillRect(tft.width() - 60, 40 + i * 30, 50, 20, TFT_BLUE);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("NAME", tft.width() - 35, 40 + i * 30 + 10, 1);
        tft.setTextDatum(TL_DATUM);
        
        // If this is the selected transmitter, show uptime
        if (knownTransmitters[i] == selectedTransmitterId) {
            // Format uptime nicely (hours:minutes:seconds)
            unsigned long uptimeSec = rocketUptime / 1000;
            unsigned long hours = uptimeSec / 3600;
            unsigned long minutes = (uptimeSec % 3600) / 60;
            unsigned long seconds = uptimeSec % 60;
            
            String uptimeStr = "Uptime: " + String(hours) + ":" + 
                               (minutes < 10 ? "0" : "") + String(minutes) + ":" + 
                               (seconds < 10 ? "0" : "") + String(seconds);
                               
            tft.drawString(uptimeStr, 160, 40 + i * 30);
        }
    }
    
    // Highlight selected transmitter
    if (selectedTransmitterIndex != -1) {
        tft.setTextColor(VALUE_COLOR, TFT_BLACK);
        String displayText;
        if (rocketNames[selectedTransmitterIndex].length() > 0) {
            displayText = rocketNames[selectedTransmitterIndex];
        } else {
            displayText = "ID: 0x" + String(knownTransmitters[selectedTransmitterIndex], HEX);
        }
        tft.drawString(displayText, 10, 40 + selectedTransmitterIndex * 30);
        tft.drawRect(5, 35 + selectedTransmitterIndex * 30, tft.width() - 70, 25, VALUE_COLOR);
    }
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
    
    // Check if left navigation button was pressed (only if a rocket is selected)
    if (rocketSelected && isTouchInButton(x, y, NAV_BUTTON_MARGIN, NAV_BUTTON_MARGIN, NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT)) {
        // Go to previous page
        currentPage--;
        if (currentPage < 0) currentPage = NUM_PAGES - 1; // Wrap around to last page
        
        tft.fillScreen(TFT_BLACK);
        if (currentPage == 0) {
            drawMainPage();
        } else if (currentPage >= 1 && currentPage <= 3) {
            drawGraphPage();
        } else if (currentPage == 4) {
            updateDisplay();
        }
    }
    
    // Check if right navigation button was pressed (only if a rocket is selected)
    if (rocketSelected && isTouchInButton(x, y, tft.width() - NAV_BUTTON_WIDTH - NAV_BUTTON_MARGIN, NAV_BUTTON_MARGIN, NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT)) {
        // Go to next page
        currentPage++;
        if (currentPage >= NUM_PAGES) currentPage = 0; // Wrap around to first page
        
        tft.fillScreen(TFT_BLACK);
        if (currentPage == 0) {
            drawMainPage();
        } else if (currentPage >= 1 && currentPage <= 3) {
            drawGraphPage();
        } else if (currentPage == 4) {
            updateDisplay();
        }
    }
    
    // Check if transmitter selection page is active
    if (currentPage == 4) {
        // If keyboard is active, handle keyboard touches
        if (showKeyboard && editingName) {
            processKeyPress(x, y);
            delay(200); // Debounce
            return;
        }
        
        // Check if an edit button was pressed
        for (int i = 0; i < numTransmitters; i++) {
            if (x >= tft.width() - 60 && x <= tft.width() - 10 && 
                y >= 40 + i * 30 && y <= 60 + i * 30) {
                // Edit button pressed for transmitter i
                editingTransmitterIndex = i;
                editingName = true;
                showKeyboard = true;
                currentInput = rocketNames[i]; // Start with current name if any
                drawTransmitterSelectionPage(); // Will show keyboard
                delay(200); // Debounce
                return;
            }
        }
        
        // Check if a transmitter was selected on the transmitter selection page
        if (y >= 35 && y < 35 + numTransmitters * 30 && x < tft.width() - 65) {
            int index = (y - 35) / 30;
            if (index >= 0 && index < numTransmitters) {
                selectedTransmitterIndex = index;
                selectedTransmitterId = knownTransmitters[index];
                
                // Set the flag that a rocket has been selected
                if (!rocketSelected) {
                    rocketSelected = true;
                    // Switch to main data page after selection
                    currentPage = 0;
                }
                
                updateDisplay();
                delay(200); // Debounce
            }
        }
    }
    
    delay(50); // Debounce
}

// Process GPS packet
void processGpsPacket(uint8_t* buffer, size_t length) {
  if (length < sizeof(GpsDataPacket)) {
    Serial.println("GPS packet too small");
    errorCount++;
    return;
  }
  
  GpsDataPacket* packet = (GpsDataPacket*)buffer;
  
  // Verify checksum
  uint8_t calculatedChecksum = calculateChecksum(buffer, length - 1);
  if (calculatedChecksum != packet->checksum) {
    Serial.println("GPS packet checksum mismatch");
    errorCount++;
    return;
  }
  
  // Check if this is a new transmitter
  bool isNewTransmitter = true;
  for (int i = 0; i < numTransmitters; i++) {
    if (knownTransmitters[i] == packet->transmitterId) {
      isNewTransmitter = false;
      break;
    }
  }
  
  // Add to known transmitters if new
  if (isNewTransmitter && numTransmitters < MAX_TRANSMITTERS) {
    knownTransmitters[numTransmitters] = packet->transmitterId;
    numTransmitters++;
    
    // If this is the first transmitter, select it automatically
    if (numTransmitters == 1) {
      selectedTransmitterIndex = 0;
      selectedTransmitterId = packet->transmitterId;
      rocketSelected = true;
      
      // Switch to main data page after first rocket is found
      currentPage = 0;
      updateDisplay();
    }
  }
  
  // Only process packet if it's from the selected transmitter
  if (packet->transmitterId != selectedTransmitterId) {
    return;
  }
  
  // Convert fixed-point coordinates back to floating point
  currentLat = packet->latitude / 1000000.0f;
  currentLng = packet->longitude / 1000000.0f;
  
  // Record the time of the first packet for graphing time reference
  if (millisAtFirstPacket == 0) {
    millisAtFirstPacket = millis();
  }
  
  lastPacketTime = millis();
  packetCount++;
  
  updateDisplay();
}

// Process altitude packet
void processAltitudePacket(uint8_t* buffer, size_t length) {
  if (length < sizeof(AltitudePacket)) {
    Serial.println("Altitude packet too small");
    errorCount++;
    return;
  }
  
  AltitudePacket* packet = (AltitudePacket*)buffer;
  
  // Verify checksum
  uint8_t calculatedChecksum = calculateChecksum(buffer, length - 1);
  if (calculatedChecksum != packet->checksum) {
    Serial.println("Altitude packet checksum mismatch");
    errorCount++;
    return;
  }
  
  // If this packet is not from the selected transmitter, ignore it
  if (selectedTransmitterIndex != -1 && packet->transmitterId != selectedTransmitterId) {
    return;
  }
  
  currentAltitude = packet->currentAltitude;
  maxAltitude = packet->maxAltitude;
  temperature = packet->temperature / 10.0f; // Convert back to degrees C
  maxG = packet->maxG / 10.0f; // Convert back to g
  launchState = packet->launchState;
  
  lastPacketTime = millis();
  packetCount++;
  
  // Calculate speed from altitude changes
  unsigned long currentTime = millis();
  if (lastAltitudeTime > 0) {
    // Calculate speed in m/s based on altitude change
    float timeDelta = (currentTime - lastAltitudeTime) / 1000.0f; // seconds
    if (timeDelta > 0) {
      currentSpeed = (currentAltitude - lastAltitude) / timeDelta;
      // Apply simple low-pass filter to smooth speed data
      currentSpeed = 0.7f * currentSpeed + 0.3f * dataHistory[1][historyIndex > 0 ? historyIndex - 1 : MAX_DATA_POINTS - 1];
    }
  }
  lastAltitude = currentAltitude;
  lastAltitudeTime = currentTime;
  
  // Store data for graphs
  dataHistory[0][historyIndex] = currentAltitude; // Altitude
  dataHistory[1][historyIndex] = currentSpeed;    // Speed
  dataHistory[2][historyIndex] = txPower;         // TX Power
  timeHistory[historyIndex] = millis() - millisAtFirstPacket;
  historyIndex = (historyIndex + 1) % MAX_DATA_POINTS;
  historyCount = min(historyCount + 1, MAX_DATA_POINTS);
  
  updateDisplay();
}

// Process system data packet
void processSystemPacket(uint8_t* buffer, size_t length) {
  if (length < sizeof(SystemDataPacket)) {
    Serial.println("System packet too small");
    errorCount++;
    return;
  }
  
  SystemDataPacket* packet = (SystemDataPacket*)buffer;
  
  // Verify checksum
  uint8_t calculatedChecksum = calculateChecksum(buffer, length - 1);
  if (calculatedChecksum != packet->checksum) {
    Serial.println("System packet checksum mismatch");
    errorCount++;
    return;
  }
  
  // If this packet is not from the selected transmitter, ignore it
  if (selectedTransmitterIndex != -1 && packet->transmitterId != selectedTransmitterId) {
    return;
  }
  
  // Update system data
  rocketUptime = packet->uptime;
  batteryVoltage = packet->batteryMillivolts / 1000.0f;
  batteryPercent = packet->batteryPercent;
  txPower = packet->txPower;
  
  lastPacketTime = millis();
  packetCount++;
  
  updateDisplay();
}

// Send SNR feedback to the logger
void sendSnrFeedback() {
    if (lastSnr == 0.0f || selectedTransmitterId == 0) {
        return; // No SNR data to send or no transmitter selected
    }
    
    SnrFeedbackPacket packet;
    packet.version = PROTOCOL_VERSION;
    packet.packetType = PACKET_TYPE_FEEDBACK;
    packet.subType = FEEDBACK_SUBTYPE_SNR;
    packet.transmitterId = selectedTransmitterId;
    packet.snrValue = lastSnr;
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    radio->transmit((uint8_t*)&packet, sizeof(packet));
    
    Serial.print("Sent SNR feedback to transmitter 0x");
    Serial.print(selectedTransmitterId, HEX);
    Serial.print(": ");
    Serial.println(lastSnr);
}

// Display low battery shutdown message and shut down
void lowBatteryShutdown() {
    // Clear screen and show shutdown message
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("LOW BATTERY", tft.width()/2, tft.height()/2 - 20, 4);
    tft.drawString("SHUTTING DOWN", tft.width()/2, tft.height()/2 + 20, 4);
    
    // Show battery percentage
    tft.drawString(String((int)receiverBatteryPercent) + "%", tft.width()/2, tft.height()/2 + 60, 4);
    
    // Wait a few seconds to show the message
    delay(5000);
    
    // Turn off display
    tft.fillScreen(TFT_BLACK);
    tft.writecommand(0x28); // DISPOFF command
    tft.writecommand(0x10); // SLPIN command
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

void loop() {
    // Read battery level
    static unsigned long lastBatteryCheck = 0;
    if (millis() - lastBatteryCheck >= 10000) { // Check every 10 seconds
        receiverBatteryPercent = batteryMonitor.percent();
        drawBatteryIndicator(); // Update battery indicator
        
        // Check for low battery condition
        if (receiverBatteryPercent < LOW_BATTERY_THRESHOLD) {
            lowBatteryShutdown();
        }
        
        lastBatteryCheck = millis();
    }
    
    // Handle touch events
    handleTouch();
    
    uint8_t data[MAX_PACKET_SIZE];
    int packetSize = radio->receive(data, MAX_PACKET_SIZE);
    
    if (packetSize > 0) {
        // Check packet version
        if (data[0] != PROTOCOL_VERSION) {
            Serial.println("Invalid protocol version");
            errorCount++;
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
                    errorCount++;
                    break;
            }
            
            // Get SNR from radio
            lastSnr = radio->getSNR();
            
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
        updateDisplay();
        lastDisplayUpdate = millis();
    }
}
