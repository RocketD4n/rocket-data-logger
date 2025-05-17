#ifndef ROCKET_MONITOR_SCREEN_H
#define ROCKET_MONITOR_SCREEN_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>

// Forward declarations
class Radio;

class RocketMonitorScreen {
public:
    // Constructor
    RocketMonitorScreen(TFT_eSPI& tft, XPT2046_Touchscreen& ts);
    
    // Initialization
    void begin();
    void sleepDisplay();
    
    // Constants
    static const int MAX_TRANSMITTERS = 10;
    static const int MAX_NAME_LENGTH = 16;
    static const int MAX_DATA_POINTS = 100;
    static const int NUM_PAGES = 6; // Main page + 3 graph pages + transmitter selection page + last positions page
    
    // Display layout constants
    static const int HEADER_HEIGHT = 30;
    static const int BATTERY_ICON_WIDTH = 25;
    static const int BATTERY_ICON_HEIGHT = 12;
    static const int VALUE_HEIGHT = 26;
    static const int RIGHT_COL = 180;
    static const uint16_t LABEL_COLOR = TFT_YELLOW;
    static const uint16_t VALUE_COLOR = TFT_WHITE;
    
    // Navigation button constants
    static const int NAV_BUTTON_WIDTH = 40;
    static const int NAV_BUTTON_HEIGHT = 40;
    static const int NAV_BUTTON_MARGIN = 5;
    static const uint16_t NAV_BUTTON_COLOR = TFT_BLUE;
    
    // Graph constants
    static const int GRAPH_X = 50;
    static const int GRAPH_Y = 50;
    static const int GRAPH_WIDTH = 220;
    static const int GRAPH_HEIGHT = 140;
    static const uint16_t GRAPH_COLOR = TFT_GREEN;
    static const uint16_t AXIS_COLOR = TFT_WHITE;
    
    // Keyboard constants
    static const int KEY_WIDTH = 30;
    static const int KEY_HEIGHT = 30;
    static const int KEYBOARD_X = 10;
    static const int KEYBOARD_Y = 100;
    
    // Drawing functions
    void drawBatteryIndicator(float batteryPercent);
    void drawNavigationButtons(bool rocketSelected);
    void drawMainPage();
    void updateMainPageValues();
    void drawGraphPage();
    void updateGraph();
    void drawTransmitterSelectionPage();
    void drawLastPositionsPage();
    void drawKeyboard();
    void drawLowBattery();
    
    // Touch handling
    bool handleTouch(int x, int y);
    bool isTouchInButton(uint16_t x, uint16_t y, uint16_t btnX, uint16_t btnY, uint16_t btnW, uint16_t btnH);
    void processKeyPress(int x, int y);
    
    // Rocket naming
    void saveRocketName();
    void loadRocketNames();
    
    // Getters and setters
    int getCurrentPage() const { return currentPage; }
    void setCurrentPage(int page) { currentPage = page; }
    bool isRocketSelected() const { return rocketSelected; }
    void setRocketSelected(bool selected) { rocketSelected = selected; }
    int getSelectedTransmitterIndex() const { return selectedTransmitterIndex; }
    void setSelectedTransmitterIndex(int index) { selectedTransmitterIndex = index; }
    uint32_t getSelectedTransmitterId() const { return selectedTransmitterId; }
    void setSelectedTransmitterId(uint32_t id) { selectedTransmitterId = id; }
    void setReceiverBatteryPercent(uint8_t percent) { batteryPercent = percent; }
    uint8_t getReceiverBatteryPercent() const { return batteryPercent; }
    
    // Get transmitter name for a given ID
    String getTransmitterName(uint32_t transmitterId) const {
        for (int i = 0; i < numTransmitters; i++) {
            if (knownTransmitters[i] == transmitterId) {
                return rocketNames[i];
            }
        }
        return String(transmitterId, HEX); // Return ID as hex if name not found
    }
    
    // Get number of known transmitters
    int getNumTransmitters() const { return numTransmitters; }
    
    // Get transmitter ID by index
    uint32_t getTransmitterId(int index) const {
        if (index >= 0 && index < numTransmitters) {
            return knownTransmitters[index];
        }
        return 0;
    }
    
    // Public data members for telemetry and packet statistics
    // These are made public to allow easier access from the main program
    float currentAltitude = 0.0f;
    float maxAltitude = 0.0f;
    float currentLat = 0.0f;
    float currentLng = 0.0f;
    float batteryVoltage = 0.0f;
    uint8_t batteryPercent = 0;
    float temperature = 0.0f;
    float maxG = 0.0f;
    int8_t txPower = 0;
    uint8_t launchState = 0;
    uint32_t rocketUptime = 0; // Rocket uptime in milliseconds
    
    // Last known position storage
    void saveLastKnownPosition(uint32_t transmitterId, float lat, float lng);
    void clearLastKnownPosition(uint32_t transmitterId);
    bool getLastKnownPosition(uint32_t transmitterId, float &lat, float &lng);
    
    // Buzzer control
    void setBuzzerActive(bool active) { buzzerActive = active; }
    bool isBuzzerActive() const { return buzzerActive; }
    
    // Abort control
    void setAbortSent(bool sent) { abortSent = sent; }
    bool isAbortSent() const { return abortSent; }
    
    // Packet statistics
    unsigned long lastPacketTime = 0;
    int packetCount = 0;
    int errorCount = 0;
    float lastSnr = 0.0f;
    
    // Buzzer and abort states
    bool buzzerActive = false;
    bool abortSent = false;
    
    // Data update methods for different packet types
    void updateGpsData(float lat, float lng);
    void updateAltitudeData(float altitude, float maxAlt, float temp, float maxG, uint8_t launchState);
    void updateSystemData(float battV, uint8_t battPct, int8_t txPwr, uint32_t uptime);
    void updateGraphData(float altitude, float speed, int8_t txPower);
    void addTransmitter(uint32_t transmitterId);
    void updateDisplay();
    

    
private:
    // References to external objects
    TFT_eSPI& tft;
    XPT2046_Touchscreen& ts;
    Preferences rocketNamesStorage; // For storing rocket names
    Preferences lastPositionsStorage; // For storing last known positions
    
    // Display page control
    int currentPage = 4; // 0 = main data page, 1-3 = graph pages, 4 = transmitter selection page
    bool rocketSelected = false; // Flag to indicate if a rocket has been selected
    
    // Transmitter selection
    uint32_t knownTransmitters[MAX_TRANSMITTERS] = {0};
    String rocketNames[MAX_TRANSMITTERS];
    int numTransmitters = 0;
    int selectedTransmitterIndex = -1; // -1 means no transmitter selected
    uint32_t selectedTransmitterId = 0;
    
    // Graph data history
    float dataHistory[3][MAX_DATA_POINTS]; // 0=altitude, 1=speed, 2=txPower
    unsigned long timeHistory[MAX_DATA_POINTS];
    int historyIndex = 0;
    int historyCount = 0;
    unsigned long millisAtFirstPacket = 0; // Timestamp when first packet was received
    
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
    
    // Speed calculation variables (these remain private)
    
    // Speed calculation variables
    float currentSpeed = 0.0f;
    float lastAltitude = 0.0f;
    unsigned long lastAltitudeTime = 0;
    
    // Graph configuration
    const char* graphTitles[3] = {"Altitude vs Time", "Speed vs Time", "TX Power vs Time"};
    const char* graphYLabels[3] = {"Altitude (m)", "Speed (m/s)", "Power (dBm)"};
    const uint16_t graphColors[3] = {TFT_GREEN, TFT_CYAN, TFT_YELLOW};
};

#endif // ROCKET_MONITOR_SCREEN_H
