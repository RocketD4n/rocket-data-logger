#ifndef ROCKET_MONITOR_SCREEN_H
#define ROCKET_MONITOR_SCREEN_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>
#include <SD.h>

// Forward declarations
class Radio;

class RocketMonitorScreen {
public:
    // Constructor
    RocketMonitorScreen(Adafruit_ILI9341& tft, XPT2046_Touchscreen& ts);
    
    // Initialization
    void begin();
    void sleepDisplay();
    
    // Screen page enum
    enum ScreenPage {
        PAGE_MAIN = 0,
        PAGE_ALTITUDE_GRAPH = 1,
        PAGE_SPEED_GRAPH = 2,
        PAGE_POWER_GRAPH = 3,
        PAGE_ORIENTATION = 4,
        PAGE_LAST_POSITIONS = 5,
        PAGE_TRANSMITTER_SELECTION = 6,
        NUM_PAGES
    };
    
    // Constants
    static const int MAX_TRANSMITTERS = 10;
    static const int MAX_NAME_LENGTH = 16;
    static const int MAX_DATA_POINTS = 100;
    
    // Display layout constants for 240x320 display
    static const int HEADER_HEIGHT = 30;
    static const int BATTERY_ICON_WIDTH = 25;
    static const int BATTERY_ICON_HEIGHT = 12;
    static const int VALUE_HEIGHT = 26;
    static const int RIGHT_COL = 130;  // Adjusted for narrower display width
    static const uint16_t LABEL_COLOR = ILI9341_YELLOW;
    static const uint16_t VALUE_COLOR = ILI9341_WHITE;
    
    // Navigation button constants
    static const int NAV_BUTTON_WIDTH = 30;  // Smaller buttons for narrower display
    static const int NAV_BUTTON_HEIGHT = 30;
    static const int NAV_BUTTON_MARGIN = 5;
    static const uint16_t NAV_BUTTON_COLOR = ILI9341_BLUE;
    
    // Graph constants
    static const int GRAPH_X = 30;  // Adjusted for narrower display
    static const int GRAPH_Y = 60;  // More vertical space available
    static const int GRAPH_WIDTH = 180;  // Narrower graph
    static const int GRAPH_HEIGHT = 180;  // Taller graph for better use of portrait orientation
    static const uint16_t GRAPH_COLOR = ILI9341_GREEN;
    static const uint16_t AXIS_COLOR = ILI9341_WHITE;
    
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
    void drawOrientationPage();
    void updateOrientationData();
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
    ScreenPage getCurrentPage() const { return currentPage; }
    void setCurrentPage(ScreenPage page) { currentPage = page; }
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
    float currentSpeed = 0.0f;
    float baroVelocity = 0.0f;
    float temperature = 0.0f;
    float maxG = 0.0f;
    float batteryVoltage = 0.0f;
    uint8_t batteryPercent = 0;
    int8_t txPower = 0;
    uint32_t rocketUptime = 0; // Rocket uptime in milliseconds
    uint8_t launchState = 0;
    float currentLat = 0.0f;
    float currentLng = 0.0f;
    
    // Frequency information
    float rocketFrequency[MAX_TRANSMITTERS] = {0.0f}; // Operating frequency in MHz
    uint8_t radioType[MAX_TRANSMITTERS] = {0}; // 0=CC1101, 1=SX1278, 2=SX1262
    
    // Orientation data
    float orientationX = 0.0f;
    float orientationY = 0.0f;
    float orientationZ = 0.0f;
    uint8_t tiltAngle = 0;
    
    // Position storage and calculations
    void saveLastKnownPosition(uint32_t transmitterId, float lat, float lng);
    void clearLastKnownPosition(uint32_t transmitterId);
    bool getLastKnownPosition(uint32_t transmitterId, float &lat, float &lng);
    
    // Launch position storage
    void saveLaunchPosition(uint32_t transmitterId, float lat, float lng);
    void clearLaunchPosition(uint32_t transmitterId);
    bool getLaunchPosition(uint32_t transmitterId, float &lat, float &lng);
    
    // Distance and bearing calculations
    float calculateDistance(float lat1, float lon1, float lat2, float lon2);
    float calculateBearing(float lat1, float lon1, float lat2, float lon2);
    String getBearingDirection(float bearing);
    
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
    
    // Rescan and connection status
    bool rescanRequested = false;
    bool isRescanRequested() const { return rescanRequested; }
    void clearRescanRequest() { rescanRequested = false; }
    
    // Check if we need to auto-rescan due to lost connection
    bool checkConnectionTimeout() {
        // If no data received for 10 seconds, trigger auto-rescan
        if (lastPacketTime > 0 && millis() - lastPacketTime > 10000) {
            return true;
        }
        return false;
    }
    
    // Data update methods for different packet types
    void updateGpsData(float lat, float lng);
    void updateAltitudeData(float altitude, float maxAlt, float temp, float maxG, float accelVelocity, float baroVelocity, float orientX, float orientY, float orientZ);
    void updateSystemData(float battV, uint8_t battPct, int8_t txPwr, uint32_t uptime, uint8_t tiltAngle);
    void updateGraphData(float altitude, float speed, int8_t txPower);
    void addTransmitter(uint32_t transmitterId);
    void setTransmitterFrequency(uint32_t transmitterId, float frequency, uint8_t radioType);
    float getTransmitterFrequency(uint32_t transmitterId) const;
    uint8_t getTransmitterRadioType(uint32_t transmitterId) const;
    bool isFrequencyAcknowledged(uint32_t transmitterId) const;
    void setFrequencyAcknowledged(uint32_t transmitterId, bool acknowledged);
    void updateDisplay();
    

    
private:
    // References to external objects
    Adafruit_ILI9341& tft;
    XPT2046_Touchscreen& ts;
    Preferences rocketNamesStorage; // For storing rocket names
    Preferences lastPositionsStorage; // For storing last known positions
    
    // Display page control
    ScreenPage currentPage = PAGE_TRANSMITTER_SELECTION;
    bool rocketSelected = false; // Flag to indicate if a rocket has been selected
    
    // Transmitter selection
    uint32_t knownTransmitters[MAX_TRANSMITTERS] = {0};
    String rocketNames[MAX_TRANSMITTERS];
    int numTransmitters = 0;
    int selectedTransmitterIndex = -1; // -1 means no transmitter selected
    uint32_t selectedTransmitterId = 0;
    bool frequencyAcknowledged[MAX_TRANSMITTERS] = {false}; // Whether we've acknowledged the frequency for each transmitter
    
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
    
    // Graph configuration
    const char* graphTitles[3] = {"Altitude vs Time", "Speed vs Time", "TX Power vs Time"};
    const char* graphYLabels[3] = {"Altitude (m)", "Speed (m/s)", "Power (dBm)"};
    const uint16_t graphColors[3] = {ILI9341_GREEN, ILI9341_CYAN, ILI9341_YELLOW};
};

#endif // ROCKET_MONITOR_SCREEN_H
