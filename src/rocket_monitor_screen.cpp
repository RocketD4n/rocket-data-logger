#include "rocket_monitor_screen.h"
#include "rocket_telemetry_protocol.h"

// Constructor
RocketMonitorScreen::RocketMonitorScreen(Adafruit_ILI9341& tft, XPT2046_Touchscreen& ts)
    : tft(tft), ts(ts), currentPage(PAGE_TRANSMITTER_SELECTION), rocketSelected(false), 
      numTransmitters(0), selectedTransmitterIndex(-1), selectedTransmitterId(0),
      historyIndex(0), historyCount(0), millisAtFirstPacket(0),
      showKeyboard(false), editingName(false), editingTransmitterIndex(-1)
{
    // Initialize arrays
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
        knownTransmitters[i] = 0;
        rocketNames[i] = "";
        rocketFrequency[i] = 0.0f;
        radioType[i] = 0;
        frequencyAcknowledged[i] = false;
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < MAX_DATA_POINTS; j++) {
            dataHistory[i][j] = 0.0f;
        }
    }
    
    for (int i = 0; i < MAX_DATA_POINTS; i++) {
        timeHistory[i] = 0;
    }
}

// Initialization
void RocketMonitorScreen::begin() {
    Serial.println("Display init: Starting display initialization");
    
    // Initialize display
    tft.begin();
    Serial.println("Display init: Display initialized");
    
    // Increase SPI speed for faster drawing
    SPI.setFrequency(40000000); // Set to 40MHz for better performance
    
    // Set rotation (landscape)
    tft.setRotation(1);
    Serial.println("Display init: Rotation set");
    
    // Fill screen with black
    tft.fillScreen(ILI9341_BLACK);
    Serial.println("Display init: Screen cleared");
    
    // Initialize touch screen
    ts.begin();
    Serial.println("Display init: Touch screen initialized");
    
    // Load saved rocket names
    loadRocketNames();
    Serial.println("Display init: Rocket names loaded");
    
    // Draw initial screen
    drawTransmitterSelectionPage();
    Serial.println("Display init: Initial screen drawn");
}

// Put display to sleep
void RocketMonitorScreen::sleepDisplay() {
    // Not directly supported in Adafruit_ILI9341, but we can clear the screen
    tft.fillScreen(ILI9341_BLACK);
}

// Load rocket names from storage
void RocketMonitorScreen::loadRocketNames() {
    rocketNamesStorage.begin("rockets", false);
    
    // Get number of transmitters
    numTransmitters = rocketNamesStorage.getUInt("numTx", 0);
    if (numTransmitters > MAX_TRANSMITTERS) {
        numTransmitters = MAX_TRANSMITTERS;
    }
    
    // Load each transmitter ID and name
    for (int i = 0; i < numTransmitters; i++) {
        String idKey = "txId" + String(i);
        String nameKey = "txName" + String(i);
        String freqKey = "txFreq" + String(i);
        String typeKey = "txType" + String(i);
        
        knownTransmitters[i] = rocketNamesStorage.getUInt(idKey.c_str(), 0);
        rocketNames[i] = rocketNamesStorage.getString(nameKey.c_str(), String(knownTransmitters[i], HEX));
        rocketFrequency[i] = rocketNamesStorage.getFloat(freqKey.c_str(), 0.0f);
        radioType[i] = rocketNamesStorage.getUChar(typeKey.c_str(), 0);
    }
    
    rocketNamesStorage.end();
}

// Save rocket name to storage
void RocketMonitorScreen::saveRocketName() {
    if (editingTransmitterIndex < 0 || editingTransmitterIndex >= numTransmitters) {
        return; // Invalid index
    }
    
    uint32_t transmitterId = knownTransmitters[editingTransmitterIndex];
    String name = currentInput;
    
    // Open storage
    if (!rocketNamesStorage.begin("rocketNames", false)) {
        Serial.println("Failed to open rocket names storage");
        return;
    }
    
    // Create a key from the transmitter ID
    char key[16];
    sprintf(key, "%08X", transmitterId);
    
    // Save the name
    rocketNamesStorage.putString(key, name);
    
    // Update the name in memory
    rocketNames[editingTransmitterIndex] = name;
    
    // Close storage
    rocketNamesStorage.end();
    
    Serial.print("Saved rocket name: ");
    Serial.print(name);
    Serial.print(" for transmitter ID: 0x");
    Serial.println(transmitterId, HEX);
}

// Check if touch is within button area
bool RocketMonitorScreen::isTouchInButton(uint16_t x, uint16_t y, uint16_t btnX, uint16_t btnY, uint16_t btnW, uint16_t btnH) {
    return (x >= btnX && x < btnX + btnW && y >= btnY && y < btnY + btnH);
}

// Add a new transmitter to the list
void RocketMonitorScreen::addTransmitter(uint32_t transmitterId) {
    // Check if transmitter is already known
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return; // Already in the list
        }
    }
    
    // Add new transmitter if there's room
    if (numTransmitters < MAX_TRANSMITTERS) {
        knownTransmitters[numTransmitters] = transmitterId;
        rocketNames[numTransmitters] = String(transmitterId, HEX); // Default name is ID in hex
        rocketFrequency[numTransmitters] = 0.0f;
        radioType[numTransmitters] = 0;
        frequencyAcknowledged[numTransmitters] = false;
        
        // Save to preferences
        rocketNamesStorage.begin("rockets", false);
        rocketNamesStorage.putUInt("numTx", numTransmitters + 1);
        
        String idKey = "txId" + String(numTransmitters);
        String nameKey = "txName" + String(numTransmitters);
        
        rocketNamesStorage.putUInt(idKey.c_str(), transmitterId);
        rocketNamesStorage.putString(nameKey.c_str(), rocketNames[numTransmitters]);
        rocketNamesStorage.end();
        
        numTransmitters++;
        
        // If this is the first transmitter, select it automatically
        if (numTransmitters == 1) {
            selectedTransmitterIndex = 0;
            selectedTransmitterId = transmitterId;
            rocketSelected = true;
        }
    }
}

// Set transmitter frequency
void RocketMonitorScreen::setTransmitterFrequency(uint32_t transmitterId, float frequency, uint8_t radioTypeVal) {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            rocketFrequency[i] = frequency;
            radioType[i] = radioTypeVal;
            return;
        }
    }
}

// Get transmitter frequency
float RocketMonitorScreen::getTransmitterFrequency(uint32_t transmitterId) const {
    // Find the transmitter in our list
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return rocketFrequency[i];
        }
    }
    return 0.0f; // Not found
}

// Get transmitter radio type
uint8_t RocketMonitorScreen::getTransmitterRadioType(uint32_t transmitterId) const {
    // Find the transmitter in our list
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return radioType[i];
        }
    }
    return 0; // Not found
}

// Check if frequency is acknowledged
bool RocketMonitorScreen::isFrequencyAcknowledged(uint32_t transmitterId) const {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return frequencyAcknowledged[i];
        }
    }
    return false;
}

// Set frequency acknowledged flag
void RocketMonitorScreen::setFrequencyAcknowledged(uint32_t transmitterId, bool acknowledged) {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            frequencyAcknowledged[i] = acknowledged;
            return;
        }
    }
}

void RocketMonitorScreen::updateGpsData(float lat, float lng) {
    currentLat = lat;
    currentLng = lng;
    
    // If we have a selected rocket, save its position
    if (rocketSelected && selectedTransmitterId != 0) {
        saveLastKnownPosition(selectedTransmitterId, lat, lng);
    }
    
    // Update display if on main page
    if (currentPage == PAGE_MAIN) {
        updateMainPageValues();
    }
}

// Update altitude data
void RocketMonitorScreen::updateAltitudeData(float altitude, float maxAlt, float temp, float maxG, 
                                         float accelVelocity, float baroVel, float orientX, float orientY, float orientZ) {
    currentAltitude = altitude;
    maxAltitude = maxAlt;
    temperature = temp;
    this->maxG = maxG;
    currentSpeed = accelVelocity;
    baroVelocity = baroVel;
    orientationX = orientX;
    orientationY = orientY;
    orientationZ = orientZ;
    
    // Update display if on main or orientation page
    if (currentPage == PAGE_MAIN) {
        updateMainPageValues();
    } else if (currentPage == PAGE_ORIENTATION) {
        updateOrientationData();
    }
}

// Update system data
void RocketMonitorScreen::updateSystemData(float battV, uint8_t battPct, int8_t txPwr, uint32_t uptime, uint8_t tiltAng) {
    batteryVoltage = battV;
    batteryPercent = battPct;
    txPower = txPwr;
    rocketUptime = uptime;
    tiltAngle = tiltAng;
    
    // Update battery indicator
    drawBatteryIndicator(batteryPercent);
    
    // Update display if on main page
    if (currentPage == PAGE_MAIN) {
        updateMainPageValues();
    }
}

// Update graph data
void RocketMonitorScreen::updateGraphData(float altitude, float speed, int8_t txPower) {
    // Initialize time tracking if this is the first packet
    if (historyCount == 0) {
        millisAtFirstPacket = millis();
    }
    
    // Store data in circular buffer
    dataHistory[0][historyIndex] = altitude;
    dataHistory[1][historyIndex] = speed;
    dataHistory[2][historyIndex] = txPower;
    timeHistory[historyIndex] = millis() - millisAtFirstPacket;
    
    // Update indices
    historyIndex = (historyIndex + 1) % MAX_DATA_POINTS;
    if (historyCount < MAX_DATA_POINTS) {
        historyCount++;
    }
    
    // Update graph if on a graph page
    if (currentPage == PAGE_ALTITUDE_GRAPH || currentPage == PAGE_SPEED_GRAPH || currentPage == PAGE_POWER_GRAPH) {
        updateGraph();
    }
}

// Update just the values on the main page
void RocketMonitorScreen::updateMainPageValues() {
    // Only update if we're on the main page
    if (currentPage != PAGE_MAIN) {
        return;
    }
    
    tft.setTextSize(1);
    tft.setTextColor(VALUE_COLOR, ILI9341_BLACK);
    
    // Left column values
    tft.setCursor(5, HEADER_HEIGHT + 5 + 10);
    tft.print(currentAltitude, 1);
    tft.print(" m      ");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT + 10);
    tft.print(maxAltitude, 1);
    tft.print(" m      ");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 2 + 10);
    tft.print(currentSpeed, 1);
    tft.print(" m/s      ");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 3 + 10);
    tft.print(temperature, 1);
    tft.print(" C      ");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 4 + 10);
    tft.print(maxG, 1);
    tft.print(" g      ");
    
    // Right column values
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + 10);
    tft.print(currentLat, 6);
    tft.print("      ");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT + 10);
    tft.print(currentLng, 6);
    tft.print("      ");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 2 + 10);
    tft.print(batteryVoltage, 2);
    tft.print("V (");
    tft.print(batteryPercent);
    tft.print("%)      ");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 3 + 10);
    tft.print(txPower);
    tft.print(" dBm      ");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 4 + 10);
    // Format uptime as HH:MM:SS
    uint32_t seconds = rocketUptime / 1000;
    uint8_t hours = seconds / 3600;
    uint8_t minutes = (seconds % 3600) / 60;
    uint8_t secs = seconds % 60;
    
    char timeStr[10];
    sprintf(timeStr, "%02d:%02d:%02d", hours, minutes, secs);
    tft.print(timeStr);
    tft.print("      ");
}

// Draw graph page
void RocketMonitorScreen::drawGraphPage() {
    // Clear screen
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw header
    tft.fillRect(0, 0, tft.width(), HEADER_HEIGHT, ILI9341_NAVY);
    
    // Draw title
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(5, 10);
    
    if (rocketSelected && selectedTransmitterIndex >= 0) {
        tft.print(rocketNames[selectedTransmitterIndex]);
        tft.print(" - ");
    }
    
    // Add graph type to title
    switch (currentPage) {
        case PAGE_ALTITUDE_GRAPH:
            tft.print("Altitude");
            break;
        case PAGE_SPEED_GRAPH:
            tft.print("Speed");
            break;
        case PAGE_POWER_GRAPH:
            tft.print("TX Power");
            break;
        default:
            break;
    }
    
    // Draw battery indicator
    drawBatteryIndicator(batteryPercent);
    
    // Draw graph axes
    int graphX = 30;
    int graphY = HEADER_HEIGHT + 10;
    int graphWidth = tft.width() - graphX - 10;
    int graphHeight = tft.height() - graphY - 40;
    
    // Draw axes
    tft.drawLine(graphX, graphY, graphX, graphY + graphHeight, ILI9341_WHITE);
    tft.drawLine(graphX, graphY + graphHeight, graphX + graphWidth, graphY + graphHeight, ILI9341_WHITE);
    
    // Draw axis labels
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    
    // Y-axis label
    tft.setCursor(5, graphY + graphHeight / 2);
    switch (currentPage) {
        case PAGE_ALTITUDE_GRAPH:
            tft.print("m");
            break;
        case PAGE_SPEED_GRAPH:
            tft.print("m/s");
            break;
        case PAGE_POWER_GRAPH:
            tft.print("dBm");
            break;
        default:
            break;
    }
    
    // X-axis label (time)
    tft.setCursor(graphX + graphWidth / 2, graphY + graphHeight + 10);
    tft.print("Time (s)");
    
    // Draw navigation buttons
    drawNavigationButtons(rocketSelected);
    
    // Draw the graph data
    updateGraph();
}

// Update graph data points
void RocketMonitorScreen::updateGraph() {
    // Only update if we're on a graph page
    if (currentPage != PAGE_ALTITUDE_GRAPH && currentPage != PAGE_SPEED_GRAPH && currentPage != PAGE_POWER_GRAPH) {
        return;
    }
    
    // Define graph area
    int graphX = 30;
    int graphY = HEADER_HEIGHT + 10;
    int graphWidth = tft.width() - graphX - 10;
    int graphHeight = tft.height() - graphY - 40;
    
    // Clear graph area (but not axes)
    tft.fillRect(graphX + 1, graphY, graphWidth - 1, graphHeight, ILI9341_BLACK);
    
    // Select which data series to display based on current page
    int dataIndex;
    float maxValue = 0;
    float minValue = 0;
    
    switch (currentPage) {
        case PAGE_ALTITUDE_GRAPH:
            dataIndex = 0; // Altitude
            maxValue = 1000; // Default max altitude in meters
            minValue = 0;
            break;
        case PAGE_SPEED_GRAPH:
            dataIndex = 1; // Speed
            maxValue = 300; // Default max speed in m/s
            minValue = 0;
            break;
        case PAGE_POWER_GRAPH:
            dataIndex = 2; // TX Power
            maxValue = 20; // Default max power in dBm
            minValue = 0;
            break;
        default:
            return;
    }
    
    // Find actual min/max values in the data
    if (historyCount > 0) {
        maxValue = dataHistory[dataIndex][0];
        minValue = dataHistory[dataIndex][0];
        
        for (int i = 1; i < historyCount; i++) {
            if (dataHistory[dataIndex][i] > maxValue) {
                maxValue = dataHistory[dataIndex][i];
            }
            if (dataHistory[dataIndex][i] < minValue) {
                minValue = dataHistory[dataIndex][i];
            }
        }
        
        // Add some margin to the max/min
        float range = maxValue - minValue;
        if (range < 0.1) range = 0.1; // Avoid division by zero
        
        maxValue += range * 0.1;
        minValue -= range * 0.1;
        
        // Ensure min is at least 0 for altitude and speed
        if ((currentPage == PAGE_ALTITUDE_GRAPH || currentPage == PAGE_SPEED_GRAPH) && minValue < 0) {
            minValue = 0;
        }
    }
    
    // Draw Y-axis scale
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    
    // Max value
    tft.setCursor(5, graphY);
    tft.print((int)maxValue);
    
    // Mid value
    tft.setCursor(5, graphY + graphHeight / 2);
    tft.print((int)((maxValue + minValue) / 2));
    
    // Min value
    tft.setCursor(5, graphY + graphHeight - 8);
    tft.print((int)minValue);
    
    // Draw X-axis scale (time)
    uint32_t maxTime = 60000; // Default 60 seconds
    
    if (historyCount > 0) {
        maxTime = timeHistory[(historyIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS];
        if (maxTime < 60000) maxTime = 60000; // Minimum 60 seconds
    }
    
    // Start time
    tft.setCursor(graphX, graphY + graphHeight + 10);
    tft.print("0");
    
    // Mid time
    tft.setCursor(graphX + graphWidth / 2, graphY + graphHeight + 10);
    tft.print(maxTime / 2000);
    
    // End time
    tft.setCursor(graphX + graphWidth - 15, graphY + graphHeight + 10);
    tft.print(maxTime / 1000);
    
    // Draw data points if we have any
    if (historyCount > 1) {
        uint16_t color;
        switch (currentPage) {
            case PAGE_ALTITUDE_GRAPH:
                color = ILI9341_CYAN;
                break;
            case PAGE_SPEED_GRAPH:
                color = ILI9341_GREEN;
                break;
            case PAGE_POWER_GRAPH:
                color = ILI9341_YELLOW;
                break;
            default:
                color = ILI9341_WHITE;
                break;
        }
        
        // Draw lines connecting data points
        for (int i = 1; i < historyCount; i++) {
            int idx1 = (historyIndex - i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
            int idx2 = (historyIndex - i + 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS;
            
            // Calculate x coordinates (time)
            int x1 = graphX + (timeHistory[idx1] * graphWidth) / maxTime;
            int x2 = graphX + (timeHistory[idx2] * graphWidth) / maxTime;
            
            // Calculate y coordinates (data value)
            float valueRange = maxValue - minValue;
            if (valueRange < 0.1) valueRange = 0.1; // Avoid division by zero
            
            int y1 = graphY + graphHeight - ((dataHistory[dataIndex][idx1] - minValue) * graphHeight) / valueRange;
            int y2 = graphY + graphHeight - ((dataHistory[dataIndex][idx2] - minValue) * graphHeight) / valueRange;
            
            // Ensure points are within graph area
            x1 = constrain(x1, graphX, graphX + graphWidth);
            x2 = constrain(x2, graphX, graphX + graphWidth);
            y1 = constrain(y1, graphY, graphY + graphHeight);
            y2 = constrain(y2, graphY, graphY + graphHeight);
            
            // Draw line between points
            tft.drawLine(x1, y1, x2, y2, color);
        }
    }
}

// Draw transmitter selection page
void RocketMonitorScreen::drawTransmitterSelectionPage() {
    // Set drawing area to full screen for faster drawing
    tft.setAddrWindow(0, 0, tft.width(), tft.height());
    
    // Clear screen with faster method
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw header - use a single operation for better performance
    tft.fillRect(0, 0, tft.width(), HEADER_HEIGHT, ILI9341_NAVY);
    
    // Draw title
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(5, 10);
    tft.print("Select Rocket");
    
    // Only draw battery indicator if we have valid battery data
    if (batteryPercent > 0) {
        drawBatteryIndicator(batteryPercent);
    }
    
    // Draw list of transmitters
    int itemHeight = 30;
    int startY = HEADER_HEIGHT + 5;
    
    // Calculate available height for items
    int availableHeight = tft.height() - HEADER_HEIGHT - NAV_BUTTON_HEIGHT - NAV_BUTTON_MARGIN - 10;
    int maxItems = availableHeight / (itemHeight + 5);
    
    // Draw "Add New" button at the top if we have room
    if (numTransmitters < MAX_TRANSMITTERS) {
        // Use a single drawing operation for the button
        tft.fillRoundRect(20, startY, tft.width() - 40, itemHeight, 5, ILI9341_DARKGREEN);
        tft.drawRoundRect(20, startY, tft.width() - 40, itemHeight, 5, ILI9341_WHITE);
        
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(1);
        tft.setCursor(tft.width() / 2 - 30, startY + itemHeight / 2 - 4);
        tft.print("+ Add New Rocket");
        
        startY += itemHeight + 5;
        maxItems--; // Reduce available items by one
    }
    
    // Draw list of known transmitters
    int itemsToDraw = min(numTransmitters, maxItems);
    for (int i = 0; i < itemsToDraw; i++) {
        // Draw item background with a single operation
        uint16_t bgColor = (selectedTransmitterIndex == i) ? ILI9341_BLUE : ILI9341_DARKGREY;
        tft.fillRoundRect(20, startY, tft.width() - 40, itemHeight, 5, bgColor);
        tft.drawRoundRect(20, startY, tft.width() - 40, itemHeight, 5, ILI9341_WHITE);
        
        // Draw rocket name
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(1);
        tft.setCursor(30, startY + 8);
        tft.print(rocketNames[i]);
        
        // Draw transmitter ID in smaller text
        tft.setCursor(30, startY + 20);
        tft.print("ID: 0x");
        tft.print(knownTransmitters[i], HEX);
        
        // Draw edit button with a single operation
        tft.fillRoundRect(tft.width() - 60, startY + 5, 30, 20, 3, ILI9341_OLIVE);
        tft.drawRoundRect(tft.width() - 60, startY + 5, 30, 20, 3, ILI9341_WHITE);
        tft.setCursor(tft.width() - 55, startY + 10);
        tft.print("Edit");
        
        startY += itemHeight + 5;
    }
    
    // Draw keyboard if active
    if (showKeyboard) {
        drawKeyboard();
    }
    
    // Draw navigation buttons
    drawNavigationButtons(rocketSelected);
}

// Draw on-screen keyboard
void RocketMonitorScreen::drawKeyboard() {
    // Keyboard overlay
    int keyboardY = 80;
    int keyboardHeight = 140;
    
    // Draw keyboard background
    tft.fillRect(0, keyboardY, tft.width(), keyboardHeight, ILI9341_NAVY);
    tft.drawRect(0, keyboardY, tft.width(), keyboardHeight, ILI9341_WHITE);
    
    // Draw text input field
    tft.fillRect(10, keyboardY + 5, tft.width() - 20, 20, ILI9341_BLACK);
    tft.drawRect(10, keyboardY + 5, tft.width() - 20, 20, ILI9341_WHITE);
    
    // Draw current text
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setCursor(15, keyboardY + 10);
    tft.print(currentInput);
    
    // Draw keyboard keys
    const char* keys[] = {
        "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P",
        "A", "S", "D", "F", "G", "H", "J", "K", "L", "_",
        "Z", "X", "C", "V", "B", "N", "M", ".", "-", "<"
    };
    
    int keyWidth = tft.width() / 10;
    int keyHeight = 20;
    int startKeyY = keyboardY + 30;
    
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 10; col++) {
            int keyIndex = row * 10 + col;
            int keyX = col * keyWidth;
            int keyY = startKeyY + row * (keyHeight + 5);
            
            // Draw key background
            tft.fillRect(keyX + 2, keyY, keyWidth - 4, keyHeight, ILI9341_DARKGREY);
            tft.drawRect(keyX + 2, keyY, keyWidth - 4, keyHeight, ILI9341_WHITE);
            
            // Draw key label
            tft.setTextColor(ILI9341_WHITE);
            tft.setTextSize(1);
            tft.setCursor(keyX + keyWidth / 2 - 3, keyY + keyHeight / 2 - 3);
            tft.print(keys[keyIndex]);
        }
    }
    
    // Draw space bar
    int spaceY = startKeyY + 4 * (keyHeight + 5);
    tft.fillRect(keyWidth, spaceY, tft.width() - 2 * keyWidth, keyHeight, ILI9341_DARKGREY);
    tft.drawRect(keyWidth, spaceY, tft.width() - 2 * keyWidth, keyHeight, ILI9341_WHITE);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setCursor(tft.width() / 2 - 15, spaceY + keyHeight / 2 - 3);
    tft.print("SPACE");
    
    // Draw OK button
    tft.fillRect(tft.width() - keyWidth, spaceY, keyWidth, keyHeight, ILI9341_GREEN);
    tft.drawRect(tft.width() - keyWidth, spaceY, keyWidth, keyHeight, ILI9341_WHITE);
    tft.setCursor(tft.width() - keyWidth + 5, spaceY + keyHeight / 2 - 3);
    tft.print("OK");
    
    // Draw Cancel button
    tft.fillRect(0, spaceY, keyWidth, keyHeight, ILI9341_RED);
    tft.drawRect(0, spaceY, keyWidth, keyHeight, ILI9341_WHITE);
    tft.setCursor(5, spaceY + keyHeight / 2 - 3);
    tft.print("CANCEL");
}

// Handle touch events
bool RocketMonitorScreen::handleTouch(int x, int y) {
    // Check if touch is in navigation buttons
    if (y < HEADER_HEIGHT) {
        // Touch in header area
        return false;
    }
    
    // Handle keyboard if it's showing
    if (showKeyboard) {
        processKeyPress(x, y);
        return true;
    }
    
    // Handle based on current page
    switch (currentPage) {
        case PAGE_TRANSMITTER_SELECTION: {
            // Check if touch is on a transmitter item
            if (numTransmitters > 0) {
                int itemHeight = 30;
                int startY = HEADER_HEIGHT + 5;
                
                for (int i = 0; i < numTransmitters; i++) {
                    if (y >= startY + i * itemHeight && y < startY + (i + 1) * itemHeight) {
                        selectedTransmitterIndex = i;
                        selectedTransmitterId = knownTransmitters[i];
                        rocketSelected = true;
                        currentPage = PAGE_MAIN; // Go to main page after selection
                        updateDisplay();
                        return true;
                    }
                }
                
                // Check if touch is on "Add New" button
                int addBtnY = startY + numTransmitters * itemHeight;
                if (y >= addBtnY && y < addBtnY + itemHeight) {
                    // Show keyboard for entering new name
                    editingName = false;
                    showKeyboard = true;
                    currentInput = "";
                    drawKeyboard();
                    return true;
                }
            }
            break;
        }
            
        case PAGE_MAIN: {
            // Check if touch is in navigation buttons
            if (y > tft.height() - NAV_BUTTON_HEIGHT - 5) {
                // Left button
                if (x < NAV_BUTTON_WIDTH + NAV_BUTTON_MARGIN) {
                    currentPage = PAGE_TRANSMITTER_SELECTION;
                    updateDisplay();
                    return true;
                }
                // Right button
                else if (x > tft.width() - NAV_BUTTON_WIDTH - NAV_BUTTON_MARGIN) {
                    currentPage = PAGE_ALTITUDE_GRAPH;
                    updateDisplay();
                    return true;
                }
            }
            break;
        }
            
        case PAGE_ALTITUDE_GRAPH:
        case PAGE_SPEED_GRAPH:
        case PAGE_POWER_GRAPH: {
            // Check if touch is in navigation buttons
            if (y > tft.height() - NAV_BUTTON_HEIGHT - 5) {
                // Left button
                if (x < NAV_BUTTON_WIDTH + NAV_BUTTON_MARGIN) {
                    currentPage = (ScreenPage)(currentPage - 1);
                    updateDisplay();
                    return true;
                }
                // Right button
                else if (x > tft.width() - NAV_BUTTON_WIDTH - NAV_BUTTON_MARGIN) {
                    currentPage = (ScreenPage)(currentPage + 1);
                    if (currentPage >= NUM_PAGES) {
                        currentPage = PAGE_MAIN;
                    }
                    updateDisplay();
                    return true;
                }
            }
            break;
        }
            
        default: {
            // For other pages, just handle navigation
            if (y > tft.height() - NAV_BUTTON_HEIGHT - 5) {
                // Left button
                if (x < NAV_BUTTON_WIDTH + NAV_BUTTON_MARGIN) {
                    currentPage = (ScreenPage)(currentPage - 1);
                    updateDisplay();
                    return true;
                }
                // Right button
                else if (x > tft.width() - NAV_BUTTON_WIDTH - NAV_BUTTON_MARGIN) {
                    currentPage = (ScreenPage)(currentPage + 1);
                    if (currentPage >= NUM_PAGES) {
                        currentPage = PAGE_MAIN;
                    }
                    updateDisplay();
                    return true;
                }
            }
            break;
        }
    }
    
    return false;
}

// Process keyboard key press
void RocketMonitorScreen::processKeyPress(int x, int y) {
    // Keyboard layout constants
    int keyboardY = 80;
    int keyWidth = tft.width() / 10;
    int keyHeight = 20;
    int startKeyY = keyboardY + 30;
    
    // Check if touch is on a key
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 10; col++) {
            int keyX = col * keyWidth;
            int keyY = startKeyY + row * (keyHeight + 5);
            
            if (isTouchInButton(x, y, keyX + 2, keyY, keyWidth - 4, keyHeight)) {
                // Key pressed
                const char* keys[] = {
                    "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
                    "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P",
                    "A", "S", "D", "F", "G", "H", "J", "K", "L", "_",
                    "Z", "X", "C", "V", "B", "N", "M", ".", "-", "<"
                };
                
                int keyIndex = row * 10 + col;
                
                if (keys[keyIndex][0] == '<') {
                    // Backspace
                    if (currentInput.length() > 0) {
                        currentInput = currentInput.substring(0, currentInput.length() - 1);
                    }
                } else {
                    // Add character if we have room
                    if (currentInput.length() < MAX_NAME_LENGTH) {
                        currentInput += keys[keyIndex];
                    }
                }
                
                drawKeyboard();
                return;
            }
        }
    }
    
    // Check space bar
    int spaceY = startKeyY + 4 * (keyHeight + 5);
    if (isTouchInButton(x, y, keyWidth, spaceY, tft.width() - 2 * keyWidth, keyHeight)) {
        // Space bar pressed
        if (currentInput.length() < MAX_NAME_LENGTH) {
            currentInput += " ";
        }
        drawKeyboard();
        return;
    }
    
    // Check OK button
    if (isTouchInButton(x, y, tft.width() - keyWidth, spaceY, keyWidth, keyHeight)) {
        // OK button pressed
        if (editingName && editingTransmitterIndex >= 0) {
            // Save the edited name
            saveRocketName();
        }
        
        // Close keyboard
        showKeyboard = false;
        editingName = false;
        editingTransmitterIndex = -1;
        drawTransmitterSelectionPage();
        return;
    }
    
    // Check Cancel button
    if (isTouchInButton(x, y, 0, spaceY, keyWidth, keyHeight)) {
        // Cancel button pressed
        showKeyboard = false;
        editingName = false;
        editingTransmitterIndex = -1;
        drawTransmitterSelectionPage();
        return;
    }
}

// Update display based on current page
void RocketMonitorScreen::updateDisplay() {
    switch (currentPage) {
        case PAGE_MAIN:
            drawMainPage();
            break;
        case PAGE_ALTITUDE_GRAPH:
        case PAGE_SPEED_GRAPH:
        case PAGE_POWER_GRAPH:
            drawGraphPage();
            break;
        case PAGE_ORIENTATION:
            drawOrientationPage();
            break;
        case PAGE_LAST_POSITIONS:
            drawLastPositionsPage();
            break;
        case PAGE_TRANSMITTER_SELECTION:
            drawTransmitterSelectionPage();
            break;
        default:
            break;
    }
}
