#include "rocket_monitor_screen.h"
#include "rocket_telemetry_protocol.h"

// Constructor
RocketMonitorScreen::RocketMonitorScreen(TFT_eSPI& tft, XPT2046_Touchscreen& ts)
    : tft(tft), ts(ts), currentPage(PAGE_TRANSMITTER_SELECTION), rocketSelected(false), 
      numTransmitters(0), selectedTransmitterIndex(-1), selectedTransmitterId(0),
      historyIndex(0), historyCount(0), millisAtFirstPacket(0),
      showKeyboard(false), editingName(false), editingTransmitterIndex(-1)
{
    // Initialize arrays
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
        knownTransmitters[i] = 0;
        rocketNames[i] = "";
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
    // Initialize display
    tft.init();
    tft.setRotation(3); // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
       
    // Initialize touch screen
    ts.begin();
    ts.setRotation(3);
    
    // Initialize rocket names storage
    rocketNamesStorage.begin("rockets", false);
    rocketNamesStorage.end();
    
    // Initialize last positions storage
    lastPositionsStorage.begin("lastpos", false);
    lastPositionsStorage.end();
    
    // Load saved rocket names
    loadRocketNames();
    
    // Set initial page and draw it
    currentPage = PAGE_TRANSMITTER_SELECTION;
    drawTransmitterSelectionPage();
}

void RocketMonitorScreen::sleepDisplay() {
    tft.fillScreen(TFT_BLACK);
    tft.writecommand(0x28); // DISPOFF command
    tft.writecommand(0x10); // SLPIN command
}

// Load rocket names from storage
void RocketMonitorScreen::loadRocketNames() {
    rocketNamesStorage.begin("rockets", false); // false = read/write mode
    
    // Load saved rocket names
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
        String key = String(i);
        String name = rocketNamesStorage.getString(key.c_str(), "");
        
        // Check if we have a transmitter ID saved for this slot
        String idKey = "id" + key;
        uint32_t savedId = rocketNamesStorage.getUInt(idKey.c_str(), 0);
        
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
            //if (!found && numTransmitters < MAX_TRANSMITTERS) {
              //  knownTransmitters[numTransmitters] = savedId;
               // rocketNames[numTransmitters] = name;
              //  numTransmitters++;
            //}
        }
    }
    
    rocketNamesStorage.end();
}

// Save rocket name to storage
void RocketMonitorScreen::saveRocketName() {
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
    
    // Save to storage
    rocketNamesStorage.begin("rockets", false);
    String key = String(editingTransmitterIndex);
    rocketNamesStorage.putString(key.c_str(), currentInput);
    
    // Also save the transmitter ID
    String idKey = "id" + key;
    rocketNamesStorage.putUInt(idKey.c_str(), knownTransmitters[editingTransmitterIndex]);
    
    rocketNamesStorage.end();
    
    // Reset editing state
    editingName = false;
    showKeyboard = false;
    currentInput = "";
    editingTransmitterIndex = -1;
}

// Check if touch is within button area
bool RocketMonitorScreen::isTouchInButton(uint16_t x, uint16_t y, uint16_t btnX, uint16_t btnY, uint16_t btnW, uint16_t btnH) {
    return (x >= btnX && x <= btnX + btnW && y >= btnY && y <= btnY + btnH);
}

// Add a new transmitter to the list
void RocketMonitorScreen::addTransmitter(uint32_t transmitterId) {
    // Check if transmitter already exists
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return; // Already in the list
        }
    }
    
    // Add new transmitter if there's space
    if (numTransmitters < MAX_TRANSMITTERS) {
        knownTransmitters[numTransmitters] = transmitterId;
        rocketNames[numTransmitters] = String(transmitterId, HEX); // Default name is the ID in hex
        rocketFrequency[numTransmitters] = 0.0f; // Initialize frequency to 0
        radioType[numTransmitters] = 2; // Default to SX1262
        frequencyAcknowledged[numTransmitters] = false; // Initialize to not acknowledged
        numTransmitters++;
        
        // If this is the first transmitter, select it automatically
        if (numTransmitters == 1) {
            selectedTransmitterIndex = 0;
            selectedTransmitterId = transmitterId;
            rocketSelected = true;
        }
        
        // Redraw the transmitter selection page if we're on it
        if (currentPage == PAGE_TRANSMITTER_SELECTION) {
            drawTransmitterSelectionPage();
        }
    }
}

void RocketMonitorScreen::setTransmitterFrequency(uint32_t transmitterId, float frequency, uint8_t radioType) {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            rocketFrequency[i] = frequency;
            this->radioType[i] = radioType;
            return;
        }
    }
}

float RocketMonitorScreen::getTransmitterFrequency(uint32_t transmitterId) const {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return rocketFrequency[i];
        }
    }
    return 0.0f; // Default if not found
}

uint8_t RocketMonitorScreen::getTransmitterRadioType(uint32_t transmitterId) const {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return radioType[i];
        }
    }
    return 2; // Default to SX1262 if not found
}

bool RocketMonitorScreen::isFrequencyAcknowledged(uint32_t transmitterId) const {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            return frequencyAcknowledged[i];
        }
    }
    return false; // Default if not found
}

void RocketMonitorScreen::setFrequencyAcknowledged(uint32_t transmitterId, bool acknowledged) {
    for (int i = 0; i < numTransmitters; i++) {
        if (knownTransmitters[i] == transmitterId) {
            frequencyAcknowledged[i] = acknowledged;
            return;
        }
    }
}

// Update GPS data
void RocketMonitorScreen::updateGpsData(float lat, float lng) {
    currentLat = lat;
    currentLng = lng;
    
    // Save last known position if we have a valid GPS fix
    if (lat != 0.0f && lng != 0.0f && selectedTransmitterId != 0) {
        saveLastKnownPosition(selectedTransmitterId, lat, lng);
        
        // If the rocket is in pre-launch state, save this as the launch position
        if (launchState == LAUNCH_STATE_WAITING) {
            saveLaunchPosition(selectedTransmitterId, lat, lng);
        }
    }
    
    // Update last packet time
    lastPacketTime = millis();
    packetCount++;
}

// Update altitude data
void RocketMonitorScreen::updateAltitudeData(float altitude, float maxAlt, float temp, float maxG, float accelVelocity, float baroVelocity, float orientX, float orientY, float orientZ) {
    currentAltitude = altitude;
    maxAltitude = maxAlt;
    temperature = temp;
    this->maxG = maxG;
    
    // Use transmitted velocity values instead of calculating locally
    // We'll use the accelerometer-based velocity as our primary speed value
    currentSpeed = accelVelocity;
    
    // Store barometric velocity for potential display or comparison
    this->baroVelocity = baroVelocity;
    
    // Store orientation data
    orientationX = orientX;
    orientationY = orientY;
    orientationZ = orientZ;
    
    // Update last packet time
    lastPacketTime = millis();
    packetCount++;
    
    // Update graph data
    updateGraphData(altitude, currentSpeed, txPower);
}

// Update system data
void RocketMonitorScreen::updateSystemData(float battV, uint8_t battPct, int8_t txPwr, uint32_t uptime, uint8_t tiltAng) {
    batteryVoltage = battV;
    batteryPercent = battPct;
    txPower = txPwr;
    rocketUptime = uptime;
    tiltAngle = tiltAng;
    
    // Update last packet time
    lastPacketTime = millis();
    packetCount++;
}

// Update graph data
void RocketMonitorScreen::updateGraphData(float altitude, float speed, int8_t txPower) {
    // Record the time of the first packet for graphing time reference
    if (millisAtFirstPacket == 0) {
        millisAtFirstPacket = millis();
    }
    
    // Store data for graphs
    dataHistory[0][historyIndex] = altitude; // Altitude
    dataHistory[1][historyIndex] = speed;    // Speed
    dataHistory[2][historyIndex] = txPower;  // TX Power
    timeHistory[historyIndex] = millis() - millisAtFirstPacket;
    historyIndex = (historyIndex + 1) % MAX_DATA_POINTS;
    historyCount = min(historyCount + 1, MAX_DATA_POINTS);
}

// Draw battery icon and level at the top center of the screen
void RocketMonitorScreen::drawBatteryIndicator(float batteryPercent) {
    int centerX = tft.width() / 2;
    int batteryX = centerX - BATTERY_ICON_WIDTH / 2;
    int batteryY = 5;
    
    // Draw battery outline
    tft.drawRect(batteryX, batteryY, BATTERY_ICON_WIDTH, BATTERY_ICON_HEIGHT, TFT_WHITE);
    tft.drawRect(batteryX + BATTERY_ICON_WIDTH, batteryY + 2, 3, BATTERY_ICON_HEIGHT - 4, TFT_WHITE);
    
    // Draw battery fill based on percentage
    int fillWidth = (batteryPercent / 100.0) * (BATTERY_ICON_WIDTH - 2);
    
    // Choose color based on battery level
    uint16_t fillColor;
    if (batteryPercent > 75) {
        fillColor = TFT_GREEN;
    } else if (batteryPercent > 45) {
        fillColor = TFT_YELLOW;
    } else {
        fillColor = TFT_RED;
    }
    
    // Fill battery icon
    tft.fillRect(batteryX + 1, batteryY + 1, fillWidth, BATTERY_ICON_HEIGHT - 2, fillColor);
    
    // Draw percentage text
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(String((int)batteryPercent) + "%", centerX, batteryY + BATTERY_ICON_HEIGHT + 8, 2);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Draw navigation buttons
void RocketMonitorScreen::drawNavigationButtons(bool rocketSelected) {
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

// Draw the main data page
void RocketMonitorScreen::drawMainPage() {
    tft.fillScreen(TFT_BLACK);
    drawBatteryIndicator(batteryPercent);
    drawNavigationButtons(rocketSelected);
    
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
    
    // Draw buzzer button (if a rocket is selected and either pre-launch or landed)
    if (rocketSelected && (launchState == LAUNCH_STATE_WAITING || launchState == LAUNCH_STATE_LANDED)) {
        // Draw button background
        uint16_t buttonColor = buzzerActive ? TFT_RED : TFT_GREEN;
        tft.fillRoundRect(tft.width() - 100, HEADER_HEIGHT + VALUE_HEIGHT * 6, 
                         80, 30, 5, buttonColor);
        
        // Draw button text
        tft.setTextColor(TFT_WHITE, buttonColor);
        tft.setTextDatum(MC_DATUM); // Middle center
        tft.drawString(buzzerActive ? "BUZZER OFF" : "BUZZER ON", 
                      tft.width() - 60, HEADER_HEIGHT + VALUE_HEIGHT * 6 + 15, 2);
        tft.setTextDatum(TL_DATUM); // Back to top left
    }
    
    // Draw abort button (if a rocket is selected and in flight)
    if (rocketSelected && launchState == LAUNCH_STATE_LAUNCHED) {
        // Draw button background - Red for emergency
        uint16_t abortButtonColor = abortSent ? TFT_DARKGREY : TFT_RED;
        tft.fillRoundRect(20, HEADER_HEIGHT + VALUE_HEIGHT * 6, 
                         100, 40, 5, abortButtonColor);
        
        // Draw button text
        tft.setTextColor(TFT_WHITE, abortButtonColor);
        tft.setTextDatum(MC_DATUM); // Middle center
        tft.drawString(abortSent ? "ABORT SENT" : "EMERGENCY ABORT", 
                      70, HEADER_HEIGHT + VALUE_HEIGHT * 6 + 20, 2);
        tft.setTextDatum(TL_DATUM); // Back to top left
    }
    
    // Update values
    updateMainPageValues();
}

// Update just the values on the main page
void RocketMonitorScreen::updateMainPageValues() {
    if (currentPage != PAGE_MAIN) return;
    
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
        case LAUNCH_STATE_WAITING: launchStateStr = "Waiting..."; break;
        case LAUNCH_STATE_LAUNCHED: launchStateStr = "Launched"; break;
        case LAUNCH_STATE_LANDED: launchStateStr = "Landed"; break;
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

// Draw graph page
void RocketMonitorScreen::drawGraphPage() {
    int graphPage;
    if (currentPage == PAGE_ALTITUDE_GRAPH) graphPage = 0;
    else if (currentPage == PAGE_SPEED_GRAPH) graphPage = 1;
    else if (currentPage == PAGE_POWER_GRAPH) graphPage = 2;
    else return; // Not a graph page
    
    tft.fillScreen(TFT_BLACK);
    drawBatteryIndicator(batteryPercent);
    drawNavigationButtons(rocketSelected);
    
    // Draw graph title
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM); // Top center
    tft.drawString(graphTitles[graphPage], tft.width()/2, 35, 2);
    
    // Draw axes
    tft.drawLine(GRAPH_X, GRAPH_Y, GRAPH_X, GRAPH_Y + GRAPH_HEIGHT, AXIS_COLOR); // Y axis
    tft.drawLine(GRAPH_X, GRAPH_Y + GRAPH_HEIGHT, GRAPH_X + GRAPH_WIDTH, GRAPH_Y + GRAPH_HEIGHT, AXIS_COLOR); // X axis
    
    // Draw Y axis label
    tft.setTextDatum(ML_DATUM); // Middle left
    tft.drawString(graphYLabels[graphPage], GRAPH_X - 5, GRAPH_Y + GRAPH_HEIGHT/2, 2);
    
    // Draw X axis label
    tft.setTextDatum(BC_DATUM); // Bottom center
    tft.drawString("Time (s)", GRAPH_X + GRAPH_WIDTH/2, GRAPH_Y + GRAPH_HEIGHT + 15, 2);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
    
    // Draw the graph data
    updateGraph();
}

// Update graph data points
void RocketMonitorScreen::updateGraph() {
    int graphPage;
    if (currentPage == PAGE_ALTITUDE_GRAPH) graphPage = 0;
    else if (currentPage == PAGE_SPEED_GRAPH) graphPage = 1;
    else if (currentPage == PAGE_POWER_GRAPH) graphPage = 2;
    else return; // Not a graph page
    
    // Find min and max values for scaling
    float minVal = 0;
    float maxVal = 0;
    bool firstPoint = true;
    
    for (int i = 0; i < historyCount; i++) {
        float val = dataHistory[graphPage][i];
        if (firstPoint || val < minVal) minVal = val;
        if (firstPoint || val > maxVal) maxVal = val;
        firstPoint = false;
    }
    
    // Ensure we have a range (avoid division by zero)
    if (maxVal == minVal) {
        maxVal = minVal + 1;
    }
    
    // Add some padding to the range
    float range = maxVal - minVal;
    minVal -= range * 0.1;
    maxVal += range * 0.1;
    
    // Draw the data points
    int prevX = 0;
    int prevY = 0;
    unsigned long maxTime = 0;
    
    // Find the maximum time for scaling
    if (historyCount > 0) {
        maxTime = timeHistory[(historyIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS];
        if (maxTime == 0) maxTime = 1; // Avoid division by zero
    } else {
        maxTime = 1; // Default if no data
    }
    
    // Clear the graph area (but not the axes)
    tft.fillRect(GRAPH_X + 1, GRAPH_Y, GRAPH_WIDTH - 1, GRAPH_HEIGHT, TFT_BLACK);
    
    // Draw the data points
    for (int i = 0; i < historyCount; i++) {
        int dataIndex = (historyIndex - historyCount + i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
        float val = dataHistory[graphPage][dataIndex];
        unsigned long time = timeHistory[dataIndex];
        
        // Scale to graph dimensions
        int x = GRAPH_X + (time * GRAPH_WIDTH) / maxTime;
        int y = GRAPH_Y + GRAPH_HEIGHT - ((val - minVal) * GRAPH_HEIGHT) / (maxVal - minVal);
        
        // Ensure y is within bounds
        y = constrain(y, GRAPH_Y, GRAPH_Y + GRAPH_HEIGHT);
        
        // Draw point
        tft.fillCircle(x, y, 2, graphColors[graphPage]);
        
        // Draw line from previous point
        if (i > 0) {
            tft.drawLine(prevX, prevY, x, y, graphColors[graphPage]);
        }
        
        prevX = x;
        prevY = y;
    }
    
    // Draw min/max values on Y axis
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MR_DATUM); // Middle right
    tft.drawString(String(maxVal, 1), GRAPH_X - 5, GRAPH_Y, 1);
    tft.drawString(String(minVal, 1), GRAPH_X - 5, GRAPH_Y + GRAPH_HEIGHT, 1);
    
    // Draw max time on X axis
    tft.setTextDatum(TC_DATUM); // Top center
    tft.drawString(String(maxTime/1000) + "s", GRAPH_X + GRAPH_WIDTH, GRAPH_Y + GRAPH_HEIGHT + 5, 1);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Draw transmitter selection page
void RocketMonitorScreen::drawTransmitterSelectionPage() {
    tft.fillScreen(TFT_BLACK);
    drawBatteryIndicator(batteryPercent);
    
    // Only show navigation buttons if a rocket has been selected
    if (rocketSelected) {
        drawNavigationButtons(true);
    }
    
    // Draw title
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM); // Top center
    tft.drawString("Select Rocket", tft.width()/2, 35, 2);
    tft.setTextDatum(TL_DATUM); // Reset to top left
    
    // Draw rescan button
    const int rescanBtnWidth = 100;
    const int rescanBtnHeight = 30;
    const int rescanBtnX = (tft.width() - rescanBtnWidth) / 2;
    const int rescanBtnY = tft.height() - rescanBtnHeight - 10;
    
    tft.fillRoundRect(rescanBtnX, rescanBtnY, rescanBtnWidth, rescanBtnHeight, 5, TFT_BLUE);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM); // Middle center
    tft.drawString("RESCAN", rescanBtnX + rescanBtnWidth/2, rescanBtnY + rescanBtnHeight/2, 2);
    tft.setTextDatum(TL_DATUM); // Reset to top left
    
    // If keyboard is showing, draw it
    if (showKeyboard) {
        drawKeyboard();
        return;
    }
    
    // If no transmitters found yet, show searching message
    if (numTransmitters == 0) {
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(MC_DATUM); // Middle center
        tft.drawString("Searching for rockets...", tft.width()/2, tft.height()/2, 2);
        tft.setTextDatum(TL_DATUM); // Reset to top left
        return;
    }
    
    // Draw list of transmitters
    const int startY = 60;
    const int itemHeight = 40; // Increased height for more info
    const int nameButtonWidth = 50;
    
    for (int i = 0; i < numTransmitters; i++) {
        uint16_t color = (i == selectedTransmitterIndex) ? TFT_GREEN : TFT_WHITE;
        tft.setTextColor(color, TFT_BLACK);
        
        // Draw rocket name or ID
        String displayText;
        if (rocketNames[i].length() > 0) {
            displayText = rocketNames[i];
        } else {
            displayText = "Rocket #" + String(knownTransmitters[i], HEX);
        }
        
        // Draw the text
        tft.drawString(displayText, 20, startY + i * itemHeight);
        
        // Draw frequency information
        String freqStr;
        if (rocketFrequency[i] > 0.0f) {
            // Determine radio type text
            String radioTypeStr;
            switch (radioType[i]) {
                case 0: radioTypeStr = "CC1101"; break;
                case 1: radioTypeStr = "SX1278"; break;
                case 2: radioTypeStr = "SX1262"; break;
                default: radioTypeStr = "Unknown";
            }
            
            freqStr = String(rocketFrequency[i], 3) + " MHz (" + radioTypeStr + ")";
            uint16_t freqColor = frequencyAcknowledged[i] ? TFT_GREEN : TFT_YELLOW;
            tft.setTextColor(freqColor, TFT_BLACK);
            tft.drawString(freqStr, 20, startY + i * itemHeight + 16);
            tft.setTextColor(color, TFT_BLACK); // Restore color
        } else {
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.drawString("Waiting for frequency...", 20, startY + i * itemHeight + 16);
            tft.setTextColor(color, TFT_BLACK); // Restore color
        }
        
        // Draw uptime if available
        if (i == selectedTransmitterIndex) {
            String uptimeStr = "Uptime: " + String(rocketUptime / 60000) + "m";
            tft.drawString(uptimeStr, 150, startY + i * itemHeight);
        }
        
        // Draw "NAME" button
        tft.fillRoundRect(tft.width() - nameButtonWidth - 10, startY + i * itemHeight - 5, 
                         nameButtonWidth, 25, 5, TFT_BLUE);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(MC_DATUM); // Middle center
        tft.drawString("NAME", tft.width() - nameButtonWidth/2 - 10, 
                      startY + i * itemHeight + 25/2 - 5, 2);
    }
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Draw on-screen keyboard
void RocketMonitorScreen::drawKeyboard() {
    // Draw input field with current text
    tft.fillRect(10, 60, tft.width() - 20, 30, TFT_NAVY);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(currentInput + "_", 15, 65, 2);
    
    // Draw keyboard background
    tft.fillRect(KEYBOARD_X - 5, KEYBOARD_Y - 5, 10 * KEY_WIDTH + 10, 4 * KEY_HEIGHT + 10, TFT_DARKGREY);
    
    // Draw keys
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 10; col++) {
            int x = KEYBOARD_X + col * KEY_WIDTH;
            int y = KEYBOARD_Y + row * KEY_HEIGHT;
            
            // Special keys
            if (row == 3 && col == 7) { // Backspace
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_RED);
                tft.setTextColor(TFT_WHITE);
                tft.setTextDatum(MC_DATUM);
                tft.drawString("<-", x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 2);
            } 
            else if (row == 3 && col == 8) { // Space
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_BLUE);
                tft.setTextColor(TFT_WHITE);
                tft.setTextDatum(MC_DATUM);
                tft.drawString("SPC", x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 1);
            }
            else if (row == 3 && col == 9) { // Enter
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_GREEN);
                tft.setTextColor(TFT_BLACK);
                tft.setTextDatum(MC_DATUM);
                tft.drawString("OK", x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 2);
            }
            else { // Regular character keys
                tft.fillRect(x, y, KEY_WIDTH, KEY_HEIGHT, TFT_LIGHTGREY);
                tft.setTextColor(TFT_BLACK);
                tft.setTextDatum(MC_DATUM);
                String keyStr(keyboardChars[row][col]);
                tft.drawString(keyStr, x + KEY_WIDTH/2, y + KEY_HEIGHT/2, 2);
            }
        }
    }
    
    // Draw cancel button
    tft.fillRect(KEYBOARD_X, KEYBOARD_Y + 4 * KEY_HEIGHT + 10, 5 * KEY_WIDTH, KEY_HEIGHT, TFT_RED);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("CANCEL", KEYBOARD_X + 2.5 * KEY_WIDTH, KEYBOARD_Y + 4.5 * KEY_HEIGHT + 10, 2);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Handle touch events
bool RocketMonitorScreen::handleTouch(int x, int y) {
    // If keyboard is showing, handle keyboard input
    if (showKeyboard) {
        processKeyPress(x, y);
        return true;
    }
    
    // Check for left navigation button
    if (isTouchInButton(x, y, NAV_BUTTON_MARGIN, NAV_BUTTON_MARGIN, NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT)) {
        // Go to previous page
        currentPage = static_cast<ScreenPage>((static_cast<int>(currentPage) - 1) % NUM_PAGES);
        updateDisplay();
        return true;
    }
    
    // Check for right navigation button
    if (isTouchInButton(x, y, tft.width() - NAV_BUTTON_MARGIN - NAV_BUTTON_WIDTH, NAV_BUTTON_MARGIN, NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT)) {
        // Go to next page
        currentPage = static_cast<ScreenPage>((static_cast<int>(currentPage) + 1) % NUM_PAGES);
        updateDisplay();
        return true;
    }
    
    // Private class member to track confirmation dialog state
    static bool showingAbortConfirmation = false;

// Handle page-specific touch events
    switch (currentPage) {
        case PAGE_MAIN:
            // If showing abort confirmation dialog, handle that first
            if (showingAbortConfirmation) {
                // Check if YES button was pressed
                if (isTouchInButton(x, y, tft.width()/2 - 100, tft.height()/2, 80, 40)) {
                    // User confirmed abort - send the command
                    abortSent = true;
                    showingAbortConfirmation = false;
                    
                    // Redraw main page
                    drawMainPage();
                    
                    // External function will need to be called to send the abort command
                    // This will be handled in the main loop
                    return true;
                }
                
                // Check if NO button was pressed
                if (isTouchInButton(x, y, tft.width()/2 + 20, tft.height()/2, 80, 40)) {
                    // User canceled abort
                    showingAbortConfirmation = false;
                    
                    // Redraw main page
                    drawMainPage();
                    return true;
                }
                
                // Ignore other touches while confirmation dialog is showing
                return true;
            }
            
            // Check for buzzer button (if a rocket is selected and either pre-launch or landed)
            if (rocketSelected && (launchState == LAUNCH_STATE_WAITING || launchState == LAUNCH_STATE_LANDED)) {
                // Check if touch is within buzzer button area
                if (isTouchInButton(x, y, tft.width() - 100, HEADER_HEIGHT + VALUE_HEIGHT * 6, 80, 30)) {
                    // Toggle buzzer state
                    buzzerActive = !buzzerActive;
                    
                    // Redraw button with new state
                    uint16_t buttonColor = buzzerActive ? TFT_RED : TFT_GREEN;
                    tft.fillRoundRect(tft.width() - 100, HEADER_HEIGHT + VALUE_HEIGHT * 6, 
                                     80, 30, 5, buttonColor);
                    
                    tft.setTextColor(TFT_WHITE, buttonColor);
                    tft.setTextDatum(MC_DATUM); // Middle center
                    tft.drawString(buzzerActive ? "BUZZER OFF" : "BUZZER ON", 
                                  tft.width() - 60, HEADER_HEIGHT + VALUE_HEIGHT * 6 + 15, 2);
                    tft.setTextDatum(TL_DATUM); // Back to top left
                    
                    // External function will need to be called to send the command
                    // This will be handled in the main loop
                    return true;
                }
            }
            
            // Check for abort button (if a rocket is selected and in flight)
            if (rocketSelected && launchState == LAUNCH_STATE_LAUNCHED && !abortSent) {
                // Check if touch is within abort button area
                if (isTouchInButton(x, y, 20, HEADER_HEIGHT + VALUE_HEIGHT * 6, 100, 40)) {
                    // Show confirmation dialog
                    showingAbortConfirmation = true;
                    
                    // Draw confirmation dialog
                    tft.fillRoundRect(tft.width()/2 - 150, tft.height()/2 - 100, 300, 200, 10, TFT_DARKGREY);
                    tft.fillRoundRect(tft.width()/2 - 145, tft.height()/2 - 95, 290, 190, 8, TFT_BLACK);
                    
                    // Dialog title
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.setTextDatum(TC_DATUM); // Top center
                    tft.drawString("CONFIRM EMERGENCY ABORT", tft.width()/2, tft.height()/2 - 80, 2);
                    
                    // Dialog message
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);
                    tft.setTextDatum(MC_DATUM); // Middle center
                    tft.drawString("Are you sure you want to", tft.width()/2, tft.height()/2 - 40, 2);
                    tft.drawString("trigger emergency parachute", tft.width()/2, tft.height()/2 - 20, 2);
                    tft.drawString("deployment?", tft.width()/2, tft.height()/2, 2);
                    
                    // YES button (red)
                    tft.fillRoundRect(tft.width()/2 - 100, tft.height()/2 + 30, 80, 40, 5, TFT_RED);
                    tft.setTextColor(TFT_WHITE, TFT_RED);
                    tft.drawString("YES", tft.width()/2 - 60, tft.height()/2 + 50, 2);
                    
                    // NO button (green)
                    tft.fillRoundRect(tft.width()/2 + 20, tft.height()/2 + 30, 80, 40, 5, TFT_GREEN);
                    tft.setTextColor(TFT_WHITE, TFT_GREEN);
                    tft.drawString("NO", tft.width()/2 + 60, tft.height()/2 + 50, 2);
                    
                    // Reset text alignment
                    tft.setTextDatum(TL_DATUM);
                    
                    return true;
                }
            }
            break;
            
        case PAGE_TRANSMITTER_SELECTION:
        {
            // Check for rescan button
            const int rescanBtnWidth = 100;
            const int rescanBtnHeight = 30;
            const int rescanBtnX = (tft.width() - rescanBtnWidth) / 2;
            const int rescanBtnY = tft.height() - rescanBtnHeight - 10;
            
            if (isTouchInButton(x, y, rescanBtnX, rescanBtnY, rescanBtnWidth, rescanBtnHeight)) {
                // Trigger rescan - this will be handled by the main loop
                rescanRequested = true;
                
                // Clear existing transmitter list
                numTransmitters = 0;
                selectedTransmitterIndex = -1;
                selectedTransmitterId = 0;
                rocketSelected = false;
                
                // Redraw the page with "Searching..." message
                drawTransmitterSelectionPage();
                return true;
            }
            
            // Check for transmitter selection
            for (int i = 0; i < numTransmitters; i++) {
                int yPos = 50 + i * 40;
                if (y >= yPos && y < yPos + 30 && x >= 10 && x < tft.width() - 10) {
                    // Select this transmitter
                    selectedTransmitterIndex = i;
                    selectedTransmitterId = knownTransmitters[i];
                    rocketSelected = true;
                    
                    // Switch to main data page
                    currentPage = PAGE_MAIN;
                    updateDisplay();
                    return true;
                }
            }
            break;
        }
            
        case PAGE_LAST_POSITIONS:
        {
            // Check for clear buttons
            int yPos = 50;
            for (int i = 0; i < numTransmitters; i++) {
                uint32_t id = knownTransmitters[i];
                if (id == 0) continue;
                
                float lat, lng;
                if (getLastKnownPosition(id, lat, lng)) {
                    // Check if clear button was pressed
                    if (isTouchInButton(x, y, tft.width() - 60, yPos + 5, 50, 30)) {
                        // Clear this position
                        clearLastKnownPosition(id);
                        // Redraw the page
                        drawLastPositionsPage();
                        return true;
                    }
                    
                    yPos += 60;
                }
            }
            break;
        }
    }
    
    return false;
}

// Process keyboard key press
void RocketMonitorScreen::processKeyPress(int x, int y) {
    // Check if in keyboard area
    if (x < KEYBOARD_X - 5 || x > KEYBOARD_X + 10 * KEY_WIDTH + 5 || 
        y < KEYBOARD_Y - 5 || y > KEYBOARD_Y + 5 * KEY_HEIGHT + 15) {
        return;
    }
    
    // Check for cancel button
    if (y > KEYBOARD_Y + 4 * KEY_HEIGHT + 10 && y < KEYBOARD_Y + 5 * KEY_HEIGHT + 10 &&
        x > KEYBOARD_X && x < KEYBOARD_X + 5 * KEY_WIDTH) {
        // Cancel editing
        showKeyboard = false;
        editingName = false;
        currentInput = "";
        editingTransmitterIndex = -1;
        drawTransmitterSelectionPage();
        return;
    }
    
    // Calculate which key was pressed
    int col = (x - KEYBOARD_X) / KEY_WIDTH;
    int row = (y - KEYBOARD_Y) / KEY_HEIGHT;
    
    // Ensure within bounds
    if (row < 0 || row >= 4 || col < 0 || col >= 10) {
        return;
    }
    
    // Handle special keys
    if (row == 3 && col == 7) { // Backspace
        if (currentInput.length() > 0) {
            currentInput = currentInput.substring(0, currentInput.length() - 1);
        }
    } 
    else if (row == 3 && col == 8) { // Space
        if (currentInput.length() < MAX_NAME_LENGTH) {
            currentInput += " ";
        }
    }
    else if (row == 3 && col == 9) { // Enter/OK
        saveRocketName();
        drawTransmitterSelectionPage();
        return;
    }
    else { // Regular character
        if (currentInput.length() < MAX_NAME_LENGTH) {
            currentInput += keyboardChars[row][col];
        }
    }
    
    // Redraw keyboard with updated input
    drawKeyboard();
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
            drawMainPage();
            break;
    }
}

// Save last known position for a transmitter
void RocketMonitorScreen::saveLastKnownPosition(uint32_t transmitterId, float lat, float lng) {
    if (transmitterId == 0 || lat == 0.0f || lng == 0.0f) {
        return; // Don't save invalid positions
    }
    
    lastPositionsStorage.begin("lastpos", false);
    
    // Create keys for this transmitter
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    String timeKey = "time_" + idStr;
    
    // Save position and current time
    lastPositionsStorage.putFloat(latKey.c_str(), lat);
    lastPositionsStorage.putFloat(lngKey.c_str(), lng);
    lastPositionsStorage.putULong(timeKey.c_str(), millis());
    
    lastPositionsStorage.end();
}

// Clear last known position for a transmitter
void RocketMonitorScreen::clearLastKnownPosition(uint32_t transmitterId) {
    if (transmitterId == 0) {
        return;
    }
    
    lastPositionsStorage.begin("lastpos", false);
    
    // Create keys for this transmitter
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    String timeKey = "time_" + idStr;
    
    // Remove position data
    lastPositionsStorage.remove(latKey.c_str());
    lastPositionsStorage.remove(lngKey.c_str());
    lastPositionsStorage.remove(timeKey.c_str());
    
    lastPositionsStorage.end();
}

// Get last known position for a transmitter
bool RocketMonitorScreen::getLastKnownPosition(uint32_t transmitterId, float &lat, float &lng) {
    if (transmitterId == 0) {
        return false;
    }
    
    lastPositionsStorage.begin("lastpos", true); // Read-only mode
    
    // Create keys for this transmitter
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    
    // Check if we have position data for this transmitter
    if (!lastPositionsStorage.isKey(latKey.c_str()) || !lastPositionsStorage.isKey(lngKey.c_str())) {
        lastPositionsStorage.end();
        return false;
    }
    
    // Get position data
    lat = lastPositionsStorage.getFloat(latKey.c_str(), 0.0f);
    lng = lastPositionsStorage.getFloat(lngKey.c_str(), 0.0f);
    
    lastPositionsStorage.end();
    return (lat != 0.0f && lng != 0.0f);
}

// Draw the orientation data page
void RocketMonitorScreen::drawOrientationPage() {
    // Clear screen
    tft.fillScreen(TFT_BLACK);
    
    // Draw page title
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(80, 10);
    tft.print("Rocket Orientation");
    
    // Draw navigation buttons
    drawNavigationButtons(rocketSelected);
    
    // Draw battery indicator
    drawBatteryIndicator(batteryPercent);
    
    // Draw tilt angle
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(20, 50);
    tft.print("Tilt from vertical:");
    
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(180, 50);
    // Convert from 0.5 degree resolution back to float
    float tiltDegrees = tiltAngle * 0.5f;
    tft.print(tiltDegrees, 1);
    tft.print("\xB0"); // Degree symbol
    
    // Draw rocket diagram
    int centerX = tft.width() / 2;
    int centerY = tft.height() / 2 + 10;
    int rocketHeight = 80;
    int rocketWidth = 30;
    
    // Calculate rocket tilt based on orientation data
    float tiltRadians = tiltDegrees * PI / 180.0f;
    
    // Draw rocket body (tilted)
    int tipX = centerX - sin(tiltRadians) * rocketHeight / 2;
    int tipY = centerY - cos(tiltRadians) * rocketHeight / 2;
    int baseX = centerX + sin(tiltRadians) * rocketHeight / 2;
    int baseY = centerY + cos(tiltRadians) * rocketHeight / 2;
    
    // Draw rocket body
    tft.drawLine(tipX, tipY, baseX, baseY, TFT_WHITE);
    
    // Draw rocket tip (triangle)
    int tipSize = 10;
    int tipX1 = tipX - cos(tiltRadians) * tipSize / 2;
    int tipY1 = tipY + sin(tiltRadians) * tipSize / 2;
    int tipX2 = tipX + cos(tiltRadians) * tipSize / 2;
    int tipY2 = tipY - sin(tiltRadians) * tipSize / 2;
    
    tft.fillTriangle(tipX, tipY, tipX1, tipY1, tipX2, tipY2, TFT_WHITE);
    
    // Draw fins
    int finSize = 15;
    int finX1 = baseX - cos(tiltRadians) * finSize;
    int finY1 = baseY + sin(tiltRadians) * finSize;
    int finX2 = baseX + cos(tiltRadians) * finSize;
    int finY2 = baseY - sin(tiltRadians) * finSize;
    int finOffset = 20;
    int finBaseX1 = baseX - sin(tiltRadians) * finOffset;
    int finBaseY1 = baseY - cos(tiltRadians) * finOffset;
    int finBaseX2 = baseX - sin(tiltRadians) * (finOffset - 10);
    int finBaseY2 = baseY - cos(tiltRadians) * (finOffset - 10);
    
    tft.drawLine(finBaseX1, finBaseY1, finX1, finY1, TFT_WHITE);
    tft.drawLine(finBaseX2, finBaseY2, finX2, finY2, TFT_WHITE);
    
    // Draw orientation vector values
    tft.setTextSize(1);
    tft.setCursor(20, tft.height() - 30);
    tft.setTextColor(TFT_YELLOW);
    tft.print("X-axis: ");
    tft.setTextColor(TFT_WHITE);
    tft.print(orientationX, 2);
    
    tft.setCursor(110, tft.height() - 30);
    tft.setTextColor(TFT_YELLOW);
    tft.print("Y-axis: ");
    tft.setTextColor(TFT_WHITE);
    tft.print(orientationY, 2);
    
    tft.setCursor(200, tft.height() - 30);
    tft.setTextColor(TFT_YELLOW);
    tft.print("Z-axis: ");
    tft.setTextColor(TFT_WHITE);
    tft.print(orientationZ, 2);
    
    // Draw staleness indicator
    unsigned long timeSinceLastPacket = millis() - lastPacketTime;
    tft.setTextSize(1);
    tft.setCursor(10, tft.height() - 15);
    tft.setTextColor(TFT_YELLOW);
    tft.print("Staleness: ");
    tft.setTextColor(timeSinceLastPacket > 10000 ? TFT_RED : TFT_WHITE);
    tft.print(timeSinceLastPacket / 1000);
    tft.print("s");
}

// Update orientation data display
void RocketMonitorScreen::updateOrientationData() {
    // Only update the dynamic parts of the orientation page
    if (currentPage == PAGE_ORIENTATION) {
        // Update tilt angle
        tft.fillRect(180, 50, 100, 20, TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(180, 50);
        float tiltDegrees = tiltAngle * 0.5f;
        tft.print(tiltDegrees, 1);
        tft.print("\xB0"); // Degree symbol
        
        // Redraw the entire orientation display since it's complex
        drawOrientationPage();
    }
}

// Draw the last known positions page
void RocketMonitorScreen::drawLastPositionsPage() {
    // Clear screen
    tft.fillScreen(TFT_BLACK);
    
    // Draw page title
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM); // Top center
    tft.drawString("Last Known Positions", tft.width()/2, 10, 2);
    
    // Draw navigation buttons
    drawNavigationButtons(rocketSelected);
    
    // Draw battery indicator
    drawBatteryIndicator(batteryPercent);
    
    // Set text properties for the list
    tft.setTextDatum(TL_DATUM); // Top left
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    
    int yPos = 50;
    int count = 0;
    
    // Display last known position for each transmitter
    for (int i = 0; i < numTransmitters; i++) {
        uint32_t id = knownTransmitters[i];
        if (id == 0) continue;
        
        float lat, lng;
        if (getLastKnownPosition(id, lat, lng)) {
            // Get rocket name
            String name = getTransmitterName(id);
            
            // Draw rocket name
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.drawString(name, 10, yPos, 2);
            
            // Draw coordinates
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            char coordStr[50];
            sprintf(coordStr, "Lat: %0.6f, Lng: %0.6f", lat, lng);
            tft.drawString(coordStr, 10, yPos + 20, 2);
            
            // Check if we have a launch position for this transmitter
            float launchLat, launchLng;
            if (getLaunchPosition(id, launchLat, launchLng)) {
                // Calculate distance and bearing from launch position
                float distance = calculateDistance(launchLat, launchLng, lat, lng);
                float bearing = calculateBearing(launchLat, launchLng, lat, lng);
                String direction = getBearingDirection(bearing);
                
                // Format distance (convert to meters if less than 1km)
                String distStr;
                if (distance < 1.0f) {
                    distStr = String(distance * 1000.0f, 0) + "m";
                } else {
                    distStr = String(distance, 2) + "km";
                }
                
                // Draw distance and bearing information
                char navStr[50];
                sprintf(navStr, "Dist: %s, Bearing: %.0f° (%s)", distStr.c_str(), bearing, direction.c_str());
                tft.drawString(navStr, 10, yPos + 40, 2);
                
                // Adjust vertical position for the additional line
                yPos += 80;
            } else {
                yPos += 60;
            }
            
            // Draw clear button
            tft.fillRoundRect(tft.width() - 60, yPos - 55, 50, 30, 5, TFT_RED);
            tft.setTextColor(TFT_WHITE, TFT_RED);
            tft.setTextDatum(MC_DATUM); // Middle center
            tft.drawString("Clear", tft.width() - 35, yPos - 40, 2);
            tft.setTextDatum(TL_DATUM); // Back to top left
            count++;
        }
    }
    
    // If no positions found
    if (count == 0) {
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(MC_DATUM); // Middle center
        tft.drawString("No saved positions", tft.width()/2, tft.height()/2, 2);
        tft.setTextDatum(TL_DATUM); // Back to top left
    }
}

// Display low battery warning screen
void RocketMonitorScreen::drawLowBattery() {
    // Clear screen
    tft.fillScreen(TFT_BLACK);
    
    // Display warning message
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextDatum(MC_DATUM); // Middle center
    tft.drawString("LOW BATTERY", tft.width()/2, tft.height()/2 - 20, 4);
    tft.drawString(String((int)batteryPercent) + "%", tft.width()/2, tft.height()/2 + 20, 4);
    tft.drawString("SHUTTING DOWN", tft.width()/2, tft.height()/2 + 60, 2);
    
    // Reset text alignment
    tft.setTextDatum(TL_DATUM);
}

// Save launch position for a transmitter
void RocketMonitorScreen::saveLaunchPosition(uint32_t transmitterId, float lat, float lng) {
    if (transmitterId == 0 || lat == 0.0f || lng == 0.0f) {
        return; // Don't save invalid positions
    }
    
    lastPositionsStorage.begin("launchpos", false);
    
    // Create keys for this transmitter
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    String timeKey = "time_" + idStr;
    
    // Save position and current time
    lastPositionsStorage.putFloat(latKey.c_str(), lat);
    lastPositionsStorage.putFloat(lngKey.c_str(), lng);
    lastPositionsStorage.putULong(timeKey.c_str(), millis());
    
    lastPositionsStorage.end();
}

// Clear launch position for a transmitter
void RocketMonitorScreen::clearLaunchPosition(uint32_t transmitterId) {
    if (transmitterId == 0) {
        return;
    }
    
    lastPositionsStorage.begin("launchpos", false);
    
    // Create keys for this transmitter
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    String timeKey = "time_" + idStr;
    
    // Remove position data
    lastPositionsStorage.remove(latKey.c_str());
    lastPositionsStorage.remove(lngKey.c_str());
    lastPositionsStorage.remove(timeKey.c_str());
    
    lastPositionsStorage.end();
}

// Get launch position for a transmitter
bool RocketMonitorScreen::getLaunchPosition(uint32_t transmitterId, float &lat, float &lng) {
    if (transmitterId == 0) {
        return false;
    }
    
    lastPositionsStorage.begin("launchpos", true); // Read-only mode
    
    // Create keys for this transmitter
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    
    // Check if we have position data for this transmitter
    if (!lastPositionsStorage.isKey(latKey.c_str()) || !lastPositionsStorage.isKey(lngKey.c_str())) {
        lastPositionsStorage.end();
        return false;
    }
    
    // Get position data
    lat = lastPositionsStorage.getFloat(latKey.c_str(), 0.0f);
    lng = lastPositionsStorage.getFloat(lngKey.c_str(), 0.0f);
    
    lastPositionsStorage.end();
    return (lat != 0.0f && lng != 0.0f);
}

// Calculate distance between two GPS coordinates in kilometers
float RocketMonitorScreen::calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    // Convert degrees to radians
    float lat1Rad = lat1 * PI / 180.0f;
    float lon1Rad = lon1 * PI / 180.0f;
    float lat2Rad = lat2 * PI / 180.0f;
    float lon2Rad = lon2 * PI / 180.0f;
    
    // Haversine formula
    float dlon = lon2Rad - lon1Rad;
    float dlat = lat2Rad - lat1Rad;
    float a = pow(sin(dlat/2), 2) + cos(lat1Rad) * cos(lat2Rad) * pow(sin(dlon/2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = 6371.0f * c; // Earth radius in km
    
    return distance;
}

// Calculate bearing between two GPS coordinates in degrees
float RocketMonitorScreen::calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    // Convert degrees to radians
    float lat1Rad = lat1 * PI / 180.0f;
    float lon1Rad = lon1 * PI / 180.0f;
    float lat2Rad = lat2 * PI / 180.0f;
    float lon2Rad = lon2 * PI / 180.0f;
    
    // Calculate bearing
    float y = sin(lon2Rad - lon1Rad) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(lon2Rad - lon1Rad);
    float bearing = atan2(y, x) * 180.0f / PI;
    
    // Normalize to 0-360 degrees
    bearing = fmod((bearing + 360.0f), 360.0f);
    
    return bearing;
}

// Convert bearing to cardinal direction
String RocketMonitorScreen::getBearingDirection(float bearing) {
    const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"};
    int index = round(bearing / 45.0f);
    if (index > 8) index = 0;
    return directions[index];
}
