#include "rocket_monitor_screen.h"
#include "rocket_telemetry_protocol.h"
#include <math.h>

// Draw battery indicator
void RocketMonitorScreen::drawBatteryIndicator(float batteryPercent) {
    int centerX = tft.width() / 2;
    int batteryX = centerX + 60; // Offset to right side of header
    int batteryY = 5;
    
    // Draw battery outline
    tft.drawRect(batteryX, batteryY, BATTERY_ICON_WIDTH, BATTERY_ICON_HEIGHT, ILI9341_WHITE);
    tft.drawRect(batteryX + BATTERY_ICON_WIDTH, batteryY + 2, 3, BATTERY_ICON_HEIGHT - 4, ILI9341_WHITE);
    
    // Draw battery fill based on percentage
    int fillWidth = (batteryPercent / 100.0) * (BATTERY_ICON_WIDTH - 2);
    
    // Choose color based on battery level
    uint16_t fillColor;
    if (batteryPercent > 75) {
        fillColor = ILI9341_GREEN;
    } else if (batteryPercent > 45) {
        fillColor = ILI9341_YELLOW;
    } else {
        fillColor = ILI9341_RED;
    }
    
    // Fill battery icon
    tft.fillRect(batteryX + 1, batteryY + 1, fillWidth, BATTERY_ICON_HEIGHT - 2, fillColor);
    
    // Clear unfilled portion
    if (fillWidth < BATTERY_ICON_WIDTH - 2) {
        tft.fillRect(batteryX + 1 + fillWidth, batteryY + 1, 
                    BATTERY_ICON_WIDTH - 2 - fillWidth, 
                    BATTERY_ICON_HEIGHT - 2, ILI9341_BLACK);
    }
}

// Draw navigation buttons
void RocketMonitorScreen::drawNavigationButtons(bool rocketSelected) {
    // Draw bottom navigation buttons
    int buttonY = tft.height() - NAV_BUTTON_HEIGHT - NAV_BUTTON_MARGIN;
    
    // Left button (back)
    tft.fillRoundRect(NAV_BUTTON_MARGIN, buttonY, 
                     NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT, 
                     5, NAV_BUTTON_COLOR);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(NAV_BUTTON_MARGIN + 10, buttonY + 8);
    tft.print("<");
    
    // Right button (forward)
    tft.fillRoundRect(tft.width() - NAV_BUTTON_WIDTH - NAV_BUTTON_MARGIN, 
                     buttonY, NAV_BUTTON_WIDTH, NAV_BUTTON_HEIGHT, 
                     5, NAV_BUTTON_COLOR);
    tft.setCursor(tft.width() - NAV_BUTTON_WIDTH - NAV_BUTTON_MARGIN + 10, buttonY + 8);
    tft.print(">");
}

// Draw main telemetry page
void RocketMonitorScreen::drawMainPage() {
    // Clear screen
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw header
    tft.fillRect(0, 0, tft.width(), HEADER_HEIGHT, ILI9341_NAVY);
    
    // Draw title
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(5, 10);
    
    if (rocketSelected) {
        tft.print(rocketNames[selectedTransmitterIndex]);
    } else {
        tft.print("No Rocket Selected");
    }
    
    // Draw battery indicator
    drawBatteryIndicator(batteryPercent);
    
    // Draw labels
    tft.setTextSize(1);
    tft.setTextColor(LABEL_COLOR);
    
    // Left column labels
    tft.setCursor(5, HEADER_HEIGHT + 5);
    tft.print("Altitude:");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT);
    tft.print("Max Alt:");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 2);
    tft.print("Speed:");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 3);
    tft.print("Temp:");
    
    tft.setCursor(5, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 4);
    tft.print("Max G:");
    
    // Right column labels
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5);
    tft.print("Lat:");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT);
    tft.print("Lng:");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 2);
    tft.print("Batt:");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 3);
    tft.print("TX Power:");
    
    tft.setCursor(RIGHT_COL, HEADER_HEIGHT + 5 + VALUE_HEIGHT * 4);
    tft.print("Uptime:");
    
    // Draw values
    tft.setTextSize(1);
    tft.setTextColor(VALUE_COLOR);
    
    // Update values
    updateMainPageValues();
    
    // Draw navigation buttons
    drawNavigationButtons(rocketSelected);
}

// Draw orientation page
void RocketMonitorScreen::drawOrientationPage() {
    // Clear screen
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw header
    tft.fillRect(0, 0, tft.width(), HEADER_HEIGHT, ILI9341_NAVY);
    
    // Draw title
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(5, 10);
    tft.print("Orientation");
    
    // Draw battery indicator
    drawBatteryIndicator(batteryPercent);
    
    // Draw orientation data
    updateOrientationData();
    
    // Draw navigation buttons
    drawNavigationButtons(rocketSelected);
}

// Update orientation data display
void RocketMonitorScreen::updateOrientationData() {
    // Center of the orientation display
    int centerX = tft.width() / 2;
    int centerY = HEADER_HEIGHT + (tft.height() - HEADER_HEIGHT) / 2;
    int radius = 80; // Size of the orientation circle
    
    // Clear the orientation area
    tft.fillRect(0, HEADER_HEIGHT, tft.width(), tft.height() - HEADER_HEIGHT - NAV_BUTTON_HEIGHT - NAV_BUTTON_MARGIN * 2, ILI9341_BLACK);
    
    // Draw orientation circle
    tft.drawCircle(centerX, centerY, radius, ILI9341_WHITE);
    
    // Draw crosshairs
    tft.drawLine(centerX - radius, centerY, centerX + radius, centerY, ILI9341_DARKGREY);
    tft.drawLine(centerX, centerY - radius, centerX, centerY + radius, ILI9341_DARKGREY);
    
    // Calculate orientation indicator position
    // Normalize orientation values to -1 to 1 range
    float normX = constrain(orientationX, -1.0f, 1.0f);
    float normY = constrain(orientationY, -1.0f, 1.0f);
    
    // Calculate position
    int indicatorX = centerX + (normX * radius * 0.8);
    int indicatorY = centerY - (normY * radius * 0.8); // Inverted Y for display
    
    // Draw orientation indicator
    tft.fillCircle(indicatorX, indicatorY, 10, ILI9341_RED);
    
    // Draw tilt angle
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(centerX - 40, centerY + radius + 20);
    tft.print("Tilt: ");
    tft.print(tiltAngle);
    tft.print((char)247); // Degree symbol
}

// Draw last positions page
void RocketMonitorScreen::drawLastPositionsPage() {
    // Clear screen
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw header
    tft.fillRect(0, 0, tft.width(), HEADER_HEIGHT, ILI9341_NAVY);
    
    // Draw title
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(5, 10);
    tft.print("Last Positions");
    
    // Draw battery indicator
    drawBatteryIndicator(batteryPercent);
    
    // Display current position
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(5, HEADER_HEIGHT + 10);
    tft.print("Current Position:");
    
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(5, HEADER_HEIGHT + 25);
    tft.print("Lat: ");
    tft.print(currentLat, 6);
    
    tft.setCursor(5, HEADER_HEIGHT + 40);
    tft.print("Lng: ");
    tft.print(currentLng, 6);
    
    // Display launch position if available
    float launchLat, launchLng;
    if (getLaunchPosition(selectedTransmitterId, launchLat, launchLng)) {
        tft.setTextColor(ILI9341_YELLOW);
        tft.setCursor(5, HEADER_HEIGHT + 60);
        tft.print("Launch Position:");
        
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(5, HEADER_HEIGHT + 75);
        tft.print("Lat: ");
        tft.print(launchLat, 6);
        
        tft.setCursor(5, HEADER_HEIGHT + 90);
        tft.print("Lng: ");
        tft.print(launchLng, 6);
        
        // Calculate distance from launch
        float distance = calculateDistance(currentLat, currentLng, launchLat, launchLng);
        float bearing = calculateBearing(launchLat, launchLng, currentLat, currentLng);
        String direction = getBearingDirection(bearing);
        
        tft.setTextColor(ILI9341_YELLOW);
        tft.setCursor(5, HEADER_HEIGHT + 110);
        tft.print("Distance from Launch:");
        
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(5, HEADER_HEIGHT + 125);
        if (distance < 1000) {
            tft.print(distance, 1);
            tft.print(" m ");
        } else {
            tft.print(distance/1000.0, 2);
            tft.print(" km ");
        }
        
        tft.print(direction);
        tft.print(" (");
        tft.print((int)bearing);
        tft.print((char)247); // Degree symbol
        tft.print(")");
    }
    
    // Draw navigation buttons
    drawNavigationButtons(rocketSelected);
}

// Draw low battery warning
void RocketMonitorScreen::drawLowBattery() {
    // Clear screen
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw warning box
    int boxWidth = 200;
    int boxHeight = 100;
    int boxX = (tft.width() - boxWidth) / 2;
    int boxY = (tft.height() - boxHeight) / 2;
    
    tft.fillRoundRect(boxX, boxY, boxWidth, boxHeight, 10, ILI9341_RED);
    tft.drawRoundRect(boxX, boxY, boxWidth, boxHeight, 10, ILI9341_WHITE);
    
    // Draw warning text
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    
    tft.setCursor(boxX + 20, boxY + 20);
    tft.print("LOW BATTERY");
    
    tft.setCursor(boxX + 20, boxY + 50);
    tft.print("Shutting down...");
    
    // Draw battery icon
    int iconX = boxX + boxWidth - 50;
    int iconY = boxY + 20;
    
    tft.drawRect(iconX, iconY, 30, 15, ILI9341_WHITE);
    tft.drawRect(iconX + 30, iconY + 3, 3, 9, ILI9341_WHITE);
    tft.fillRect(iconX + 2, iconY + 2, 5, 11, ILI9341_WHITE);
}
