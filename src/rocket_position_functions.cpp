#include "rocket_monitor_screen.h"
#include <math.h>

// Position storage and calculations
void RocketMonitorScreen::saveLastKnownPosition(uint32_t transmitterId, float lat, float lng) {
    lastPositionsStorage.begin("positions", false);
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    
    lastPositionsStorage.putFloat(latKey.c_str(), lat);
    lastPositionsStorage.putFloat(lngKey.c_str(), lng);
    lastPositionsStorage.end();
}

void RocketMonitorScreen::clearLastKnownPosition(uint32_t transmitterId) {
    lastPositionsStorage.begin("positions", false);
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    
    lastPositionsStorage.remove(latKey.c_str());
    lastPositionsStorage.remove(lngKey.c_str());
    lastPositionsStorage.end();
}

bool RocketMonitorScreen::getLastKnownPosition(uint32_t transmitterId, float &lat, float &lng) {
    lastPositionsStorage.begin("positions", false);
    String idStr = String(transmitterId, HEX);
    String latKey = "lat_" + idStr;
    String lngKey = "lng_" + idStr;
    
    if (lastPositionsStorage.isKey(latKey.c_str()) && lastPositionsStorage.isKey(lngKey.c_str())) {
        lat = lastPositionsStorage.getFloat(latKey.c_str(), 0.0f);
        lng = lastPositionsStorage.getFloat(lngKey.c_str(), 0.0f);
        lastPositionsStorage.end();
        return true;
    }
    
    lastPositionsStorage.end();
    return false;
}

// Launch position storage
void RocketMonitorScreen::saveLaunchPosition(uint32_t transmitterId, float lat, float lng) {
    lastPositionsStorage.begin("launches", false);
    String idStr = String(transmitterId, HEX);
    String latKey = "launch_lat_" + idStr;
    String lngKey = "launch_lng_" + idStr;
    
    lastPositionsStorage.putFloat(latKey.c_str(), lat);
    lastPositionsStorage.putFloat(lngKey.c_str(), lng);
    lastPositionsStorage.end();
}

void RocketMonitorScreen::clearLaunchPosition(uint32_t transmitterId) {
    lastPositionsStorage.begin("launches", false);
    String idStr = String(transmitterId, HEX);
    String latKey = "launch_lat_" + idStr;
    String lngKey = "launch_lng_" + idStr;
    
    lastPositionsStorage.remove(latKey.c_str());
    lastPositionsStorage.remove(lngKey.c_str());
    lastPositionsStorage.end();
}

bool RocketMonitorScreen::getLaunchPosition(uint32_t transmitterId, float &lat, float &lng) {
    lastPositionsStorage.begin("launches", false);
    String idStr = String(transmitterId, HEX);
    String latKey = "launch_lat_" + idStr;
    String lngKey = "launch_lng_" + idStr;
    
    if (lastPositionsStorage.isKey(latKey.c_str()) && lastPositionsStorage.isKey(lngKey.c_str())) {
        lat = lastPositionsStorage.getFloat(latKey.c_str(), 0.0f);
        lng = lastPositionsStorage.getFloat(lngKey.c_str(), 0.0f);
        lastPositionsStorage.end();
        return true;
    }
    
    lastPositionsStorage.end();
    return false;
}

// Distance and bearing calculations
float RocketMonitorScreen::calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    // Convert coordinates to radians
    float lat1Rad = lat1 * M_PI / 180.0;
    float lon1Rad = lon1 * M_PI / 180.0;
    float lat2Rad = lat2 * M_PI / 180.0;
    float lon2Rad = lon2 * M_PI / 180.0;
    
    // Earth radius in meters
    float earthRadius = 6371000.0;
    
    // Haversine formula
    float dLat = lat2Rad - lat1Rad;
    float dLon = lon2Rad - lon1Rad;
    float a = sin(dLat/2) * sin(dLat/2) + cos(lat1Rad) * cos(lat2Rad) * sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = earthRadius * c;
    
    return distance;
}

float RocketMonitorScreen::calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    // Convert coordinates to radians
    float lat1Rad = lat1 * M_PI / 180.0;
    float lon1Rad = lon1 * M_PI / 180.0;
    float lat2Rad = lat2 * M_PI / 180.0;
    float lon2Rad = lon2 * M_PI / 180.0;
    
    // Calculate bearing
    float y = sin(lon2Rad - lon1Rad) * cos(lat2Rad);
    float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(lon2Rad - lon1Rad);
    float bearing = atan2(y, x) * 180.0 / M_PI;
    
    // Normalize to 0-360 degrees
    if (bearing < 0) {
        bearing += 360.0;
    }
    
    return bearing;
}

String RocketMonitorScreen::getBearingDirection(float bearing) {
    // Convert bearing to cardinal direction
    if (bearing >= 337.5 || bearing < 22.5) return "N";
    if (bearing >= 22.5 && bearing < 67.5) return "NE";
    if (bearing >= 67.5 && bearing < 112.5) return "E";
    if (bearing >= 112.5 && bearing < 157.5) return "SE";
    if (bearing >= 157.5 && bearing < 202.5) return "S";
    if (bearing >= 202.5 && bearing < 247.5) return "SW";
    if (bearing >= 247.5 && bearing < 292.5) return "W";
    return "NW";
}
