#ifndef _XPT2046_TOUCHSCREEN_H_
#define _XPT2046_TOUCHSCREEN_H_

#include <Arduino.h>
#include <SPI.h>

#define XPT2046_DRIVER 1

class TS_Point {
public:
    TS_Point(void) : x(0), y(0), z(0) {}
    TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
    bool operator==(TS_Point p) { return ((p.x == x) && (p.y == y) && (p.z == z)); }
    bool operator!=(TS_Point p) { return ((p.x != x) || (p.y != y) || (p.z != z)); }
    int16_t x, y, z;
};

class XPT2046_Touchscreen {
public:
    XPT2046_Touchscreen(uint8_t cs_pin, uint8_t tirq_pin = 255);
    bool begin();
    void setRotation(uint8_t rotation);
    TS_Point getPoint();
    bool touched();
    void update();

private:
    void update_axis(uint8_t axis);
    uint16_t besttwoavg(uint16_t x, uint16_t y, uint16_t z);
    uint8_t csPin, tirqPin, rotation;
    int16_t xraw, yraw, zraw;
    uint16_t x_measures[3], y_measures[3], z_measures[3];
};

#endif
