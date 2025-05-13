#include "XPT2046_Touchscreen.h"

#define Z_THRESHOLD 400
#define Z_THRESHOLD_INT 75
#define MSEC_THRESHOLD 3
#define SPI_SETTING SPISettings(2000000, MSBFIRST, SPI_MODE0)

XPT2046_Touchscreen::XPT2046_Touchscreen(uint8_t cs_pin, uint8_t tirq_pin) {
    csPin = cs_pin;
    tirqPin = tirq_pin;
    rotation = 0;
    xraw = 0;
    yraw = 0;
    zraw = 0;
}

bool XPT2046_Touchscreen::begin() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    
    if (tirqPin != 255) {
        pinMode(tirqPin, INPUT);
    }
    
    return true;
}

void XPT2046_Touchscreen::setRotation(uint8_t n) {
    rotation = n % 4;
}

TS_Point XPT2046_Touchscreen::getPoint() {
    update();
    
    int16_t x, y;
    
    switch (rotation) {
        case 0:
            x = 4095 - xraw;
            y = yraw;
            break;
        case 1:
            x = 4095 - yraw;
            y = 4095 - xraw;
            break;
        case 2:
            x = xraw;
            y = 4095 - yraw;
            break;
        case 3:
        default:
            x = yraw;
            y = xraw;
            break;
    }
    
    return TS_Point(x, y, zraw);
}

bool XPT2046_Touchscreen::touched() {
    update();
    return (zraw >= Z_THRESHOLD);
}

void XPT2046_Touchscreen::update() {
    update_axis(0xB1); // Z1
    zraw = z_measures[0];
    if (zraw >= Z_THRESHOLD) {
        update_axis(0xC1); // Z2
        update_axis(0x91); // X
        update_axis(0xD1); // Y
        
        zraw = besttwoavg(z_measures[0], z_measures[1], z_measures[2]);
        xraw = besttwoavg(x_measures[0], x_measures[1], x_measures[2]);
        yraw = besttwoavg(y_measures[0], y_measures[1], y_measures[2]);
    }
}

void XPT2046_Touchscreen::update_axis(uint8_t axis) {
    SPI.beginTransaction(SPI_SETTING);
    digitalWrite(csPin, LOW);
    
    SPI.transfer(axis);
    
    int16_t data = SPI.transfer16(0) >> 3;
    
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    
    if (axis == 0xB1) { // Z1
        z_measures[0] = data;
    } else if (axis == 0xC1) { // Z2
        z_measures[1] = data;
    } else if (axis == 0x91) { // X
        x_measures[0] = data;
    } else if (axis == 0xD1) { // Y
        y_measures[0] = data;
    }
}

uint16_t XPT2046_Touchscreen::besttwoavg(uint16_t x, uint16_t y, uint16_t z) {
    uint16_t da, db, dc;
    uint16_t reta = 0;
    
    da = abs(x - y);
    db = abs(x - z);
    dc = abs(y - z);
    
    if (da <= db && da <= dc) {
        reta = (x + y) >> 1;
    } else if (db <= da && db <= dc) {
        reta = (x + z) >> 1;
    } else {
        reta = (y + z) >> 1;
    }
    
    return reta;
}
