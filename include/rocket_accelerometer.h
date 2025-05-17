#ifndef ROCKET_ACCELEROMETER_H
#define ROCKET_ACCELEROMETER_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <SD.h>

class RocketAccelerometer {
public:
    // Constructor
    RocketAccelerometer();
    
    // Initialization
    bool begin();
    
    // Calibration
    void calibrate(const char* filename_base = nullptr);
    
    // Velocity calculation
    void updateVelocity(sensors_event_t& accel);
    
    // Update barometric velocity
    void updateBaroVelocity(float currentAltitude);
    
    // Getters
    float getAccelVelocity() const { return accelVelocity; }
    float getMaxAccelVelocity() const { return maxAccelVelocity; }
    float getBaroVelocity() const { return baroVelocity; }
    float getMaxBaroVelocity() const { return maxBaroVelocity; }
    float getMaxG() const { return maxG; }
    
    // Orientation getters
    float getTiltAngle() const; // Returns tilt angle in degrees
    void getCurrentOrientation(float& x, float& y, float& z) const; // Gets current orientation vector
    uint8_t getTiltAngleInt() const { return (uint8_t)(getTiltAngle() * 2); } // Returns tilt angle * 2 as uint8_t (0.5 degree resolution)
    
    // Reset maximum values
    void resetMaxValues();
    
    // Set pointers to launch detection state variables
    void setLaunchDetectionPointers(bool* launchDetected, bool* landedDetected);
    
    // MPU6050 sensor - made public for direct access
    Adafruit_MPU6050 mpu;
    
private:
    
    // Velocity calculation variables
    float accelVelocity;      // Velocity from accelerometer integration
    float maxAccelVelocity;   // Maximum accelerometer-based velocity
    float baroVelocity;       // Velocity from barometric altitude changes
    float maxBaroVelocity;    // Maximum barometer-based velocity
    unsigned long lastAccelReadTime;
    unsigned long lastBaroReadTime;
    float lastAltitudeForVelocity;    // Last altitude reading for baro velocity calc
    float maxG;               // Maximum G-force experienced
    
    // Bias and orientation variables
    float accelBias[3];       // Bias offsets for x, y, z
    float rocketAxis[3];      // Rocket's vertical axis (normalized)
    float gravityVector[3];   // Gravity vector in sensor frame (normalized)
    float tiltAngle;          // Tilt angle from vertical in degrees
    float currentOrientation[3]; // Current orientation vector (updated during flight)
    
    // Launch detection pointers
    bool* pLaunchDetected;
    bool* pLandedDetected;
    
    // Friend class to allow access to private members
    friend class RocketDataLogger;
};

#endif // ROCKET_ACCELEROMETER_H
