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
    float accelBias[3];       // Bias offsets for x, y, z
    float lastAltitudeForVelocity; // Last altitude reading for baro velocity calc
    
    // Orientation estimation variables
    float rocketAxis[3];      // Rocket's vertical axis
    float gravityVector[3];   // Gravity vector in sensor frame
    
    // Maximum G-force
    float maxG;
    
    // Launch detection state (passed from main program)
    bool* pLaunchDetected;
    bool* pLandedDetected;
    
    // Friend class to allow access to private members
    friend class RocketDataLogger;
};

#endif // ROCKET_ACCELEROMETER_H
