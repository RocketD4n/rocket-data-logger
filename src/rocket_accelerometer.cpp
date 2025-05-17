#include "rocket_accelerometer.h"

RocketAccelerometer::RocketAccelerometer() :
    accelVelocity(0.0f),
    maxAccelVelocity(0.0f),
    baroVelocity(0.0f),
    maxBaroVelocity(0.0f),
    lastAccelReadTime(0),
    lastBaroReadTime(0),
    lastAltitudeForVelocity(0.0f),
    maxG(0.0f),
    pLaunchDetected(nullptr),
    pLandedDetected(nullptr)
{
    // Initialize arrays
    accelBias[0] = accelBias[1] = accelBias[2] = 0.0f;
    rocketAxis[0] = rocketAxis[1] = 0.0f;
    rocketAxis[2] = 1.0f; // Default orientation (Z-axis is up)
    gravityVector[0] = gravityVector[1] = gravityVector[2] = 0.0f;
}

bool RocketAccelerometer::begin() {
    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
        return false;
    }
    
    Serial.println("MPU6050 ready!");
    
    // Set accelerometer range to +/- 16g for better high-acceleration measurements
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    
    return true;
}

void RocketAccelerometer::setLaunchDetectionPointers(bool* launchDetected, bool* landedDetected) {
    pLaunchDetected = launchDetected;
    pLandedDetected = landedDetected;
}

void RocketAccelerometer::calibrate(const char* filename_base) {
    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    float rawX[numSamples], rawY[numSamples], rawZ[numSamples]; // Store raw values for logging
    
    Serial.println("Calibrating accelerometer... Keep device still.");
    
    // Take multiple readings and average them
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // Store raw values
        rawX[i] = a.acceleration.x;
        rawY[i] = a.acceleration.y;
        rawZ[i] = a.acceleration.z;
        
        // Sum the readings
        sumX += a.acceleration.x;
        sumY += a.acceleration.y;
        sumZ += a.acceleration.z;
        
        delay(10);
    }
    
    // Calculate average values
    float avgX = sumX / numSamples;
    float avgY = sumY / numSamples;
    float avgZ = sumZ / numSamples;
    
    // Store the gravity vector in sensor frame
    // This represents the direction of gravity relative to the sensor
    gravityVector[0] = avgX;
    gravityVector[1] = avgY;
    gravityVector[2] = avgZ;
    
    // Normalize the gravity vector
    float gravityMagnitude = sqrt(avgX*avgX + avgY*avgY + avgZ*avgZ);
    gravityVector[0] /= gravityMagnitude;
    gravityVector[1] /= gravityMagnitude;
    gravityVector[2] /= gravityMagnitude;
    
    // Calculate the rocket's vertical axis (opposite to gravity vector)
    rocketAxis[0] = -gravityVector[0];
    rocketAxis[1] = -gravityVector[1];
    rocketAxis[2] = -gravityVector[2];
    
    // Calculate bias values (removing gravity component)
    // We only want to remove electronic bias, not gravity
    accelBias[0] = avgX - (gravityVector[0] * 9.8f);
    accelBias[1] = avgY - (gravityVector[1] * 9.8f);
    accelBias[2] = avgZ - (gravityVector[2] * 9.8f);
    
    // Calculate tilt angle from vertical (in degrees)
    float tiltAngle = acos(rocketAxis[2]) * 180.0f / PI;
    
    Serial.println("Accelerometer calibration complete.");
    Serial.print("Bias X: "); Serial.print(accelBias[0]);
    Serial.print(" Y: "); Serial.print(accelBias[1]);
    Serial.print(" Z: "); Serial.println(accelBias[2]);
    
    Serial.println("Rocket orientation estimated:");
    Serial.print("Vertical axis X: "); Serial.print(rocketAxis[0]);
    Serial.print(" Y: "); Serial.print(rocketAxis[1]);
    Serial.print(" Z: "); Serial.println(rocketAxis[2]);
    Serial.print("Tilt from vertical: "); Serial.print(tiltAngle); Serial.println(" degrees");
    
    // Log calibration data to SD card if filename_base is provided
    if (filename_base != nullptr) {
        // Create filename
        char filename[40];
        snprintf(filename, sizeof(filename), "%s_cal.txt", filename_base);
        
        // Open file for writing
        File calibFile = SD.open(filename, FILE_WRITE);
        if (calibFile) {
            // Write header and calibration results
            calibFile.println("Rocket Data Logger - Accelerometer Calibration Data");
            calibFile.println("----------------------------------------");
            calibFile.print("Timestamp: ");
            calibFile.println(filename_base);
            calibFile.println();
            
            calibFile.println("Raw Accelerometer Data:");
            calibFile.println("Sample,X,Y,Z");
            for (int i = 0; i < numSamples; i++) {
                calibFile.print(i);
                calibFile.print(",");
                calibFile.print(rawX[i], 6);
                calibFile.print(",");
                calibFile.print(rawY[i], 6);
                calibFile.print(",");
                calibFile.println(rawZ[i], 6);
            }
            calibFile.println();
            
            calibFile.println("Calibration Results:");
            calibFile.print("Average Raw X: "); calibFile.println(avgX, 6);
            calibFile.print("Average Raw Y: "); calibFile.println(avgY, 6);
            calibFile.print("Average Raw Z: "); calibFile.println(avgZ, 6);
            calibFile.println();
            
            calibFile.print("Gravity Vector X: "); calibFile.println(gravityVector[0], 6);
            calibFile.print("Gravity Vector Y: "); calibFile.println(gravityVector[1], 6);
            calibFile.print("Gravity Vector Z: "); calibFile.println(gravityVector[2], 6);
            calibFile.println();
            
            calibFile.print("Rocket Axis X: "); calibFile.println(rocketAxis[0], 6);
            calibFile.print("Rocket Axis Y: "); calibFile.println(rocketAxis[1], 6);
            calibFile.print("Rocket Axis Z: "); calibFile.println(rocketAxis[2], 6);
            calibFile.println();
            
            calibFile.print("Tilt Angle (degrees): "); calibFile.println(tiltAngle, 2);
            calibFile.println();
            
            calibFile.print("Bias X: "); calibFile.println(accelBias[0], 6);
            calibFile.print("Bias Y: "); calibFile.println(accelBias[1], 6);
            calibFile.print("Bias Z: "); calibFile.println(accelBias[2], 6);
            
            // Close the file
            calibFile.close();
            Serial.print("Calibration data saved to ");
            Serial.println(filename);
        } else {
            Serial.print("Error opening calibration file: ");
            Serial.println(filename);
        }
    }
}

void RocketAccelerometer::updateVelocity(sensors_event_t& accel) {
    unsigned long currentTime = millis();
    
    // First time through, just initialize the time
    if (lastAccelReadTime == 0) {
        lastAccelReadTime = currentTime;
        return;
    }
    
    // Calculate time delta in seconds
    float dt = (currentTime - lastAccelReadTime) / 1000.0f;
    lastAccelReadTime = currentTime;
    
    // Ignore very large time steps (e.g., after long delays)
    if (dt > 0.1f) {
        dt = 0.1f;
    }
    
    // Apply bias correction to raw acceleration values
    float accelX = accel.acceleration.x - accelBias[0];
    float accelY = accel.acceleration.y - accelBias[1];
    float accelZ = accel.acceleration.z - accelBias[2];
    
    // Project acceleration onto rocket's vertical axis
    // This gives us the acceleration along the rocket's flight path
    float accelAlongAxis = accelX * rocketAxis[0] + 
                           accelY * rocketAxis[1] + 
                           accelZ * rocketAxis[2];
    
    // Handle gravity compensation based on flight state
    if ((pLaunchDetected == nullptr || !(*pLaunchDetected)) || 
        (pLandedDetected != nullptr && *pLandedDetected)) {
        // When not in flight, subtract gravity component along rocket axis
        accelAlongAxis -= 9.8f;
    }
    
    // Compute total acceleration magnitude for maxG calculation
    float accelMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    // Update maxG (convert to g units)
    float currentG = accelMagnitude / 9.8f;
    if (currentG > maxG) {
        maxG = currentG;
    }
    
    // Reset velocity to zero if we're not in flight and not moving much
    // This helps prevent drift when the rocket is stationary
    if ((pLaunchDetected == nullptr || !(*pLaunchDetected)) && abs(accelAlongAxis) < 0.5f) {
        accelVelocity = 0.0f;
        return;
    }
    
    // Integrate acceleration to get velocity
    accelVelocity += accelAlongAxis * dt;
    
    // Track maximum velocity (absolute value)
    if (abs(accelVelocity) > abs(maxAccelVelocity)) {
        maxAccelVelocity = accelVelocity;
    }
    
    // If we've landed, gradually decay the velocity to zero
    // This helps correct for drift
    if (pLandedDetected != nullptr && *pLandedDetected && abs(accelVelocity) < 2.0f) {
        accelVelocity *= 0.9f; // Decay factor
    }
}

void RocketAccelerometer::updateBaroVelocity(float currentAltitude) {
    unsigned long currentTime = millis();
    
    // First time through, just initialize values
    if (lastBaroReadTime == 0) {
        lastBaroReadTime = currentTime;
        lastAltitudeForVelocity = currentAltitude;
        return;
    }
    
    // Calculate time delta in seconds
    float dt = (currentTime - lastBaroReadTime) / 1000.0f;
    lastBaroReadTime = currentTime;
    
    // Ignore very large time steps
    if (dt > 0.5f) {
        lastAltitudeForVelocity = currentAltitude;
        lastBaroReadTime = currentTime;
        return;
    }
    
    // Calculate velocity as change in altitude over time (m/s)
    float altitudeChange = currentAltitude - lastAltitudeForVelocity;
    baroVelocity = altitudeChange / dt;
    
    // Apply a simple low-pass filter to reduce noise
    static float filteredVelocity = 0.0f;
    filteredVelocity = filteredVelocity * 0.7f + baroVelocity * 0.3f;
    baroVelocity = filteredVelocity;
    
    // Track maximum velocity (absolute value)
    if (abs(baroVelocity) > abs(maxBaroVelocity)) {
        maxBaroVelocity = baroVelocity;
    }
    
    // Store current values for next iteration
    lastAltitudeForVelocity = currentAltitude;
    lastBaroReadTime = currentTime;
}

void RocketAccelerometer::resetMaxValues() {
    maxAccelVelocity = 0.0f;
    maxBaroVelocity = 0.0f;
    maxG = 0.0f;
}
