/*
 *  TODO:
 *  add led when launch detected
 *  initialize time with gbp
 *  log gps values
 *  relay output after x meters of drop 
 */
 
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define LED_PIN 2 // Built-in LED on NodeMCU (D4)

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
File Textfile;
Adafruit_MPU6050 mpu;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(5, 4); // D1 (GPIO5) for RX, D2 (GPIO4) for TX

int countFiles(File dir) {
  int count = 0;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      count++;
    }
    entry.close();
  }
  return count;
}

void getAltitudeAndTemp(float& altitude, float& temp) {
  sensors_event_t event;
  bmp.getEvent(&event);
  altitude = bmp.pressureToAltitude(1013.25, event.pressure);
  bmp.getTemperature(&temp);
}

void setup() {
  Wire.begin(D2, D1);
  Serial.begin(74880);
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start with LED off

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    return;
  }
  Serial.println("MPU6050 ready!");

  if (!bmp.begin()) {
    Serial.println("BMP180 not detected. Check wiring.");
    return;
  }
  Serial.println("BMP180 ready.");
  
  Serial.println("Initializing SD card");
  if (!SD.begin(15)) {
    Serial.println("Initialization failed!");
    return;
  }

  Serial.println("Waiting for GPS....");
  gpsSerial.begin(9600);
  while (gpsSerial.available() <= 0) {
    // wait for hdop to be < 3
    while (gpsSerial.available() > 0) 
      gps.encode(gpsSerial.read());
   // gps.hdop.hdop();
  }
  
  
 // gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.altitude.meters()
 // gps.date.year(), month(), day(), hour(), minute(), second(), 
  Serial.println("Initialization complated");

  // Get GPS date and time
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Format filename as YYYYMMDD_HHMMSS.csv
  char filename[24];
  snprintf(filename, sizeof(filename), "%04d%02d%02d_%02d%02d%02d.csv",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
  Textfile = SD.open(filename, FILE_WRITE);

  if (Textfile) {
    Serial.print("Writing header to file ");
    Serial.println(filename);
    Textfile.println("micros, XG, YG, ZG, XAc, YAc, ZAc, altitude, temp");
    Textfile.print(micros());
    float altitude, temp;
    getAltitudeAndTemp(altitude, temp);
    Textfile.print(",0,0,0,0,0,0,");
    Textfile.print(altitude);
    Textfile.print(",");
    Textfile.println(temp);
    Textfile.flush();
    Serial.println("done");
  }

  Serial.println("Waiting for launch....");
}

bool launchDetected = false;
unsigned lastFlush = 0;

void loop() { 
  char line[512];
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (!launchDetected) {
    // Compute total acceleration magnitude
    float aTotal = sqrt(a.acceleration.x * a.acceleration.x 
                      + a.acceleration.y * a.acceleration.y 
                      + a.acceleration.z * a.acceleration.z);
    if (aTotal > 1.7) { // 1.8?
      Serial.println("Launch detected!");
      launchDetected = true;
      digitalWrite(LED_PIN, HIGH);  // Turn on LED when launch is detected
    }
    else {
      delay(10);  // milliseconds
      return;
    }
  }
  float altitude, temperature;
  getAltitudeAndTemp(altitude, temperature);

  while (gpsSerial.available() > 0) 
    gps.encode(gpsSerial.read());
  
 // gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.altitude.meters()
//  gps.date.year(), month(), day(), hour(), minute(), second(), 
  snprintf(line, sizeof(line), "%lu,%d,%d,%d,%d,%d,%d,%f,%f", micros(), 
            g.gyro.x, g.gyro.y, g.gyro.z,  // divide by 131.0 to convert to degrees per second
            a.acceleration.x, a.acceleration.y, a.acceleration.z,  // divide by 16384.0 to convert to accel in G
            altitude, temperature);
  Textfile.println(line);
  Serial.println(line);
  if (micros() - lastFlush > 1000000) { // flush to write the file from the memory buffer at least every second
    Textfile.flush();
    lastFlush = micros();
  }
  delay(50); // millis
}
