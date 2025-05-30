#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

// Create an instance of the TFT_eSPI class
TFT_eSPI tft = TFT_eSPI();

// Pin definitions - define these explicitly to ensure they match User_Setup.h
#define TFT_MISO_PIN 13
#define TFT_MOSI_PIN 11
#define TFT_SCLK_PIN 12
#define TFT_CS_PIN   10
#define TFT_DC_PIN    9
#define TFT_RST_PIN  14

void setup() {
  Serial.begin(115200);
  delay(2000); // Give serial time to initialize
  
  Serial.println("\n\nTFT_eSPI Init-Only Test");
  Serial.println("------------------------");
  
  // Initialize SPI manually first
  Serial.println("1. Initializing SPI bus manually");
  SPI.begin(TFT_SCLK_PIN, TFT_MISO_PIN, TFT_MOSI_PIN, TFT_CS_PIN);
  SPI.setFrequency(1000000); // Very low frequency for reliability
  Serial.println("   SPI initialized");
  
  // Initialize pins manually
  Serial.println("2. Setting up pins manually");
  pinMode(TFT_DC_PIN, OUTPUT);
  pinMode(TFT_CS_PIN, OUTPUT);
  digitalWrite(TFT_CS_PIN, HIGH); // Deselect by default
  
  // Perform hardware reset
  Serial.println("3. Performing hardware reset");
  pinMode(TFT_RST_PIN, OUTPUT);
  digitalWrite(TFT_RST_PIN, HIGH);
  delay(100);
  digitalWrite(TFT_RST_PIN, LOW);
  delay(100);
  digitalWrite(TFT_RST_PIN, HIGH);
  delay(200);
  Serial.println("   Reset complete");
  
  // Initialize TFT with minimal operations
  Serial.println("4. Attempting TFT initialization");
  
  // Try to initialize with a delay before each step
  delay(100);
  
  // Initialize TFT
  tft.init();
  Serial.println("   TFT initialized successfully!");
  
  // Just set rotation and clear screen - minimal operations
  tft.setRotation(1);
  Serial.println("   Rotation set");
  
  tft.fillScreen(TFT_BLACK);
  Serial.println("   Screen cleared");
  
  Serial.println("Setup complete - if you see this message, initialization was successful!");
}

void loop() {
  // Just blink the built-in LED to show the program is running
  static uint32_t lastToggle = 0;
  static bool ledState = false;
  
  if (millis() - lastToggle > 500) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastToggle = millis();
    
    // Print a message every second to show we're still running
    if (ledState) {
      Serial.println("Running...");
    }
  }
}
