/*
 * Board Configuration File
 * 
 * This file contains pin definitions and configurations for different ESP32 boards.
 * Change BOARD_TYPE to switch between different supported boards.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// Board type: 0 = ESP32-C3 SuperMini, 1 = Regular ESP32
#ifndef BOARD_TYPE
#define BOARD_TYPE 1
#endif

// Use SoftwareSerial for GPS on ESP32-C3, use hardware Serial2 for ESP32
#if BOARD_TYPE == 0
#define USE_SOFTWARE_SERIAL_GPS 1  // ESP32-C3 uses SoftwareSerial
#else
#define USE_SOFTWARE_SERIAL_GPS 0  // Regular ESP32 uses hardware Serial2
#endif

// ===== ESP32-C3 SuperMini Pin Definitions =====
#if BOARD_TYPE == 0
    // Radio configuration pins
    #define RADIO_CS 7     // GPIO7 on ESP32-C3
    #define RADIO_DIO0 2   // GPIO2 on ESP32-C3 (DIO0 for SX1278, GDO0 for CC1101)
    #define RADIO_DIO1 6   // GPIO6 on ESP32-C3 (DIO1 for SX1262)
    #define RADIO_BUSY 3   // GPIO3 on ESP32-C3 (BUSY for SX1262)
    #define RADIO_RST 1    // GPIO1 on ESP32-C3 (RESET for SX1262/SX1278)
    
    // I2C pins
    #define I2C_SDA 20     // GPIO20 for SDA
    #define I2C_SCL 21     // GPIO21 for SCL
    
    // GPS pins
    #define GPS_RX_PIN 3   // GPIO3 for RX
    #define GPS_TX_PIN 0   // GPIO0 for TX
    #define USE_SOFTWARE_SERIAL_GPS 1  // Use SoftwareSerial for GPS
    
    // SD Card
    #define SD_CS_PIN 10   // GPIO10 for SD Card CS
    
    // LED pin
    #define LED_PIN 8      // Built-in LED on GPIO8

// ===== Regular ESP32 (ESP32-WROOM) Pin Definitions =====
#elif BOARD_TYPE == 1
    // SPI Pins (default VSPI on ESP32)
    #define SPI_SCK  18    // SCK  - GPIO18 (VSPI SCK)
    #define SPI_MISO 19    // MISO - GPIO19 (VSPI MISO)
    #define SPI_MOSI 23    // MOSI - GPIO23 (VSPI MOSI)
    
    // Radio configuration pins
    #define RADIO_CS  5    // NSS/CS - GPIO5 (Chip Select)
    #define RADIO_RST 4    // RESET  - GPIO4 (Reset)
    #define RADIO_DIO0 13  // DIO0   - GPIO13 (Interrupt/Data) 
    #define RADIO_DIO1 14  // DIO1   - GPIO14 (Interrupt/Data - SX1262)
    #define RADIO_BUSY 35  // BUSY   - GPIO35 (Busy - SX1262)
    
    // I2C pins (using default ESP32 I2C pins)
    #define I2C_SDA 21     // Default SDA on GPIO21
    #define I2C_SCL 22     // Default SCL on GPIO22
    
    // GPS pins - Using Hardware Serial2 (UART2)
    #define GPS_TX_PIN 16  // RX2 on GPIO16
    #define GPS_RX_PIN 17  // TX2 on GPIO17
    #define USE_SOFTWARE_SERIAL_GPS 0  // Using Hardware Serial2 for better reliability
    
    // SD Card
    #define SD_CS_PIN 15    // GPIO5 for SD Card CS
#endif

// Common definitions for both boards
#define PCF8574_ADDRESS 0x20  // PCF8574 pins (8-bit I2C expander)
// Control pins
#define PRIMARY_RELAY_PIN 0   // P0 on PCF8574
#define BACKUP_RELAY_PIN 1    // P1 on PCF8574
#define BUZZER_PIN 2         // P2 on PCF8574
// RGB LED pins
#define LED_RED_PIN 3        // P3 on PCF8574
#define LED_GREEN_PIN 4      // P4 on PCF8574
#define LED_BLUE_PIN 5       // P5 on PCF8574 I2C expander

#endif // BOARD_CONFIG_H
