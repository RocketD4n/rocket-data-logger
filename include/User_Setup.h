#define USER_SETUP_INFO "User_Setup"

// Basic configuration for ILI9341 on ESP32-S3
#define ILI9341_DRIVER

// Use the pins that are actually connected
#define TFT_MISO 13
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST  14  // Enable reset pin

// Define LED backlight pin - connect this to 3.3V or a GPIO pin
// Note: We're connecting backlight directly to 3.3V, so this is just for reference
#define TFT_BL   15  // LED backlight control pin

// ESP32-S3 specific settings
#define USE_HSPI_PORT // Use HSPI port for ESP32-S3

// SPI settings - use very conservative settings for reliability
#define SPI_FREQUENCY       1000000  // 1MHz for more reliable communication
#define SPI_READ_FREQUENCY  1000000  // Match the write frequency
#define SPI_TOUCH_FREQUENCY 1000000  // Touch screen SPI frequency

// Disable touch for now to simplify testing
// #define TOUCH_CS 7

// Disable SD card for now to simplify testing
// #define SD_CS    38

// Font settings
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define SMOOTH_FONT

// Color depth has to be 16 bits
#define COLOR_DEPTH 16

// Display dimensions
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// Processor specific options
#define ESP32_DMA

// SPI settings - use a very low frequency for more reliable communication during testing
#define SPI_FREQUENCY       4000000   // 4MHz for more reliable communication
#define SPI_READ_FREQUENCY  4000000   // Match the write frequency
#define SPI_TOUCH_FREQUENCY 2500000   // Touch screen SPI frequency

// Calibration for touch screen - will need adjustment after testing
#define TOUCH_CALIBRATION_X 240
#define TOUCH_CALIBRATION_Y 320
#define TOUCH_OFFSET_X 0
#define TOUCH_OFFSET_Y 0

// Additional settings to make initialization more robust
#define TFT_SPI_MODE SPI_MODE0  // Most displays use Mode 0
#define SUPPORT_TRANSACTIONS
