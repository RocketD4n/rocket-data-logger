#define USER_SETUP_INFO "User_Setup"

#define ILI9341_DRIVER

#define TFT_MISO 19  // ESP32 GPIO19
#define TFT_MOSI 23  // ESP32 GPIO23
#define TFT_SCLK 18  // ESP32 GPIO18
#define TFT_CS   15  // ESP32 GPIO15
#define TFT_DC    2  // ESP32 GPIO2
#define TFT_RST   4  // ESP32 GPIO4

// Touch screen pins for ILI9341 with XPT2046 touch controller
#define TOUCH_CS 21     // ESP32 GPIO21
// MISO/MOSI/SCK shared with TFT

// Enable touch support
#define TOUCH_DRIVER XPT2046_DRIVER

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters

#define SMOOTH_FONT

// Color depth has to be 16 bits
#define COLOR_DEPTH 16

// Processor specific options
#define ESP32_DMA
// Removed parallel mode to enable touch support

// Display driver configuration
#define TFT_WIDTH  320
#define TFT_HEIGHT 240

// Other options
#define SPI_FREQUENCY  27000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

// Calibration for touch screen
#define TOUCH_CALIBRATION_X 320
#define TOUCH_CALIBRATION_Y 240
#define TOUCH_OFFSET_X 0
#define TOUCH_OFFSET_Y 0
