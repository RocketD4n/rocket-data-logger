#define USER_SETUP_INFO "User_Setup"

#define ILI9341_DRIVER

// ESP32-S3-N16R8 SPI pins for ILI9341 display
#define TFT_MISO 13  // ESP32-S3 GPIO13
#define TFT_MOSI 11  // ESP32-S3 GPIO11
#define TFT_SCLK 12  // ESP32-S3 GPIO12
#define TFT_CS   10  // ESP32-S3 GPIO10
#define TFT_DC    9  // ESP32-S3 GPIO9
#define TFT_RST  14  // ESP32-S3 GPIO14

// Touch screen pins for XPT2046 touch controller
#define TOUCH_CS 7   // ESP32-S3 GPIO7
#define T_IRQ    8   // ESP32-S3 GPIO8 for touch interrupt
// MISO/MOSI/SCK shared with TFT

// SD Card SPI pins
#define SD_CS    38  // ESP32-S3 GPIO38
// SD card shares MISO/MOSI/SCK with display

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

// Display driver configuration - note the reversed width/height for portrait orientation
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// Other options
#define SPI_FREQUENCY  40000000  // ESP32-S3 can handle higher SPI frequency
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

// Calibration for touch screen - will need adjustment after testing
#define TOUCH_CALIBRATION_X 240
#define TOUCH_CALIBRATION_Y 320
#define TOUCH_OFFSET_X 0
#define TOUCH_OFFSET_Y 0
