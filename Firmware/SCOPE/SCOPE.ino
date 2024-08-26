#include <EEPROM.h>
#include <avr/io.h>   // For fast PWM
#include "fix_fft.h"  // Spectrum analysis

// OLED display setting
#include <SPI.h>               // For OLED display
#include <Adafruit_GFX.h>      // For OLED display
#include <Adafruit_SSD1306.h>  // For OLED display
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_MOSI 9
#define OLED_CLK 10
#define OLED_DC 11
#define OLED_CS 12
#define OLED_RESET 13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// EEPROM addresses
#define ENCODER_DIR_ADDR 0        // EEPROM address to store the encoder direction
#define MENUTIMER_DIR_ADDR 2      // EEPROM address to store the menuTimer direction
const int EEPROM_MODE_ADDR = 4;
const int EEPROM_PARAM_SELECT_ADDR = 5;
const int EEPROM_PARAM_ADDR = 6;
const int EEPROM_PARAM1_ADDR = 7;
const int EEPROM_PARAM2_ADDR = 8;

// Rotary encoder setting
#define ENCODER_OPTIMIZE_INTERRUPTS  // Countermeasure for rotary encoder noise
#include <Encoder.h>
Encoder myEnc(2, 4);  // Rotary encoder digitalRead pin
float oldPosition = -999;  // Rotary encoder counter
float newPosition = -999;

byte mode = 1;       // 1=low freq oscilo, 2=high freq oscilo, 3=mid freq oscilo with external trig, 4=spectrum analyze
byte old_mode = 1;   // For initial setting when mode change
byte param_select = 0;
byte old_param_select = 0;
byte param = 0;
byte old_param = 0;
byte param1 = 1;
bool param1_select = 0;
byte old_param1 = 0;
byte param2 = 1;
bool param2_select = 0;
byte old_param2 = 0;
int rfrs = 0;  // Display refresh rate

bool trig = 0;      // External trigger
bool old_trig = 0;  // External trigger, OFF -> ON detect
bool old_SW = 0;    // Push switch, OFF -> ON detect
bool SW = 0;        // Push switch

unsigned long trigTimer = 0;
unsigned long hideTimer = 0;  // Hide parameter count
unsigned long saveTimer = 0;  // Save parameter count
bool hide = 0;                // 1=hide, 0=not hide

char data[128], im[128], cv[128];  // Data and im are used for spectrum, cv is used for oscilo

// New global configs
bool encoderPressed = false;
bool secretMenuActive = false;
unsigned long enterPressStartTime = 0;  // Timer for entering the secret menu
unsigned long exitPressStartTime = 0;   // Timer for exiting the secret menu
byte secretMenuOption = 1;              // 1 for encoderDirection, 2 for menuTimer
unsigned int menuTimer = 5;             // Default value for menuTimer, set to 5
int encoderDirection = 1;               // 1 for normal, -1 for reversed
unsigned long lastSaveTime = 0;
const unsigned long saveInterval = 30000;  // Save interval 

// 'Modulove_Logo', 128x64px // Boot Logo
const unsigned char Modulove_Logo [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xe0, 0x3c, 0x00, 0x07, 0x00, 0x02, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0xa0, 0x6c, 0x00, 0x0d, 0x80, 0x06, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0x20, 0xc8, 0x00, 0x0d, 0x00, 0x04, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0x60, 0x88, 0x00, 0x1b, 0x00, 0x0c, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x02, 0x41, 0x98, 0x00, 0x13, 0x00, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x06, 0xc3, 0x30, 0x00, 0x36, 0x00, 0x19, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x07, 0x83, 0x60, 0x00, 0x24, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x07, 0x06, 0x40, 0x00, 0x6c, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0e, 0x04, 0x80, 0x00, 0x78, 0x00, 0x32, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0e, 0x0d, 0x80, 0x00, 0x70, 0x00, 0x36, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0c, 0x0f, 0x00, 0x00, 0xf0, 0x00, 0x2c, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0x7f, 0x18, 0x1e, 0x0e, 0x02, 0xe4, 0x08, 0x2c, 0x38, 0x41, 0xcc, 0x60, 0x00, 0x00, 
	0x00, 0x03, 0xfc, 0x18, 0x18, 0x1f, 0x0e, 0xc4, 0x18, 0x78, 0x7c, 0xc3, 0xc8, 0xc0, 0x00, 0x00, 
	0x00, 0x0e, 0x00, 0x38, 0x30, 0x19, 0xfd, 0xc4, 0x10, 0x70, 0xc7, 0xc2, 0x7f, 0x80, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x6d, 0xe0, 0x1f, 0xf9, 0x8c, 0x30, 0x60, 0x46, 0x86, 0x3e, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x00, 0xc7, 0xe0, 0x01, 0x61, 0x88, 0x30, 0xe0, 0x7c, 0x84, 0x20, 0x00, 0x38, 0x00, 
	0x00, 0x60, 0x01, 0x80, 0x40, 0x61, 0x43, 0x18, 0x60, 0xc3, 0x3d, 0x8c, 0x20, 0x01, 0xff, 0x00, 
	0x00, 0x40, 0x03, 0x00, 0x40, 0xc3, 0xc3, 0x18, 0x60, 0x82, 0x05, 0x88, 0x60, 0x03, 0x08, 0x00, 
	0x00, 0xc0, 0x02, 0x00, 0xc0, 0x83, 0x83, 0x30, 0xc1, 0x86, 0x0d, 0x88, 0x40, 0x06, 0x00, 0x00, 
	0x00, 0x80, 0x06, 0x00, 0xc1, 0x83, 0x86, 0x30, 0xc3, 0x86, 0x0d, 0x98, 0x40, 0x04, 0x00, 0x00, 
	0x00, 0xc0, 0x0c, 0x00, 0xc1, 0x07, 0x86, 0x71, 0xc3, 0x0e, 0x09, 0x90, 0x40, 0x04, 0x00, 0x00, 
	0x00, 0xc0, 0x38, 0x00, 0xc3, 0x07, 0x8e, 0x71, 0x87, 0x0e, 0x19, 0xb0, 0x40, 0x04, 0x00, 0x00, 
	0x00, 0xc0, 0x60, 0x00, 0xc7, 0x0d, 0x0e, 0xf3, 0x8d, 0x1e, 0x11, 0xb0, 0x60, 0x0c, 0x00, 0x00, 
	0x00, 0x60, 0xc0, 0x00, 0xcd, 0x99, 0x1e, 0xf2, 0x99, 0x32, 0x30, 0xa0, 0x30, 0x18, 0x00, 0x00, 
	0x00, 0x3f, 0x80, 0x00, 0x79, 0xf1, 0xf7, 0xbe, 0xf1, 0xe3, 0xe0, 0xe0, 0x3f, 0xf0, 0x00, 0x00, 
	0x00, 0x0e, 0x00, 0x00, 0x30, 0xe0, 0xc3, 0x18, 0x60, 0xc0, 0xc0, 0x40, 0x0f, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// (Total bytes used to store images in PROGMEM = 1040)

const int width = 128;
const int height = 64;

void drawAnimation() {  // Boot Logo
    for (int x = 0; x <= width; x += 10) {
        display.clearDisplay();
        display.drawBitmap(0, 0, Modulove_Logo, width, height, WHITE);
        display.fillRect(x, 0, width - x, height, BLACK);
        display.display();
    }
}

void setup() {
    // Load the saved encoder direction and menuTimer from EEPROM
    EEPROM.get(ENCODER_DIR_ADDR, encoderDirection);
    delay(10);
    if (encoderDirection != 1 && encoderDirection != -1) {
        encoderDirection = 1;  // Default to normal direction if EEPROM contains invalid data
    }

    EEPROM.get(MENUTIMER_DIR_ADDR, menuTimer);
    delay(10);
    if (menuTimer < 1 || menuTimer > 30) {
        menuTimer = 5;  // Default to 5 if menuTimer is invalid
    }

    // Display setting
    display.begin(SSD1306_SWITCHCAPVCC);
    #ifdef PANEL_USD
    display.setRotation(2);  // 180 degree rotation for upside-down use
    #else
    display.setRotation(0);  // Normal orientation
    #endif

    // Boot Logo
    drawAnimation();
    delay(3000);

    display.clearDisplay();
    display.setTextSize(0);
    display.setTextColor(WHITE);
    analogReference(DEFAULT);

    // Pin mode setting
    pinMode(3, OUTPUT);        // Offset voltage
    pinMode(5, INPUT_PULLUP);  // Push switch
    pinMode(6, INPUT);         // Input is high impedance -> no active 2-pole filter, output is an active 2-pole filter
    pinMode(7, INPUT);         // External trigger detect

    // Fast PWM setting
    TCCR2B &= B11111000;
    TCCR2B |= B00000001;

    // Fast ADC setting
    ADCSRA = ADCSRA & 0xf8;  // Fast ADC *8 speed
    ADCSRA = ADCSRA | 0x04;  // Fast ADC *8 speed

    // Load last mode settings
    byte lastMode = EEPROM.read(EEPROM_MODE_ADDR);
    mode = 0;
    mode = EEPROM.read(EEPROM_MODE_ADDR);
    loadSettings();

    if (lastMode >= 1 && lastMode <= 4 && lastMode != mode) {
        param1 = EEPROM.read(EEPROM_MODE_ADDR + lastMode + EEPROM_PARAM1_ADDR);
        param2 = EEPROM.read(EEPROM_MODE_ADDR + lastMode + EEPROM_PARAM2_ADDR);
    }

    switch (mode) {
        case 1:
            param1 = EEPROM.read(EEPROM_MODE_ADDR + 1 + EEPROM_PARAM1_ADDR);
            param2 = EEPROM.read(EEPROM_MODE_ADDR + 1 + EEPROM_PARAM2_ADDR);
            break;

        case 2:
            param1 = EEPROM.read(EEPROM_MODE_ADDR + 2 + EEPROM_PARAM1_ADDR);
            param2 = EEPROM.read(EEPROM_MODE_ADDR + 2 + EEPROM_PARAM2_ADDR);
            break;

        case 3:
            param1 = EEPROM.read(EEPROM_MODE_ADDR + 3 + EEPROM_PARAM1_ADDR);
            param2 = EEPROM.read(EEPROM_MODE_ADDR + 3 + EEPROM_PARAM2_ADDR);
            break;

        case 4:
            param1 = EEPROM.read(EEPROM_MODE_ADDR + 4 + EEPROM_PARAM1_ADDR);
            param2 = EEPROM.read(EEPROM_MODE_ADDR + 4 + EEPROM_PARAM2_ADDR);
            break;
    }
}

void loop() {
    old_SW = SW;
    old_mode = mode;
    SW = digitalRead(5);

    if (SW == LOW && !encoderPressed && !secretMenuActive) {
        encoderPressed = true;
        enterPressStartTime = millis();  // Start timing the press
    } else if (SW == HIGH && encoderPressed && !secretMenuActive) {
        encoderPressed = false;
    }

    if (encoderPressed && millis() - enterPressStartTime >= 3000 && !secretMenuActive) {
        secretMenuActive = true;
        enterPressStartTime = 0;
        encoderPressed = false;  // Reset press state
        secretMenuOption = 1;    // Reset to first option
        oldPosition = newPosition = myEnc.read();  // Reset encoder position
    }

    if (secretMenuActive) {
        secretMenu();
    } else {
        // Start of main logic
        newPosition = encoderDirection * myEnc.read();

        // Select mode by push switch
        if (old_SW == 0 && SW == 1 && param_select == param) {
            param_select = 0;
            hideTimer = millis();
        } else if (old_SW == 0 && SW == 1 && param == 1) {
            param_select = param;
            hideTimer = millis();
        } else if (old_SW == 0 && SW == 1 && param == 2) {
            param_select = param;
            hideTimer = millis();
        } else if (old_SW == 0 && SW == 1 && param == 3) {
            param_select = param;
            hideTimer = millis();
        }
        mode = constrain(mode, 1, 4);
        param = constrain(param, 1, 3);

        // Rotary encoder input
        newPosition = encoderDirection * myEnc.read();

        if ((newPosition - 3) / 4 > oldPosition / 4) {
            oldPosition = newPosition;
            hideTimer = millis();
            switch (param_select) {
                case 0:
                    param--;
                    break;

                case 1:
                    mode--;
                    break;

                case 2:
                    param1--;
                    break;

                case 3:
                    param2--;
                    break;
            }
        } else if ((newPosition + 3) / 4 < oldPosition / 4) {
            oldPosition = newPosition;
            hideTimer = millis();
            switch (param_select) {
                case 0:
                    param++;
                    break;

                case 1:
                    mode++;
                    break;

                case 2:
                    param1++;
                    break;

                case 3:
                    param2++;
                    break;
            }
        }

        // Initial setting when mode change
        if (old_mode != mode) {
            switch (mode) {
                case 1:
                    param1 = 2;  // Time
                    param2 = 1;  // Offset
                    pinMode(6, INPUT);  // No active 2-pole filter
                    analogWrite(3, 0);  // Offset = 0V
                    ADCSRA = ADCSRA & 0xf8;  // Fast ADC *8 speed
                    ADCSRA = ADCSRA | 0x04;  // Fast ADC *8 speed
                    break;

                case 2:
                    param1 = 3;  // Time
                    param2 = 3;  // Offset
                    analogWrite(3, 127);  // Offset = 2.5V
                    pinMode(6, INPUT);  // No active 2-pole filter
                    ADCSRA = ADCSRA & 0xf8;  // Fast ADC *8 speed
                    ADCSRA = ADCSRA | 0x04;  // Fast ADC *8 speed
                    break;

                case 3:
                    param1 = 2;  // Time
                    analogWrite(3, 127);  // Offset = 2.5V
                    pinMode(6, INPUT);  // No active 2-pole filter
                    ADCSRA = ADCSRA & 0xf8;  // Fast ADC *8 speed
                    ADCSRA = ADCSRA | 0x04;  // Fast ADC *8 speed
                    break;

                case 4:
                    param1 = 2;  // High freq amp
                    param2 = 3;  // Noise filter
                    analogWrite(3, 127);  // Offset = 2.5V
                    pinMode(6, OUTPUT);  // Active 2-pole filter
                    digitalWrite(6, LOW);  // Active 2-pole filter
                    ADCSRA = ADCSRA & 0xf8;  // Standard ADC speed
                    ADCSRA = ADCSRA | 0x07;  // Standard ADC speed
                    break;
            }
        }

        // OLED parameter hide while no operation
        if (hideTimer + (menuTimer * 1000) >= millis()) {
            hide = 1;
        } else {
            hide = 0;
        }

        if (mode == 1) {
            lfoMode();
        } else if (mode == 2) {
            waveMode();
        } else if (mode == 3) {
            shotMode();
        } else if (mode == 4) {
            spectrumMode();
        }

        display.display();
    }
}

void lfoMode() {
    param = constrain(param, 1, 3);
    param1 = constrain(param1, 1, 8);
    param2 = constrain(param2, 1, 8);

    display.clearDisplay();

    // Store data
    for (int i = 126 / (9 - param1); i >= 0; i--) {
        display.drawLine(127 - (i * (9 - param1)), 63 - cv[i] - (param2 - 1) * 4, 127 - (i + 1) * (9 - param1), 63 - cv[(i + 1)] - (param2 - 1) * 4, WHITE);  // Right to left
        cv[i + 1] = cv[i];
        if (i == 0) {
            cv[0] = analogRead(0) / 16;
        }
    }

    // Display
    if (hide == 1) {
        display.drawLine((param - 1) * 42, 8, (param - 1) * 42 + 36, 8, WHITE);

        display.setTextColor(WHITE);
        if (param_select == 1) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(0, 0);
        display.print("Mode:");
        display.setCursor(30, 0);
        display.print(mode);

        display.setTextColor(WHITE);
        if (param_select == 2) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(42, 0);
        display.print("Time:");
        display.setCursor(72, 0);
        display.print(param1);

        display.setTextColor(WHITE);
        if (param_select == 3) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(84, 0);
        display.print("Offs:");
        display.setCursor(114, 0);
        display.print(param2);
    }
}

void waveMode() {
    param = constrain(param, 1, 3);
    param1 = constrain(param1, 1, 8);
    param2 = constrain(param2, 1, 6);

    // Store data
    if (param1 > 5) {  // For mid frequency
        for (int i = 127; i >= 0; i--) {
            cv[i] = analogRead(0) / 16;
            delayMicroseconds((param1 - 5) * 20);
        }
        rfrs++;
        if (rfrs >= (param2 - 1) * 2) {
            rfrs = 0;
            display.clearDisplay();
            for (int i = 127; i >= 1; i--) {
                display.drawLine(127 - i, 63 - cv[i - 1], 127 - (i + 1), 63 - cv[(i)], WHITE);
            }
        }
    } else if (param1 <= 5) {  // For high frequency
        for (int i = 127 / (6 - param1); i >= 0; i--) {
            cv[i] = analogRead(0) / 16;
        }
        rfrs++;
        if (rfrs >= (param2 - 1) * 2) {
            rfrs = 0;
            display.clearDisplay();
            for (int i = 127 / (6 - param1); i >= 1; i--) {
                display.drawLine(127 - i * (6 - param1), 63 - cv[i - 1], 127 - (i + 1) * (6 - param1), 63 - cv[(i)], WHITE);
            }
        }
    }

    // Display
    if (hide == 1) {
        display.drawLine((param - 1) * 42, 8, (param - 1) * 42 + 36, 8, WHITE);
        display.setTextColor(WHITE);
        if (param_select == 1) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(0, 0);
        display.print("Mode:");
        display.setCursor(30, 0);
        display.print(mode);

        display.setTextColor(WHITE);
        if (param_select == 2) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(42, 0);
        display.print("Time:");
        display.setCursor(72, 0);
        display.print(param1);

        display.setTextColor(WHITE);
        if (param_select == 3) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(84, 0);
        display.print("Rfrs:");
        display.setCursor(114, 0);
        display.print(param2);
    }
}

void shotMode() {
    param = constrain(param, 1, 2);
    param1 = constrain(param1, 1, 4);
    old_trig = trig;
    trig = digitalRead(7);

    // Trigger detect
    if (old_trig == 0 && trig == 1) {
        for (int i = 10; i <= 127; i++) {
            cv[i] = analogRead(0) / 16;
            delayMicroseconds(100000 * param1);  // 100000 is a magic number
        }
        for (int i = 0; i < 10; i++) {
            cv[i] = 32;
        }
    }

    display.clearDisplay();
    for (int i = 126; i >= 1; i--) {
        display.drawLine(i, 63 - cv[i], (i + 1), 63 - cv[(i + 1)], WHITE);
    }

    // Display
    if (hide == 1) {
        display.drawLine((param - 1) * 42, 8, (param - 1) * 42 + 36, 8, WHITE);
        display.setTextColor(WHITE);
        if (param_select == 1) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(0, 0);
        display.print("Mode:");
        display.setCursor(30, 0);
        display.print(mode);

        display.setTextColor(WHITE);
        if (param_select == 2) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(42, 0);
        display.print("Time:");
        display.setCursor(72, 0);
        display.print(param1);
        display.setTextColor(WHITE);
        if (trig == 1) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(84, 0);
        display.print("TRIG");
    }
}

void spectrumMode() {
    param = constrain(param, 1, 3);
    param1 = constrain(param1, 1, 4);  // High freq sense
    param2 = constrain(param2, 1, 8);  // Noise filter

    for (byte i = 0; i < 128; i++) {
        int spec = analogRead(0);
        data[i] = spec / 4 - 128;
        im[i] = 0;
    }
    fix_fft(data, im, 7, 0);
    display.clearDisplay();
    for (byte i = 0; i < 64; i++) {
        int level = sqrt(data[i] * data[i] + im[i] * im[i]);
        if (level >= param2) {
            display.fillRect(i * 2, 63 - (level + i * (param1 - 1) / 8), 2, (level + i * (param1 - 1) / 8), WHITE);  // i * (param1 - 1) / 8 is high freq amp
        }
    }

    // Display
    if (hide == 1) {
        display.drawLine((param - 1) * 42, 8, (param - 1) * 42 + 36, 8, WHITE);
        display.setTextColor(WHITE);
        if (param_select == 1) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(0, 0);
        display.print("Mode:");
        display.setCursor(30, 0);
        display.print(mode);

        display.setTextColor(WHITE);
        if (param_select == 2) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(42, 0);
        display.print("High:");
        display.setCursor(72, 0);
        display.print(param1);

        display.setTextColor(WHITE);
        if (param_select == 3) {
            display.setTextColor(BLACK, WHITE);
        }
        display.setCursor(84, 0);
        display.print("Filt:");
        display.setCursor(114, 0);
        display.print(param2);
    }
}

void saveSettings() {
    int eepromAddr = EEPROM_MODE_ADDR + (mode - 1) * 4;  // Adjust the address based on the mode

    if (EEPROM.read(eepromAddr + EEPROM_PARAM_SELECT_ADDR) != param_select) {
        EEPROM.write(eepromAddr + EEPROM_PARAM_SELECT_ADDR, param_select);
    }
    if (EEPROM.read(eepromAddr + EEPROM_PARAM_ADDR) != param) {
        EEPROM.write(eepromAddr + EEPROM_PARAM_ADDR, param);
    }
    if (EEPROM.read(eepromAddr + EEPROM_PARAM1_ADDR) != param1) {
        EEPROM.write(eepromAddr + EEPROM_PARAM1_ADDR, param1);
    }
    if (EEPROM.read(eepromAddr + EEPROM_PARAM2_ADDR) != param2) {
        EEPROM.write(eepromAddr + EEPROM_PARAM2_ADDR, param2);
    }

    EEPROM.write(EEPROM_MODE_ADDR, mode);
}

void loadSettings() {
    int eepromAddr = EEPROM_MODE_ADDR + (mode - 1) * 4;
    param_select = EEPROM.read(eepromAddr + EEPROM_PARAM_SELECT_ADDR);
    param = EEPROM.read(eepromAddr + EEPROM_PARAM_ADDR);
    param1 = EEPROM.read(eepromAddr + EEPROM_PARAM1_ADDR);
    param2 = EEPROM.read(eepromAddr + EEPROM_PARAM2_ADDR);
}

void secretMenu() {
    int newDirection = encoderDirection;
    newPosition = myEnc.read();

    // Check if the encoder was turned
    if (newPosition > (oldPosition + 3)) {
        oldPosition = newPosition;
        if (secretMenuOption == 1) {
            newDirection = 1;  // Normal direction
        } else if (secretMenuOption == 2) {
            menuTimer += encoderDirection;
            menuTimer = constrain(menuTimer, 1, 60);
        }
    } else if (newPosition < (oldPosition - 3)) {
        oldPosition = newPosition;
        if (secretMenuOption == 1) {
            newDirection = -1;  // Reversed direction
        } else if (secretMenuOption == 2) {
            menuTimer -= encoderDirection;
            menuTimer = constrain(menuTimer, 1, 60);
        }
    }

    // Update encoder direction if changed
    if (newDirection != encoderDirection) {
        encoderDirection = newDirection;
    }

    // Display the secret menu
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("             MENU");
    display.setTextColor(secretMenuOption == 1 ? BLACK : WHITE, secretMenuOption == 1 ? WHITE : BLACK);
    display.println("1. Enc. Direction:");
    display.setTextColor(WHITE);
    display.println(encoderDirection == 1 ? "Normal" : "Reversed");
    display.println("");
    display.setTextColor(secretMenuOption == 2 ? BLACK : WHITE, secretMenuOption == 2 ? WHITE : BLACK);
    display.println("2. Menu Timer:");
    display.setTextColor(WHITE);

    display.println(menuTimer);
    display.display();

    // Handle encoder button press within secret menu
    if (SW == LOW && !encoderPressed) {
        encoderPressed = true;
        exitPressStartTime = millis();  // Start timing the press for exit
    } else if (SW == HIGH && encoderPressed) {
        if (millis() - exitPressStartTime < 3000) {
            secretMenuOption = secretMenuOption == 1 ? 2 : 1;
        }
        encoderPressed = false;
        exitPressStartTime = 0;
    }

    if (encoderPressed && millis() - exitPressStartTime >= 3000) {
        EEPROM.put(ENCODER_DIR_ADDR, encoderDirection);  // Save the new direction to EEPROM
        delay(10);
        EEPROM.put(MENUTIMER_DIR_ADDR, menuTimer);  // Save the new direction to EEPROM
        delay(10);
        secretMenuActive = false;  // Exit the secret menu
        display.clearDisplay();
        display.display();  // Clear the screen or return to the main menu display
        encoderPressed = false;
        exitPressStartTime = 0;
        delay(500);  // Debounce delay
    }
}
