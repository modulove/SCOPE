/**
 * @file SCOPE.ino
 * @author Modulove & friends (https://github.com/modulove/)
 * @brief HAGIWO simple Arduino-based OLED Display Eurorack module
 * @version 0.1
 * @date 2024-04-30
 *
 * @copyright Copyright (c) 2024
 *
 * Connect a clock source to the CLK input and each output will
 * output triggers according to settings set in the UI OLED screen
 *
 * Encoder:
 *      short press: Toggle between menue options (dial in values with rotation)
  *
 *
 * RST: Trigger this input to reset the Display.
 *
 */

// Flag for enabling debug print to serial monitoring output.
// Note: this affects performance and locks LED 4 & 5 on HIGH.
// #define DEBUG

// Flag for reversing the encoder direction.
// #define ENCODER_REVERSED

// Flag for using module upside down
// #define PANEL_USD

#include <EEPROM.h>

// EEPROM addresses
const int EEPROM_MODE_ADDR = 0;
const int EEPROM_PARAM_SELECT_ADDR = 1;
const int EEPROM_PARAM_ADDR = 2;
const int EEPROM_PARAM1_ADDR = 3;
const int EEPROM_PARAM2_ADDR = 4;

unsigned long lastSaveTime = 0;
const unsigned long saveInterval = 30000;  // Save interval in millisec (10sec)

#include <avr/io.h>   //for fast PWM
#include "fix_fft.h"  //spectrum analyze

//OLED display setting
#include <SPI.h>               //for OLED display
#include <Adafruit_GFX.h>      //for OLED display
#include <Adafruit_SSD1306.h>  //for OLED display
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_MOSI 9
#define OLED_CLK 10
#define OLED_DC 11
#define OLED_CS 12
#define OLED_RESET 13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

//rotery encoder setting
#define ENCODER_OPTIMIZE_INTERRUPTS  //contermeasure of rotery encoder noise
#include <Encoder.h>
#ifdef ENCODER_REVERSED
Encoder myEnc(4, 2);  // 2pin, 4pin is default
#else
Encoder myEnc(2, 4);                          // 3pin, 2pin is default
#endif
float oldPosition = -999;  //rotery encoder counter
float newPosition = -999;


byte mode = 1;      //1=low freq oscilo , 2=high freq oscilo , 3 = mid freq oscilo with external trig , 4 = spectrum analyze
byte old_mode = 1;  //for initial setting when mode change.
byte param_select = 0;
byte old_param_select = 0;
byte old_param = 0;
byte old_param1 = 0;
byte old_param2 = 0;
byte param = 0;
byte param1 = 1;
bool param1_select = 0;
byte param2 = 1;
bool param2_select = 0;
int rfrs = 0;  //display refresh rate

bool trig = 0;      //external trig
bool old_trig = 0;  //external trig , OFF -> ON detect
bool old_SW = 0;    //push sw, OFF -> ON detect
bool SW = 0;        //push sw

unsigned long trigTimer = 0;
unsigned long hideTimer = 0;  //hide parameter count
unsigned long saveTimer = 0;  //save parameter count
bool hide = 0;                //1=hide,0=not hide

char data[128], im[128], cv[128];              //data and im are used for spectrum , cv is used for oscilo.
const unsigned long longPressDuration = 3000;  // 3 sec for saving

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

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 1040)
const int myBitmapallArray_LEN = 1;
const unsigned char* myBitmapallArray[1] = {
	Modulove_Logo
};

const int width = 128;
const int height = 64;

void drawAnimation() { // Boot Logo
    for (int x = 0; x <= width; x += 10) {
        display.clearDisplay();
        display.drawBitmap(0, 0, Modulove_Logo, width, height, WHITE);
        display.fillRect(x, 0, width - x, height, BLACK);
        display.display();
    }
}

void setup() {
  delay(500);  // delay for display init
  // display setting
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

  // pin mode setting
  pinMode(3, OUTPUT);        // offset voltage
  pinMode(5, INPUT_PULLUP);  // push sw
  pinMode(6, INPUT);         // input is high impedance -> no active 2pole filter, output is an active 2pole filter
  pinMode(7, INPUT);         // external trigger detect

  // fast pwm setting
  TCCR2B &= B11111000;
  TCCR2B |= B00000001;

  // fast ADC setting
  ADCSRA = ADCSRA & 0xf8;  // fast ADC *8 speed
  ADCSRA = ADCSRA | 0x04;  // fast ADC *8 speed

  //Serial.begin(57600);

  /*
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  */

  byte lastMode = EEPROM.read(EEPROM_MODE_ADDR);
  mode = 0;
  // Read the mode separately and then call loadSettings
  mode = EEPROM.read(EEPROM_MODE_ADDR);
  loadSettings();

  // Check if the last mode is valid and different from the current mode
  if (lastMode >= 1 && lastMode <= 4 && lastMode != mode) {
    // Load param1 and param2 for the last mode from EEPROM
    param1 = EEPROM.read(EEPROM_MODE_ADDR + lastMode + EEPROM_PARAM1_ADDR);
    param2 = EEPROM.read(EEPROM_MODE_ADDR + lastMode + EEPROM_PARAM2_ADDR);
  }
  // If the last mode is not valid or is the same as the current mode, load the current mode settings

  // Modify the switch case to load param1 and param2 from EEPROM based on the mode
  switch (mode) {
    case 1:
      // Load param1 and param2 for mode 1 from EEPROM
      param1 = EEPROM.read(EEPROM_MODE_ADDR + 1 + EEPROM_PARAM1_ADDR);
      param2 = EEPROM.read(EEPROM_MODE_ADDR + 1 + EEPROM_PARAM2_ADDR);
      break;

    case 2:
      // Load param1 and param2 for mode 2 from EEPROM
      param1 = EEPROM.read(EEPROM_MODE_ADDR + 2 + EEPROM_PARAM1_ADDR);
      param2 = EEPROM.read(EEPROM_MODE_ADDR + 2 + EEPROM_PARAM2_ADDR);
      break;

    case 3:
      // Load param1 and param2 for mode 3 from EEPROM
      param1 = EEPROM.read(EEPROM_MODE_ADDR + 3 + EEPROM_PARAM1_ADDR);
      param2 = EEPROM.read(EEPROM_MODE_ADDR + 3 + EEPROM_PARAM2_ADDR);
      break;

    case 4:
      // Load param1 and param2 for mode 4 from EEPROM
      param1 = EEPROM.read(EEPROM_MODE_ADDR + 4 + EEPROM_PARAM1_ADDR);
      param2 = EEPROM.read(EEPROM_MODE_ADDR + 4 + EEPROM_PARAM2_ADDR);
      break;
  }
}



void loop() {
  old_SW = SW;
  old_mode = mode;

  SW = digitalRead(5);

  //select mode by push sw
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

  //rotery encoder input
  newPosition = myEnc.read();
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
  }

  else if ((newPosition + 3) / 4 < oldPosition / 4) {
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

  //saved settings when mode change.
  if (old_mode != mode) {
    switch (mode) {
      case 1:
        //param1 = 2; //time
        //param2 = 1; //offset
        // Load param1 and param2 for mode 1 from EEPROM
        param1 = EEPROM.read(EEPROM_MODE_ADDR + 1 + EEPROM_PARAM1_ADDR);
        param2 = EEPROM.read(EEPROM_MODE_ADDR + 1 + EEPROM_PARAM2_ADDR);
        pinMode(6, INPUT);       //no active 2pole filter
        analogWrite(3, 0);       //offset = 0V
        ADCSRA = ADCSRA & 0xf8;  //fast ADC *8 speed
        ADCSRA = ADCSRA | 0x04;  //fast ADC *8 speed
        break;

      case 2:
        //param1 = 3; //time
        //param2 = 3; //offset
        // Load param1 and param2 for mode 2 from EEPROM
        param1 = EEPROM.read(EEPROM_MODE_ADDR + 2 + EEPROM_PARAM1_ADDR);
        param2 = EEPROM.read(EEPROM_MODE_ADDR + 2 + EEPROM_PARAM2_ADDR);
        analogWrite(3, 127);     //offset = 2.5V
        pinMode(6, INPUT);       //no active 2pole filter
        ADCSRA = ADCSRA & 0xf8;  //fast ADC *8 speed
        ADCSRA = ADCSRA | 0x04;  //fast ADC *8 speed
        break;

      case 3:
        //param1 = 2; //time
        // Load param1  for mode 3 from EEPROM
        param1 = EEPROM.read(EEPROM_MODE_ADDR + 3 + EEPROM_PARAM1_ADDR);
        analogWrite(3, 127);     //offset = 2.5V
        pinMode(6, INPUT);       //no active 2pole filter
        ADCSRA = ADCSRA & 0xf8;  //fast ADC *8 speed
        ADCSRA = ADCSRA | 0x04;  //fast ADC *8 speed
        break;

      case 4:
        //param1 = 2; //high freq amp
        //param2 = 3; //noise filter
        // Load param1 and param2 for mode 4 from EEPROM
        param1 = EEPROM.read(EEPROM_MODE_ADDR + 4 + EEPROM_PARAM1_ADDR);
        param2 = EEPROM.read(EEPROM_MODE_ADDR + 4 + EEPROM_PARAM2_ADDR);
        analogWrite(3, 127);     //offset = 2.5V
        pinMode(6, OUTPUT);      //active 2pole filter
        digitalWrite(6, LOW);    //active 2pole filter
        ADCSRA = ADCSRA & 0xf8;  //standard ADC speed
        ADCSRA = ADCSRA | 0x07;  //standard ADC speed
        break;
    }
  }

  //OLED parameter hide while no operation
  if (hideTimer + 5000 >= millis()) {  //
    hide = 1;
  } else {
    hide = 0;
  }

  //LFO mode--------------------------------------------------------------------------------------------
  if (mode == 1) {
    param = constrain(param, 1, 3);
    param1 = constrain(param1, 1, 8);
    param2 = constrain(param2, 1, 8);

    display.clearDisplay();

    //store data
    for (int i = 126 / (9 - param1); i >= 0; i--) {
      display.drawLine(127 - (i * (9 - param1)), 63 - cv[i] - (param2 - 1) * 4, 127 - (i + 1) * (9 - param1), 63 - cv[(i + 1)] - (param2 - 1) * 4, WHITE);  //right to left
      cv[i + 1] = cv[i];
      if (i == 0) {
        cv[0] = analogRead(0) / 16;
      }
    }

    //display
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

  //wave mode---------------------------------------------------------------------------------------------
  else if (mode == 2) {
    param = constrain(param, 1, 3);
    param1 = constrain(param1, 1, 8);
    param2 = constrain(param2, 1, 6);

    //store data
    if (param1 > 5) {  //for mid frequency
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
    }

    else if (param1 <= 5) {  //for high frequency
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

    //display
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

  //shot mode---------------------------------------------------------------------------------------------
  else if (mode == 3) {
    param = constrain(param, 1, 2);
    param1 = constrain(param1, 1, 4);
    old_trig = trig;
    trig = digitalRead(7);

    //    trig detect
    if (old_trig == 0 && trig == 1) {
      for (int i = 10; i <= 127; i++) {
        cv[i] = analogRead(0) / 16;
        delayMicroseconds(100000 * param1);  //100000 is magic number
      }
      for (int i = 0; i < 10; i++) {
        cv[i] = 32;
      }
    }

    display.clearDisplay();
    for (int i = 126; i >= 1; i--) {
      display.drawLine(i, 63 - cv[i], (i + 1), 63 - cv[(i + 1)], WHITE);
    }

    //display
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

  //spectrum analyze mode---------------------------------------------------------------------------------------------
  else if (mode == 4) {
    param = constrain(param, 1, 3);
    param1 = constrain(param1, 1, 4);  //high freq sence
    param2 = constrain(param2, 1, 8);  //noise filter

    for (byte i = 0; i < 128; i++) {
      int spec = analogRead(0);
      data[i] = spec / 4 - 128;
      im[i] = 0;
    };
    fix_fft(data, im, 7, 0);
    display.clearDisplay();
    for (byte i = 0; i < 64; i++) {
      int level = sqrt(data[i] * data[i] + im[i] * im[i]);
      ;
      if (level >= param2) {
        display.fillRect(i * 2, 63 - (level + i * (param1 - 1) / 8), 2, (level + i * (param1 - 1) / 8), WHITE);  // i * (param1 - 1) / 8 is high freq amp
      }
    }

    //display
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


  // Save settings if they have changed
  if (old_mode != mode || old_param_select != param_select || old_param != param || old_param1 != param1 || old_param2 != param2) {
    saveSettings();
    old_mode = mode;
    old_param_select = param_select;
    old_param = param;
    old_param1 = param1;
    old_param2 = param2;
  }


  // Save settings if 30 seconds have passed
  unsigned long currentTime = millis();
  if (currentTime - lastSaveTime >= saveInterval) {
    saveSettings();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4);
    display.print("AutoSaved");
    delay(300);

    lastSaveTime = currentTime;
  }

  display.display();
};

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

  // Save the last mode to EEPROM
  EEPROM.write(EEPROM_MODE_ADDR, mode);
}


void loadSettings() {
  int eepromAddr = EEPROM_MODE_ADDR + (mode - 1) * 4;
  param_select = EEPROM.read(eepromAddr + EEPROM_PARAM_SELECT_ADDR);
  param = EEPROM.read(eepromAddr + EEPROM_PARAM_ADDR);
  param1 = EEPROM.read(eepromAddr + EEPROM_PARAM1_ADDR);
  param2 = EEPROM.read(eepromAddr + EEPROM_PARAM2_ADDR);
}

void displaySavedState(const char* paramName, int paramValue, int eepromAddr) {
  int eepromValue = EEPROM.read(eepromAddr);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4);
  display.print("Saved: ");
  display.setCursor(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4 + 16);
  display.print(paramName);
  display.setCursor(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4 + 32);
  display.print("Value: ");
  display.print(paramValue);
  display.setCursor(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4 + 48);
  display.print("EEPROM Addr: ");
  display.print(eepromAddr);
  display.setCursor(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4 + 64);
  display.print("EEPROM Value: ");
  display.print(eepromValue);
  delay(100);
}
