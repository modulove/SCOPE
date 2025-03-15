/**
 * @file SCOPE.ino
 * @author Modulove & friends (https://github.com/modulove/)
 * @brief HAGIWO simple Arduino-based OLED Display Eurorack module
 * @version 1
 * @date 2025-14-03
 *
 * @copyright Copyright (c) 2025
 *
 * Optimized Oscilloscope/Spectrum Analyzer
 * Improved version with better memory usage and code structure
 *
 * Encoder:
 *          short press: Toggle between menue options (dial in values with rotation)
 *          medium press 1-2 seconds: Save current mode parameters to EEPROM and remember last mode
 *          long press >3 seconds: Enter & Exit Global Settings Menu
 *
 * Global Settings Menu:
 *                      Encoder Driection: Normal or Reverse
 *                      Menu Timer: 1-60 seconds (hides menu when not in use)
 *
 */

// Flag for using module upside down
// #define PANEL_USD

#include <EEPROM.h>
#include <avr/io.h>   // For fast PWM
#include "fix_fft.h"  // Spectrum analysis
#include <Encoder.h>

// OLED display setting
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Display configuration
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_MOSI 9
#define OLED_CLK 10
#define OLED_DC 11
#define OLED_CS 12
#define OLED_RESET 13

// Pin definitions
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 4
#define BUTTON_PIN 5
#define FILTER_PIN 6
#define TRIGGER_PIN 7
#define OFFSET_PIN 3
#define ANALOG_INPUT_PIN 0

// EEPROM addresses
#define ENCODER_DIR_ADDR 0        // EEPROM address to store the encoder direction
#define MENUTIMER_DIR_ADDR 2      // EEPROM address to store the menuTimer direction
const int EEPROM_MODE_ADDR = 4;
const int EEPROM_PARAM_SELECT_ADDR = 5;
const int EEPROM_PARAM_ADDR = 6;
const int EEPROM_PARAM1_ADDR = 7;
const int EEPROM_PARAM2_ADDR = 8;

// Mode definitions
#define MODE_LFO 1                // Low frequency oscilloscope
#define MODE_WAVE 2               // High frequency oscilloscope
#define MODE_SHOT 3               // Mid freq oscilloscope with external trig
#define MODE_SPECTRUM 4           // Spectrum analyzer

// Initialize the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Initialize the rotary encoder
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// 'Modulove_Logo', 128x30px // Optimized Boot Logo
const unsigned char Modulove_Logo [] PROGMEM = {
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
	0x00, 0x0e, 0x00, 0x00, 0x30, 0xe0, 0xc3, 0x18, 0x60, 0xc0, 0xc0, 0x40, 0x0f, 0xe0, 0x00, 0x00
};

// Logo dimensions
const int logoWidth = 128;  // Keep full width
const int logoHeight = 30;  // Reduced height
const int yOffset = 14;     // Position in the original bitmap where content starts

// Global variables
uint8_t mode = 1;       // 1=low freq oscilo, 2=high freq oscilo, 3=mid freq oscilo with external trig, 4=spectrum analyze
uint8_t old_mode = 1;   // For initial setting when mode change
uint8_t param_select = 0;
uint8_t param = 1;
uint8_t param1 = 2;
uint8_t param2 = 1;
bool trig = 0;      // External trigger
bool old_trig = 0;  // External trigger, OFF -> ON detect
bool old_SW = 0;    // Push switch, OFF -> ON detect
bool SW = 0;        // Push switch
unsigned long hideTimer = 0;  // Hide parameter count
unsigned long saveTimer = 0;  // Save parameter count
bool hide = 0;                // 1=hide, 0=not hide
int rfrs = 0;  // Display refresh rate

// For rotary encoder
float oldPosition = -999;  // Rotary encoder counter
float newPosition = -999;

// Secret menu variables
bool encoderPressed = false;
bool secretMenuActive = false;
unsigned long enterPressStartTime = 0;  // Timer for entering the secret menu
unsigned long exitPressStartTime = 0;   // Timer for exiting the secret menu
byte secretMenuOption = 1;              // 1 for encoderDirection, 2 for menuTimer
unsigned int menuTimer = 5;             // Default value for menuTimer, set to 5
int encoderDirection = 1;               // 1 for normal, -1 for reversed
unsigned long lastSaveTime = 0;

// Data buffers (using union to share memory)
union {
  struct {
    int8_t real[128];               // FFT real component
    int8_t imag[128];               // FFT imaginary component
  } spectrum;
  uint8_t waveform[128];            // Oscilloscope waveform
} buffer;

// Function declarations
void drawBootAnimation();
void setupMode(uint8_t mode);
void runLFOMode(bool showParams);
void runWaveMode(bool showParams);
void runShotMode(bool showParams);
void runSpectrumMode(bool showParams);
void drawParameterBar(bool showParams);
void handleSecretMenu();
void saveSettings();
void loadSettings();

void setup() {
  // Load settings from EEPROM
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

  // Load last mode
  uint8_t lastMode = EEPROM.read(EEPROM_MODE_ADDR);
  if (lastMode >= MODE_LFO && lastMode <= MODE_SPECTRUM) {
    mode = lastMode;
  }

  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC);
  #ifdef PANEL_USD
  display.setRotation(2);  // 180 degree rotation for upside-down use
  #else
  display.setRotation(0);  // Normal orientation
  #endif

  // Show boot animation
  drawBootAnimation();
  delay(2000);  // Show logo for 2 seconds

  // Configure display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // Configure I/O pins
  pinMode(OFFSET_PIN, OUTPUT);        // Offset voltage
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Push switch
  pinMode(FILTER_PIN, INPUT);         // Input is high impedance -> no active 2-pole filter
  pinMode(TRIGGER_PIN, INPUT);        // External trigger detect

  // Configure Fast PWM
  TCCR2B &= B11111000;
  TCCR2B |= B00000001;

  // Load settings for current mode
  loadSettings();
  
  // Set up hardware for current mode
  setupMode(mode);
}

void loop() {
  old_SW = SW;
  old_mode = mode;
  // Read button state
  SW = digitalRead(BUTTON_PIN) == LOW;

  // Button timing logic
  static unsigned long buttonPressStart = 0;
  static bool isLongPressing = false;
  static bool hasSaved = false;
  
  // Track button press start time
  if (SW && !old_SW) {
    buttonPressStart = millis();
    isLongPressing = true;
    hasSaved = false;
  }
  
  // Check for button release
  if (!SW && old_SW) {
    isLongPressing = false;
  }
  
  // Check for save condition: long press between 1-2 seconds
  if (isLongPressing && !hasSaved && (millis() - buttonPressStart >= 1000) && (millis() - buttonPressStart < 3000)) {
    saveSettings();
    hasSaved = true; // Prevent multiple saves in one press
  }

  // Check for secret menu entry: very long press (≥3 seconds)
  if (SW && !encoderPressed && !secretMenuActive && (millis() - buttonPressStart >= 3000)) {
    secretMenuActive = true;
    encoderPressed = false;  // Reset press state
    secretMenuOption = 1;    // Reset to first option
    oldPosition = newPosition = encoder.read();  // Reset encoder position
  } else if (!SW && encoderPressed && !secretMenuActive) {
    encoderPressed = false;
  }

  // Main logic branching
  if (secretMenuActive) {
    secretMenu();
  } else {
    // Main operation logic
    // First read of encoder
    newPosition = encoderDirection * encoder.read();

    // Handle button press for parameter selection - EXACTLY like original code
    // Ignore very short presses that might be from saving
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
    
    // Parameter constraints
    mode = constrain(mode, 1, 4);
    param = constrain(param, 1, 3);

    // Second read of encoder - EXACTLY like original formula
    newPosition = encoderDirection * encoder.read();

    // Check for encoder rotation - EXACTLY like original formula
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

    // Store the current param_select value before mode change
    byte currentParamSelect = param_select;
    
    // Mode setup if changed
    if (old_mode != mode) {
      setupMode(mode);  // Load saved parameters for the new mode
      display.clearDisplay();
      
      // IMPORTANT: Restore param_select to continue mode selection
      if (currentParamSelect == 1) {
        param_select = 1;  // Keep mode selection active
      }
      
      // Reset the hide timer to ensure parameters remain visible after changing mode
      hideTimer = millis();
    }

    // Determine if parameters should be shown
    hide = (millis() - hideTimer >= (menuTimer * 1000));
    bool showParams = !hide;

    // Run appropriate mode
    switch (mode) {
      case MODE_LFO:
        runLFOMode(showParams);
        break;
      case MODE_WAVE:
        runWaveMode(showParams);
        break;
      case MODE_SHOT:
        runShotMode(showParams);
        break;
      case MODE_SPECTRUM:
        runSpectrumMode(showParams);
        break;
    }

    display.display();
  }
}

// Boot animation
void drawBootAnimation() {
  for (int x = 0; x <= logoWidth; x += 10) {
    display.clearDisplay();
    
    // Position the bitmap at the same vertical offset as in the original
    display.drawBitmap(0, yOffset, Modulove_Logo, logoWidth, logoHeight, WHITE);
    
    // Apply the wipe effect
    display.fillRect(x, yOffset, logoWidth - x, logoHeight, BLACK);
    
    display.display();
  }
}

// Configure hardware for specific mode
void setupMode(uint8_t mode) {
  // First load any saved parameters for this mode
  int baseAddr = EEPROM_PARAM_SELECT_ADDR + ((mode - 1) * 3);
  
  // Load the parameters for the selected mode
  uint8_t saved_param_select = EEPROM.read(baseAddr);
  uint8_t saved_param1 = EEPROM.read(baseAddr + 1);
  uint8_t saved_param2 = EEPROM.read(baseAddr + 2);
  
  // Only use saved values if they're valid
  if (saved_param_select <= 3) {
    param_select = saved_param_select;
  }
  
  // Now configure hardware based on mode
  switch (mode) {
    case MODE_LFO:
      // Use saved param1 and param2 if they're valid, otherwise use defaults
      if (saved_param1 >= 1 && saved_param1 <= 8) {
        param1 = saved_param1;
      } else {
        param1 = 4;  // Default Time
      }
      
      if (saved_param2 >= 1 && saved_param2 <= 8) {
        param2 = saved_param2;
      } else {
        param2 = 1;  // Default Offset
      }
      
      pinMode(FILTER_PIN, INPUT);  // No active 2-pole filter
      analogWrite(OFFSET_PIN, 0);  // Offset = 0V
      
      // Fast ADC setup
      ADCSRA = (ADCSRA & 0xf8) | 0x04;  // Fast ADC *8 speed
      break;
      
    case MODE_WAVE:
      // Use saved param1 and param2 if they're valid, otherwise use defaults
      if (saved_param1 >= 1 && saved_param1 <= 8) {
        param1 = saved_param1;
      } else {
        param1 = 8;  // Default Time
      }
      
      if (saved_param2 >= 1 && saved_param2 <= 6) {
        param2 = saved_param2;
      } else {
        param2 = 1;  // Default Refresh rate
      }
      
      analogWrite(OFFSET_PIN, 127);  // Offset = 2.5V
      pinMode(FILTER_PIN, INPUT);    // No active 2-pole filter
      
      // Fast ADC setup
      ADCSRA = (ADCSRA & 0xf8) | 0x04;  // Fast ADC *8 speed
      break;
      
    case MODE_SHOT:
      // Use saved param1 if valid, otherwise use default
      if (saved_param1 >= 1 && saved_param1 <= 4) {
        param1 = saved_param1;
      } else {
        param1 = 2;  // Default Time
      }
      
      param2 = 1;  // Not used in this mode
      analogWrite(OFFSET_PIN, 127);  // Offset = 2.5V
      pinMode(FILTER_PIN, INPUT);    // No active 2-pole filter
      
      // Fast ADC setup
      ADCSRA = (ADCSRA & 0xf8) | 0x04;  // Fast ADC *8 speed
      break;
      
    case MODE_SPECTRUM:
      // Use saved param1 and param2 if they're valid, otherwise use defaults
      if (saved_param1 >= 1 && saved_param1 <= 4) {
        param1 = saved_param1;
      } else {
        param1 = 1;  // Default High freq amp
      }
      
      if (saved_param2 >= 1 && saved_param2 <= 8) {
        param2 = saved_param2;
      } else {
        param2 = 8;  // Default Noise filter
      }
      
      analogWrite(OFFSET_PIN, 127);  // Offset = 2.5V
      
      // Active filter setup
      pinMode(FILTER_PIN, OUTPUT);
      digitalWrite(FILTER_PIN, LOW);
      
      // Standard ADC for better accuracy
      ADCSRA = (ADCSRA & 0xf8) | 0x07;
      break;
  }
  
  // Zero out the buffer
  memset(&buffer, 0, sizeof(buffer));
  
  // Reset refresh counter
  rfrs = 0;
}


// LFO Mode (Low Frequency Oscilloscope)
void runLFOMode(bool showParams) {
  // Constrain parameters to valid ranges
  param = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);  // Time scale
  param2 = constrain(param2, 1, 8);  // Vertical offset
  
  // Sample current value
  uint8_t currentSample = analogRead(ANALOG_INPUT_PIN) / 16;  // Scale to 0-63
  
  // Shift buffer for scrolling effect
  for (int i = 127; i > 0; i--) {
    buffer.waveform[i] = buffer.waveform[i - 1];
  }
  buffer.waveform[0] = currentSample;
  
  // Redraw display only when needed (parameters changed or new sample)
  static unsigned long lastDrawTime = 0;
  if (millis() - lastDrawTime >= 50) { // Limit refresh rate to avoid flickering
    lastDrawTime = millis();
    display.clearDisplay();
    
    // Draw waveform
    for (int i = 0; i < 126 / (9 - param1); i++) {
      int x1 = 127 - (i * (9 - param1));
      int y1 = 63 - buffer.waveform[i] - (param2 - 1) * 4;
      int x2 = 127 - ((i + 1) * (9 - param1));
      int y2 = 63 - buffer.waveform[i + 1] - (param2 - 1) * 4;
      
      display.drawLine(x1, y1, x2, y2, WHITE);
    }
    
    // Show parameters if needed
    if (showParams) {
      drawParameterBar(showParams);
    }
  }
}

// Wave Mode (High Frequency Oscilloscope)
void runWaveMode(bool showParams) {
  // Constrain parameters to valid ranges
  param = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);  // Time scale
  param2 = constrain(param2, 1, 6);  // Refresh rate
  
  // Calculate the actual update frequency based on refresh rate
  static unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 50 + (param2 - 1) * 100; // 50ms base + additional delay by param
  bool updateNeeded = (millis() - lastUpdateTime >= updateInterval);
  
  // Sample data based on time scale
  if (param1 > 5) {  // Mid frequency
    // Only sample and redraw when it's time to update
    if (updateNeeded) {
      lastUpdateTime = millis();
      
      // Sample data
      for (int i = 127; i >= 0; i--) {
        buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) / 16;
        delayMicroseconds((param1 - 5) * 20);
      }
      
      // Clear and redraw
      display.clearDisplay();
      
      // Draw waveform
      for (int i = 127; i >= 1; i--) {
        display.drawLine(127 - i, 63 - buffer.waveform[i - 1], 
                        127 - (i + 1), 63 - buffer.waveform[i], WHITE);
      }
      
      // Show parameters if needed
      if (showParams) {
        drawParameterBar(showParams);
      }
    }
  } else if (param1 <= 5) {  // High frequency
    // Only sample and redraw when it's time to update
    if (updateNeeded) {
      lastUpdateTime = millis();
      
      // Sample data
      for (int i = 127 / (6 - param1); i >= 0; i--) {
        buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) / 16;
      }
      
      // Clear and redraw
      display.clearDisplay();
      
      // Draw waveform with fewer points
      for (int i = 127 / (6 - param1); i >= 1; i--) {
        display.drawLine(127 - i * (6 - param1), 63 - buffer.waveform[i - 1], 
                        127 - (i + 1) * (6 - param1), 63 - buffer.waveform[i], WHITE);
      }
      
      // Show parameters if needed
      if (showParams) {
        drawParameterBar(showParams);
      }
    }
  }
}

// Shot Mode (Triggered Oscilloscope)
void runShotMode(bool showParams) {
  // Constrain parameters to valid ranges
  param = constrain(param, 1, 2);
  param1 = constrain(param1, 1, 4);  // Time scale
  
  // Update trigger state
  old_trig = trig;
  trig = digitalRead(TRIGGER_PIN);
  
  // Track when we need to update the display
  static bool redrawNeeded = true;
  static unsigned long lastUpdateTime = 0;
  
  // Detect rising edge trigger
  if (!old_trig && trig) {
    // On trigger, capture the waveform
    for (int i = 10; i <= 127; i++) {
      buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) / 16;
      delayMicroseconds(100000 * param1);  // Sampling rate
    }
    
    // Add trigger marker
    for (int i = 0; i < 10; i++) {
      buffer.waveform[i] = 32;
    }
    
    redrawNeeded = true;  // Flag that we need to redraw
  }
  
  // Only redraw occasionally to prevent flickering
  if (redrawNeeded || millis() - lastUpdateTime >= 200) {
    lastUpdateTime = millis();
    redrawNeeded = false;
    
    // Clear and draw
    display.clearDisplay();
    
    // Draw waveform
    for (int i = 126; i >= 1; i--) {
      display.drawLine(i, 63 - buffer.waveform[i], (i + 1), 63 - buffer.waveform[i + 1], WHITE);
    }
    
    // Show parameters if needed
    if (showParams) {
      drawParameterBar(showParams);
    }
  }
}

// Spectrum Mode (Frequency Analyzer)
void runSpectrumMode(bool showParams) {
  // Constrain parameters to valid ranges
  param = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 4);  // High freq sense
  param2 = constrain(param2, 1, 8);  // Noise filter
  
  // Limit update rate to prevent flickering
  static unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 100; // Update every 100ms
  
  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis();
    
    // Sample data for spectrum analysis
    for (uint8_t i = 0; i < 128; i++) {
      int spec = analogRead(ANALOG_INPUT_PIN);
      buffer.spectrum.real[i] = spec / 4 - 128;  // Convert to signed 8-bit
      buffer.spectrum.imag[i] = 0;
    }
    
    // Perform FFT
    fix_fft(buffer.spectrum.real, buffer.spectrum.imag, 7, 0);
    
    // Clear display and redraw
    display.clearDisplay();
    
    // Draw spectrum
    for (uint8_t i = 0; i < 64; i++) {
      // Calculate magnitude
      int level = sqrt(buffer.spectrum.real[i] * buffer.spectrum.real[i] + 
                      buffer.spectrum.imag[i] * buffer.spectrum.imag[i]);
      
      // Apply noise filter
      if (level >= param2) {
        // Draw bar with high frequency amplification
        display.fillRect(i * 2, 63 - (level + i * (param1 - 1) / 8), 2, 
                        (level + i * (param1 - 1) / 8), WHITE);
      }
    }
    
    // Show parameters if needed
    if (showParams) {
      drawParameterBar(showParams);
    }
  }
}

// Draw parameter bar at top of screen
void drawParameterBar(bool showParams) {
  if (!showParams) return;
  
  // Highlight current parameter
  display.drawLine((param - 1) * 42, 8, (param - 1) * 42 + 36, 8, WHITE);
  
  // Draw mode parameter
  display.setTextColor(param_select == 1 ? BLACK : WHITE, 
                       param_select == 1 ? WHITE : BLACK);
  display.setCursor(0, 0);
  display.print("Mode:");
  display.setCursor(30, 0);
  display.print(mode);
  
  // Draw param1 (changes by mode)
  display.setTextColor(param_select == 2 ? BLACK : WHITE, 
                       param_select == 2 ? WHITE : BLACK);
  display.setCursor(42, 0);
  
  switch (mode) {
    case MODE_LFO:
    case MODE_WAVE:
    case MODE_SHOT:
      display.print("Time:");
      break;
    case MODE_SPECTRUM:
      display.print("High:");
      break;
  }
  
  display.setCursor(72, 0);
  display.print(param1);
  
  // Draw param2 or trigger indicator
  if (mode == MODE_SHOT) {
    // Show trigger indicator for Shot mode
    display.setTextColor(trig ? BLACK : WHITE, 
                         trig ? WHITE : BLACK);
    display.setCursor(84, 0);
    display.print("TRIG");
  } else {
    // Show param2 for other modes
    display.setTextColor(param_select == 3 ? BLACK : WHITE, 
                         param_select == 3 ? WHITE : BLACK);
    display.setCursor(84, 0);
    
    switch (mode) {
      case MODE_LFO:
        display.print("Offs:");
        break;
      case MODE_WAVE:
        display.print("Rfrs:");
        break;
      case MODE_SPECTRUM:
        display.print("Filt:");
        break;
    }
    
    display.setCursor(114, 0);
    display.print(param2);
  }
}

// Handle secret menu for settings
void secretMenu() {
  int newDirection = encoderDirection;
  newPosition = encoder.read();

  // Check if the encoder was turned
  if (newPosition > (oldPosition + 3)) {
    oldPosition = newPosition;
    if (secretMenuOption == 1) {
      newDirection = 1;  // Normal direction
    } else if (secretMenuOption == 2) {
      menuTimer -= 1;  // Always increment when turning right
      menuTimer = constrain(menuTimer, 1, 60);
    }
  } else if (newPosition < (oldPosition - 3)) {
    oldPosition = newPosition;
    if (secretMenuOption == 1) {
      newDirection = -1;  // Reversed direction
    } else if (secretMenuOption == 2) {
      menuTimer += 1;  // Always decrement when turning left
      menuTimer = constrain(menuTimer, 1, 60);
    }
  }

  // Update encoder direction if changed
  if (newDirection != encoderDirection) {
    encoderDirection = newDirection;
  }

  // Rest of the function remains unchanged
  // Handle button press
  if (old_SW == 0 && SW == 1) {
    secretMenuOption = secretMenuOption == 1 ? 2 : 1;
  }

  // Check for long press to exit
  if (SW && !encoderPressed) {
    encoderPressed = true;
    exitPressStartTime = millis();
  } else if (!SW && encoderPressed) {
    encoderPressed = false;
    exitPressStartTime = 0;
  }

  // Calculate exit progress (0-100%)
  int exitProgress = 0;
  if (encoderPressed && exitPressStartTime > 0) {
    unsigned long pressDuration = millis() - exitPressStartTime;
    exitProgress = (pressDuration * 100) / 3000;  // 3000ms = 100%
    exitProgress = constrain(exitProgress, 0, 100);
  }

  // Exit after long press
  if (exitProgress >= 100) {
    // Save settings to EEPROM
    EEPROM.put(ENCODER_DIR_ADDR, encoderDirection);
    delay(10);
    EEPROM.put(MENUTIMER_DIR_ADDR, menuTimer);
    delay(10);
    
    // Show save confirmation
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(20, 25);
    display.print("SETTINGS SAVED");
    display.display();
    delay(1000);
    
    // Exit menu
    secretMenuActive = false;
    encoderPressed = false;
    exitPressStartTime = 0;
    oldPosition = newPosition = encoder.read() * encoderDirection;  // Reset positions
    display.clearDisplay();
    display.display();
    return;
  }

  // Draw secret menu
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.println("GLOBAL SETTINGS MENU");
  
  // Encoder direction option
  display.setTextColor(secretMenuOption == 1 ? BLACK : WHITE, 
                      secretMenuOption == 1 ? WHITE : BLACK);
  display.println("Encoder Direction:");
  display.setTextColor(WHITE);
  display.println(encoderDirection == 1 ? "Normal" : "Reversed");
  display.println("");
  
  // Menu timer option
  display.setTextColor(secretMenuOption == 2 ? BLACK : WHITE, 
                      secretMenuOption == 2 ? WHITE : BLACK);
  display.println("Menu Timer:");
  display.setTextColor(WHITE);
  display.print(menuTimer);
  display.println(" seconds");
  
  // Draw exit progress bar if button is pressed
  if (exitProgress > 0) {
    display.drawRect(0, 56, 128, 8, WHITE);
    display.fillRect(2, 58, (exitProgress * 124) / 100, 4, WHITE);
    display.setCursor(0, 48);
    display.print("Hold to save & exit");
  } else {
    display.setCursor(0, 56);
    display.print("Hold button to exit");
  }
  
  display.display();
}

void saveSettings() {
  // Save current mode
  EEPROM.write(EEPROM_MODE_ADDR, mode);
  delay(10); // Small delay to ensure write completes
  
  // Use specific mode offset for storing parameters
  // Starting at EEPROM_PARAM_SELECT_ADDR (which is 5)
  // Each mode gets 3 bytes: param_select, param1, param2
  int baseAddr = EEPROM_PARAM_SELECT_ADDR + ((mode - 1) * 3);
  
  // Write the parameters
  EEPROM.write(baseAddr, param_select);
  delay(5);
  EEPROM.write(baseAddr + 1, param1);
  delay(5);
  EEPROM.write(baseAddr + 2, param2);
  delay(5);
  
  lastSaveTime = millis();
  
  // Visual feedback for saving
  display.fillRect(0, 54, 128, 10, WHITE); // White bar at top
  display.setTextColor(BLACK);
  display.setCursor(8, 55);
  display.print("MODE SETTINGS SAVED");
  display.display();
  delay(1000);
  
  // Optional: Show what was saved (helpful for debugging)
  /*
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Mode: ");
  display.println(mode);
  display.print("Param1: ");
  display.println(param1);
  display.print("Param2: ");
  display.println(param2);
  display.print("ParamSel: ");
  display.println(param_select);
  display.display();
  delay(1000);
  */
}

// Load settings from EEPROM
void loadSettings() {
  // Calculate the mode-specific address
  int baseAddr = EEPROM_PARAM_SELECT_ADDR + ((mode - 1) * 3);
  
  // Load the parameters for the current mode
  param_select = EEPROM.read(baseAddr);
  param1 = EEPROM.read(baseAddr + 1);
  param2 = EEPROM.read(baseAddr + 2);
  
  // Validate parameters (in case of EEPROM corruption)
  if (param_select > 3) param_select = 0;
  
  // Mode-specific parameter validation
  switch (mode) {
    case MODE_LFO:
      param1 = constrain(param1, 1, 8);
      param2 = constrain(param2, 1, 8);
      break;
      
    case MODE_WAVE:
      param1 = constrain(param1, 1, 8);
      param2 = constrain(param2, 1, 6);
      break;
      
    case MODE_SHOT:
      param1 = constrain(param1, 1, 4);
      param2 = constrain(param2, 1, 1);
      break;
      
    case MODE_SPECTRUM:
      param1 = constrain(param1, 1, 4);
      param2 = constrain(param2, 1, 8);
      break;
  }
}
