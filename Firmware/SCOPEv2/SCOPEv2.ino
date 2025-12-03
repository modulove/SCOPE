/**
 * @file SCOPEv2.ino
 * @author Modulove
 * @brief Eurorack scope + OLED flip + Tuner
 * @version 1.8
 * @date 2025-12-02
 * 
 * MODIFICATIONS v1.8:
 * - Tuner mode added
 * - Config menu: Added Boot Logo On/Off option
 * - ADC interrupt sampling
 */

#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "fix_fft.h"
#include <Encoder.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Display  ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI 9
#define OLED_CLK 10
#define OLED_DC 11
#define OLED_CS 12
#define OLED_RESET 13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// ---------------- Pins ----------------
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 4
#define BUTTON_PIN    5
#define FILTER_PIN    6
#define TRIGGER_PIN   7
#define OFFSET_PIN    3
#define ANALOG_INPUT_PIN 0

Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// ---------------- EEPROM ----------------
#define ENCODER_DIR_ADDR      0
#define OLED_ROT_ADDR         1
#define MENUTIMER_DIR_ADDR    2
#define BOOTLOGO_ADDR         3
const int EEPROM_MODE_ADDR = 4;
const int EEPROM_PARAM_SELECT_ADDR = 5;

// ---------------- Modes ----------------
#define MODE_LFO      1
#define MODE_WAVE     2
#define MODE_SHOT     3
#define MODE_SPECTRUM 4
#define MODE_TUNER    5
#define NUM_MODES     5

// ---------------- ADC Sampling  ----------------
#define ADC_BUFFER_SIZE   256
#define ADC_HALF_BUFFER   128

volatile uint16_t adcSampleIndex = 0;
volatile uint16_t adcTargetSamples = 128;  // How many samples to collect
volatile bool adcBufferReady = false;
volatile bool adcSampling = false;
volatile uint8_t adcDelayCounter = 0;      // For slower sample rates
volatile uint8_t adcDelayTarget = 0;       // Skip N interrupts between samples

// ---------------- YIN Algorithm Constants ----------------
#define YIN_THRESHOLD_FP  38    // 0.15 * 256 (fixed-point threshold)
#define YIN_MIN_PERIOD    6
#define YIN_MAX_PERIOD    120

// Tuner state
float smoothedFrequency = 0;
uint8_t tunerAutoRange = 2;  // Auto-detected range: 1=LOW, 2=MID, 3=HI

// ---------------- Boot logo (PROGMEM) ----------------
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

const int logoWidth = 128;
const int logoHeight = 30;
const int yOffset = 14;

// ---------------- Shared buffer (256 bytes) ----------------
union {
  struct { int8_t real[128]; int8_t imag[128]; } spectrum;
  uint8_t raw[256];  // For waveform (first 128) and YIN (all 256)
} buffer;

// ---------------- Mode Settings Storage ----------------
// Store settings for ALL modes (15 bytes total)
struct ModeSettings {
  uint8_t param_select;
  uint8_t param1;
  uint8_t param2;
};
ModeSettings modeSettings[NUM_MODES];

// Current working values 
uint8_t mode = MODE_LFO;
uint8_t old_mode = MODE_LFO;
uint8_t param_select = 0;
uint8_t param = 1;
uint8_t param1 = 2;
uint8_t param2 = 1;

bool trig = 0, old_trig = 0;
bool SW = 0, old_SW = 0;

unsigned long hideTimer = 0;
unsigned long lastSaveTime = 0;
bool hide = 0;
int rfrs = 0;

float oldPosition = -999;
float newPosition = -999;

// config menu
bool configMenuActive = false;
byte configMenuOption = 1;
unsigned int menuTimer = 5;
int encoderDirection = 1;
uint8_t oledRotation = 0;
uint8_t bootLogoEnabled = 1;

// Function declarations
void drawBootAnimation();
void setupMode(uint8_t mode);
void runLFOMode(bool showParams);
void runWaveMode(bool showParams);
void runShotMode(bool showParams);
void runSpectrumMode(bool showParams);
void runTunerMode(bool showParams);
void drawParameterBar(bool showParams);
void configMenu();
void saveCurrentModeToRAM();
void saveAllSettings();
void loadAllSettings();
void startADCSampling(uint16_t numSamples, uint8_t prescaler, uint8_t delaySkip);
void stopADCSampling();
float detectFrequencyYIN();

// ================== Unified ADC Interrupt ==================
ISR(ADC_vect) {
  if (!adcSampling) return;
  
  // Optional delay: skip N interrupts between samples
  if (adcDelayCounter < adcDelayTarget) {
    adcDelayCounter++;
    return;
  }
  adcDelayCounter = 0;
  
  if (adcSampleIndex < adcTargetSamples) {
    buffer.raw[adcSampleIndex++] = ADCH;
    
    if (adcSampleIndex >= adcTargetSamples) {
      adcBufferReady = true;
      adcSampling = false;
      ADCSRA &= ~((1 << ADATE) | (1 << ADIE));
    }
  }
}

// Start ADC sampling with configurable parameters
// prescaler: ADC clock divider (0x05=/32, 0x06=/64, 0x07=/128)
// delaySkip: skip N interrupts between samples (for slower rates)
void startADCSampling(uint16_t numSamples, uint8_t prescaler, uint8_t delaySkip) {
  cli();
  
  adcSampleIndex = 0;
  adcTargetSamples = min(numSamples, (uint16_t)ADC_BUFFER_SIZE);
  adcBufferReady = false;
  adcSampling = true;
  adcDelayCounter = 0;
  adcDelayTarget = delaySkip;
  
  // 8-bit left-adjusted, AVCC reference
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (ANALOG_INPUT_PIN & 0x07);
  
  // Enable ADC, start conversion, auto-trigger, interrupt enable
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (prescaler & 0x07);
  ADCSRB = 0;  // Free-running mode
  
  sei();
}

void stopADCSampling() {
  ADCSRA &= ~((1 << ADIE) | (1 << ADATE));
  adcSampling = false;
}

// ---------------- Setup ----------------
void setup() {
  EEPROM.get(ENCODER_DIR_ADDR, encoderDirection);
  if (encoderDirection != 1 && encoderDirection != -1) encoderDirection = 1;

  oledRotation = EEPROM.read(OLED_ROT_ADDR);
  if (!(oledRotation == 0 || oledRotation == 2)) oledRotation = 0;

  EEPROM.get(MENUTIMER_DIR_ADDR, menuTimer);
  if (menuTimer < 1 || menuTimer > 60) menuTimer = 5;

  bootLogoEnabled = EEPROM.read(BOOTLOGO_ADDR);
  if (bootLogoEnabled > 1) bootLogoEnabled = 1;  // Default to enabled

  uint8_t lastMode = EEPROM.read(EEPROM_MODE_ADDR);
  if (lastMode >= MODE_LFO && lastMode <= MODE_TUNER) mode = lastMode;

  display.begin(SSD1306_SWITCHCAPVCC);
  display.setRotation(oledRotation);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (bootLogoEnabled) {
    drawBootAnimation();
    delay(800);
  }

  pinMode(OFFSET_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FILTER_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);

  TCCR2B &= B11111000;
  TCCR2B |= B00000001;

  loadAllSettings();  // Load ALL mode settings from EEPROM
  setupMode(mode);
}

// ---------------- Loop ----------------
void loop() {
  old_SW = SW;
  old_mode = mode;
  SW = (digitalRead(BUTTON_PIN) == LOW);

  static unsigned long buttonPressStart = 0;
  static bool isLongPressing = false;
  static bool hasSaved = false;
  static bool hasEnteredConfig = false;

  if (SW && !old_SW) {
    buttonPressStart = millis();
    isLongPressing = true;
    hasSaved = false;
    hasEnteredConfig = false;
  }
  if (!SW && old_SW) isLongPressing = false;

  if (isLongPressing && !hasSaved) {
    unsigned long d = millis() - buttonPressStart;
    if (d >= 1000) {
      saveCurrentModeToRAM();  // Ensure current mode is in RAM
      saveAllSettings();       // Save ALL modes to EEPROM
      hasSaved = true;
    }
  }

  // Enter config menu after save feedback shown (3 seconds hold)
  if (SW && !configMenuActive && !hasEnteredConfig && hasSaved && (millis() - buttonPressStart >= 3000)) {
    configMenuActive = true;
    hasEnteredConfig = true;
    configMenuOption = 1;
    oldPosition = newPosition = encoder.read();
  }

  if (configMenuActive) {
    configMenu();
    return;
  }

  newPosition = encoderDirection * encoder.read();

  if (old_SW == 0 && SW == 1 && param_select == param) { param_select = 0; hideTimer = millis(); }
  else if (old_SW == 0 && SW == 1 && (param >= 1 && param <= 3)) { param_select = param; hideTimer = millis(); }

  mode = constrain(mode, 1, 5);
  param = constrain(param, 1, 3);

  newPosition = encoderDirection * encoder.read();
  if ((newPosition - 3) / 4 > oldPosition / 4) {
    oldPosition = newPosition; hideTimer = millis();
    switch (param_select) {
      case 0: param--; break;
      case 1: mode--;  break;
      case 2: param1--;break;
      case 3: param2--;break;
    }
  } else if ((newPosition + 3) / 4 < oldPosition / 4) {
    oldPosition = newPosition; hideTimer = millis();
    switch (param_select) {
      case 0: param++; break;
      case 1: mode++;  break;
      case 2: param1++;break;
      case 3: param2++;break;
    }
  }

  byte currentParamSelect = param_select;
  if (old_mode != mode) {
    // Save current mode settings to RAM before switching
    saveCurrentModeToRAM();
    
    setupMode(mode);
    display.clearDisplay();
    if (currentParamSelect == 1) param_select = 1;
    hideTimer = millis();
  }

  hide = (millis() - hideTimer >= (menuTimer * 1000UL));
  bool showParams = !hide;

  switch (mode) {
    case MODE_LFO:      runLFOMode(showParams); break;
    case MODE_WAVE:     runWaveMode(showParams); break;
    case MODE_SHOT:     runShotMode(showParams); break;
    case MODE_SPECTRUM: runSpectrumMode(showParams); break;
    case MODE_TUNER:    runTunerMode(showParams); break;
  }

  display.display();
}

// ---------------- Boot animation ----------------
void drawBootAnimation() {
  for (int x = 0; x <= logoWidth; x += 10) {
    display.clearDisplay();
    display.drawBitmap(0, yOffset, Modulove_Logo, logoWidth, logoHeight, WHITE);
    display.fillRect(x, yOffset, logoWidth - x, logoHeight, BLACK);
    display.display();
  }
}

// ================== Settings Management ==================

// Save current working params to RAM array
void saveCurrentModeToRAM() {
  uint8_t idx = mode - 1;
  if (idx < NUM_MODES) {
    modeSettings[idx].param_select = param_select;
    modeSettings[idx].param1 = param1;
    modeSettings[idx].param2 = param2;
  }
}

// Load ALL mode settings from EEPROM into RAM
void loadAllSettings() {
  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int baseAddr = EEPROM_PARAM_SELECT_ADDR + (m * 3);
    modeSettings[m].param_select = EEPROM.read(baseAddr);
    modeSettings[m].param1 = EEPROM.read(baseAddr + 1);
    modeSettings[m].param2 = EEPROM.read(baseAddr + 2);
    
    // Validate and set defaults
    if (modeSettings[m].param_select > 3) modeSettings[m].param_select = 0;
    
    // Mode-specific validation
    switch (m + 1) {  // m+1 = actual mode number
      case MODE_LFO:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 8);
        modeSettings[m].param2 = constrain(modeSettings[m].param2, -6, 10);
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 4;
        if (modeSettings[m].param2 == 0) modeSettings[m].param2 = 1;
        break;
      case MODE_WAVE:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 8);
        modeSettings[m].param2 = constrain(modeSettings[m].param2, 1, 6);
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 8;
        if (modeSettings[m].param2 == 0) modeSettings[m].param2 = 1;
        break;
      case MODE_SHOT:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 4);
        modeSettings[m].param2 = 1;
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 2;
        break;
      case MODE_SPECTRUM:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 4);
        modeSettings[m].param2 = constrain(modeSettings[m].param2, 1, 8);
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 1;
        if (modeSettings[m].param2 == 0) modeSettings[m].param2 = 8;
        break;
      case MODE_TUNER:
        modeSettings[m].param1 = 1;  // Not used anymore (hardcoded smoothing)
        modeSettings[m].param2 = 1;  // Not used anymore (auto-range)
        break;
    }
  }
}

// Save ALL mode settings from RAM to EEPROM
void saveAllSettings() {
  EEPROM.update(EEPROM_MODE_ADDR, mode);
  
  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int baseAddr = EEPROM_PARAM_SELECT_ADDR + (m * 3);
    EEPROM.update(baseAddr, modeSettings[m].param_select);
    EEPROM.update(baseAddr + 1, modeSettings[m].param1);
    EEPROM.update(baseAddr + 2, modeSettings[m].param2);
  }

  display.fillRect(0, 54, 128, 10, WHITE);
  display.setTextColor(BLACK);
  display.setCursor(4, 55);
  display.print(F("ALL SETTINGS SAVED"));
  display.display();
  delay(500);
}

// ---------------- Mode setup ----------------
void setupMode(uint8_t m) {
  stopADCSampling();
  
  // Load settings from RAM array
  uint8_t idx = m - 1;
  if (idx < NUM_MODES) {
    param_select = modeSettings[idx].param_select;
    param1 = modeSettings[idx].param1;
    param2 = modeSettings[idx].param2;
  }

  switch (m) {
    case MODE_LFO:
      pinMode(FILTER_PIN, INPUT);
      analogWrite(OFFSET_PIN, 0);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;  // Standard analogRead prescaler
      break;

    case MODE_WAVE:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      // Will use ADC interrupt for sampling
      break;

    case MODE_SHOT:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      // Will use ADC interrupt for sampling
      break;

    case MODE_SPECTRUM:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, OUTPUT);
      digitalWrite(FILTER_PIN, LOW);
      // Will use ADC interrupt for consistent FFT sampling
      break;

    case MODE_TUNER:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      smoothedFrequency = 0;
      break;
  }

  memset(&buffer, 0, sizeof(buffer));
  rfrs = 0;
}

// ---------------- LFO Mode  ----------------
void runLFOMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);
  param2 = constrain(param2, -6, 10);

  uint8_t currentSample = analogRead(ANALOG_INPUT_PIN) >> 4;

  memmove(&buffer.raw[1], &buffer.raw[0], 127);
  buffer.raw[0] = currentSample;

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 50) {
    lastDraw = millis();
    display.clearDisplay();

    int step = (9 - param1);
    int voff = (param2 - 1) * 6;
    int segments = 126 / step;

    for (int i = 0; i < segments; i++) {
      int x1 = 127 - (i * step);
      int y1 = buffer.raw[i] + voff;
      int x2 = 127 - ((i + 1) * step);
      int y2 = buffer.raw[i + 1] + voff;
      y1 = constrain(y1, 0, 63);
      y2 = constrain(y2, 0, 63);
      display.drawLine(x1, y1, x2, y2, WHITE);
    }

    if (showParams) drawParameterBar(true);
  }
}

// ---------------- Wave Mode  ----------------
void runWaveMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);
  param2 = constrain(param2, 1, 6);

  static unsigned long lastUpdate = 0;
  static uint8_t state = 0;
  
  unsigned long interval = 50UL + (param2 - 1) * 100UL;
  
  switch (state) {
    case 0:  // Wait for interval
      if (millis() - lastUpdate >= interval) {
        // Calculate prescaler and delay based on param1 (time/zoom)
        uint8_t prescaler, delaySkip;
        if (param1 <= 5) {
          prescaler = 0x05;  // /32 = fastest
          delaySkip = (6 - param1) * 2;
        } else {
          prescaler = 0x06;  // /64
          delaySkip = (param1 - 5) * 4;
        }
        startADCSampling(128, prescaler, delaySkip);
        state = 1;
      }
      break;
      
    case 1:  // Wait for samples
      if (adcBufferReady) {
        lastUpdate = millis();
        state = 2;
      }
      break;
      
    case 2:  // Draw
      display.clearDisplay();
      
      // Scale 8-bit samples (0-255) to 6-bit range (0-63) for full screen height
      for (uint8_t i = 0; i < 128; i++) {
        buffer.raw[i] = buffer.raw[i] >> 2;
      }
      
      // Fixed: removed 63- inversion
      for (int i = 1; i < 127; i++) {
        display.drawLine(127 - i, buffer.raw[i - 1],
                         127 - (i + 1), buffer.raw[i], WHITE);
      }
      
      if (showParams) drawParameterBar(true);
      state = 0;
      break;
  }
}

// ---------------- Shot Mode  ----------------
void runShotMode(bool showParams) {
  param  = constrain(param, 1, 2);
  param1 = constrain(param1, 1, 4);

  old_trig = trig;
  trig = digitalRead(TRIGGER_PIN);

  static bool redrawNeeded = true;
  static unsigned long lastUpdate = 0;
  static uint8_t state = 0;

  switch (state) {
    case 0:  // Wait for trigger
      if (!old_trig && trig) {
        uint8_t prescaler = 0x05 + (param1 - 1);
        if (prescaler > 0x07) prescaler = 0x07;
        
        uint8_t delaySkip = (param1 > 2) ? (param1 - 2) * 2 : 0;
        
        startADCSampling(118, prescaler, delaySkip);
        state = 1;
      }
      break;
      
    case 1:  // Wait for samples
      if (adcBufferReady) {
        // Copy samples to correct positions (10-127)
        // Scale 8-bit (0-255) to 6-bit (0-63) for full screen height
        for (int i = 117; i >= 0; i--) {
          buffer.raw[i + 10] = buffer.raw[i] >> 2;
        }
        // Set trigger area to mid-point
        for (int i = 0; i < 10; i++) buffer.raw[i] = 32;
        
        redrawNeeded = true;
        state = 0;
      }
      break;
  }

  // Redraw display
  if (redrawNeeded || millis() - lastUpdate >= 200) {
    lastUpdate = millis();
    redrawNeeded = false;

    display.clearDisplay();
    // Fixed: removed 63- inversion
    for (int i = 1; i < 127; i++) {
      display.drawLine(i, buffer.raw[i], i + 1, buffer.raw[i + 1], WHITE);
    }
    if (showParams) drawParameterBar(true);
  }
}

// ---------------- Spectrum Mode ----------------
void runSpectrumMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 4);
  param2 = constrain(param2, 1, 8);

  static unsigned long lastUpdate = 0;
  static uint8_t state = 0;
  
  switch (state) {
    case 0:  // Start sampling
      if (millis() - lastUpdate >= 100) {
        startADCSampling(128, 0x06, 0);
        state = 1;
      }
      break;
      
    case 1:  // Wait for samples
      if (adcBufferReady) {
        state = 2;
      }
      break;
      
    case 2:  // Process FFT and draw
      // Convert 8-bit samples to signed for FFT
      for (uint8_t i = 0; i < 128; i++) {
        buffer.spectrum.real[i] = (int8_t)(buffer.raw[i] - 128);
        buffer.spectrum.imag[i] = 0;
      }

      fix_fft(buffer.spectrum.real, buffer.spectrum.imag, 7, 0);

      display.clearDisplay();

      for (uint8_t i = 0; i < 64; i++) {
        int re = buffer.spectrum.real[i];
        int im = buffer.spectrum.imag[i];
        int level = abs(re) + abs(im);

        if (level >= (param2 * 2)) {
          int boosted = level + ((int)i * (param1 - 1));
          boosted = constrain(boosted, 0, 63);
          display.fillRect(i * 2, 63 - boosted, 2, boosted, WHITE);
        }
      }

      if (showParams) drawParameterBar(true);
      lastUpdate = millis();
      state = 0;
      break;
  }
}

// ================== TUNER ==================

float detectFrequencyYIN() {
  int32_t runningSum = 0;
  int16_t prev2 = 256, prev1 = 256, curr = 256;
  int tauEstimate = -1;
  
  for (uint8_t tau = 1; tau < YIN_MAX_PERIOD && tauEstimate < 0; tau++) {
    int32_t sum = 0;
    uint8_t* p1 = buffer.raw;
    uint8_t* p2 = buffer.raw + tau;
    
    for (uint8_t j = 0; j < ADC_HALF_BUFFER; j++) {
      int16_t delta = (int16_t)*p1++ - (int16_t)*p2++;
      sum += delta * delta;
    }
    
    int16_t d_tau = sum >> 8;
    runningSum += d_tau;
    
    int16_t cmndf;
    if (runningSum > 0) {
      cmndf = (int16_t)(((int32_t)d_tau * tau * 256L) / runningSum);
    } else {
      cmndf = 256;
    }
    
    prev2 = prev1;
    prev1 = curr;
    curr = cmndf;
    
    if (tau >= YIN_MIN_PERIOD + 2) {
      if (prev1 < YIN_THRESHOLD_FP && prev1 <= prev2 && prev1 <= curr) {
        tauEstimate = tau - 1;
      }
    }
  }
  
  if (tauEstimate < 0) return 0;
  
  float betterTau = (float)tauEstimate;
  float denom = 2.0f * ((float)prev2 - 2.0f * (float)prev1 + (float)curr);
  if (denom != 0) {
    betterTau += ((float)prev2 - (float)curr) / denom;
  }
  
  float sampleRate;
  switch (tunerAutoRange) {
    case 1: sampleRate = 9615.0f;  break;   // LOW
    case 2: sampleRate = 19230.0f; break;   // MID
    case 3: sampleRate = 38461.0f; break;   // HI
    default: sampleRate = 19230.0f; break;
  }
  
  return sampleRate / betterTau;
}

void frequencyToNote(float freq, char* note, int8_t* octave, int8_t* cents) {
  static const char notes[] PROGMEM = "C C#D D#E F F#G G#A A#B ";
  
  float noteNum = 12.0f * log(freq / 440.0f) / log(2.0f) + 69.0f;
  int midiNote = (int)(noteNum + 0.5f);
  
  *cents = (int8_t)((noteNum - midiNote) * 100.0f);
  *octave = (midiNote / 12) - 1;
  
  uint8_t idx = (midiNote % 12) * 2;
  note[0] = pgm_read_byte(&notes[idx]);
  note[1] = pgm_read_byte(&notes[idx + 1]);
  note[2] = '\0';
  if (note[1] == ' ') note[1] = '\0';
}

// ---------------- Tuner Mode ----------------
void runTunerMode(bool showParams) {
  param = constrain(param, 1, 2);  // Only Mode selection now

  static unsigned long lastUpdate = 0;
  static uint8_t state = 0;
  
  // Hardcoded smoothing factor (0.35 = good balance of response and stability)
  const float smoothingAlpha = 0.35f;
  
  switch (state) {
    case 0:  // Start sampling with auto-detected range
      if (millis() - lastUpdate >= 80) {
        uint8_t prescaler;
        switch (tunerAutoRange) {
          case 1: prescaler = 0x07; break;  // /128 ~9.6kHz (bass <200Hz)
          case 2: prescaler = 0x06; break;  // /64  ~19kHz (mid 200-800Hz)
          case 3: prescaler = 0x05; break;  // /32  ~38kHz (high >800Hz)
          default: prescaler = 0x06; break;
        }
        startADCSampling(256, prescaler, 0);
        state = 1;
      }
      break;
      
    case 1:  // Wait for samples
      if (adcBufferReady) state = 2;
      break;
      
    case 2:  // Process
      {
        float rawFreq = detectFrequencyYIN();
        
        if (rawFreq > 30 && rawFreq < 4000) {
          if (smoothedFrequency < 10) {
            smoothedFrequency = rawFreq;
          } else {
            smoothedFrequency = smoothedFrequency * (1.0f - smoothingAlpha) + rawFreq * smoothingAlpha;
          }
          
          // Auto-range based on detected frequency for next measurement
          if (smoothedFrequency < 150) {
            tunerAutoRange = 1;  // LOW - slower sample rate for bass
          } else if (smoothedFrequency < 600) {
            tunerAutoRange = 2;  // MID - balanced
          } else {
            tunerAutoRange = 3;  // HI - faster sample rate for treble
          }
        } else {
          smoothedFrequency *= 0.9f;
          // If no signal, reset to mid-range for next detection
          if (smoothedFrequency < 20) {
            tunerAutoRange = 2;
          }
        }
        
        lastUpdate = millis();
        state = 0;
      }
      break;
  }

  // Update display at ~15fps
  static unsigned long lastDraw = 0;
  if (millis() - lastDraw < 66) return;
  lastDraw = millis();
  
  display.clearDisplay();
  
  // Calculate vertical offset based on whether params are shown
  int yOffset = showParams ? 10 : 0;
  
  if (smoothedFrequency > 30) {
    char note[3];
    int8_t octave, cents;
    frequencyToNote(smoothedFrequency, note, &octave, &cents);
    
    // Calculate note width for centering (size 2 = 12px per char)
    uint8_t noteLen = strlen(note);
    int noteWidth = (noteLen * 12) + 12;  // note chars + octave
    int noteX = (128 - noteWidth) / 2;
    
    // Draw centered note (size 2)
    display.setTextSize(2);
    display.setCursor(noteX, yOffset + 6);
    display.print(note);
    display.print(octave);
    
    // Draw cents indicator - left if negative, right if positive (with margin)
    display.setTextSize(1);
    if (cents < 0) {
      // Left side with margin
      display.setCursor(4, yOffset + 10);
      display.print(cents);
      display.print('c');
    } else if (cents > 0) {
      // Right side with margin
      int centsX = (cents < 10) ? 110 : 104;
      display.setCursor(centsX, yOffset + 10);
      display.print('+');
      display.print(cents);
      display.print('c');
    } else {
      // Centered "OK" indicator when in tune
      display.setTextSize(1);
      display.setCursor(110, yOffset + 10);
      display.print(F("OK"));
    }
    
    // Draw Hz below note
    display.setTextSize(1);
    char hzStr[10];
    if (smoothedFrequency < 100) {
      dtostrf(smoothedFrequency, 4, 1, hzStr);
    } else if (smoothedFrequency < 1000) {
      dtostrf(smoothedFrequency, 3, 0, hzStr);
    } else {
      dtostrf(smoothedFrequency, 4, 0, hzStr);
    }
    strcat(hzStr, "Hz");
    int hzWidth = strlen(hzStr) * 6;
    int hzX = (128 - hzWidth) / 2;
    display.setCursor(hzX, yOffset + 26);
    display.print(hzStr);
    
    // Tuning indicator bar at bottom
    int barY = showParams ? 44 : 42;
    display.drawRect(14, barY, 100, 6, WHITE);
    display.drawFastVLine(64, barY - 2, 10, WHITE);  // Center marker
    
    // Indicator position based on cents (-50 to +50 mapped to bar width)
    int indicator = 64 + constrain(cents, -50, 50);
    display.fillRect(indicator - 2, barY + 2, 5, 6, WHITE);
    
  } else {
    // No signal detected
    display.setTextSize(2);
    display.setCursor(40, yOffset + 10);
    display.print(F("---"));
    
    display.setTextSize(1);
    display.setCursor(40, yOffset + 28);
    display.print(F("No signal"));
    
    // Empty tuning bar
    int barY = showParams ? 54 : 52;
    display.drawRect(14, barY, 100, 6, WHITE);
    display.drawFastVLine(64, barY - 2, 10, WHITE);
  }
  
  display.setTextSize(1);
  if (showParams) drawParameterBar(true);
}

// ---------------- Parameter bar ----------------
void drawParameterBar(bool showParams) {
  if (!showParams) return;
  display.setTextSize(1);
  display.drawLine((param - 1) * 42, 8, (param - 1) * 42 + 36, 8, WHITE);

  display.setTextColor((param_select == 1) ? BLACK : WHITE, (param_select == 1) ? WHITE : BLACK);
  display.setCursor(0, 0);
  switch (mode) {
    case MODE_LFO:      display.print(F("LFO")); break;
    case MODE_WAVE:     display.print(F("WAVE")); break;
    case MODE_SHOT:     display.print(F("SHOT")); break;
    case MODE_SPECTRUM: display.print(F("SPEC")); break;
    case MODE_TUNER:    display.print(F("TUNE")); break;
  }

  display.setTextColor((param_select == 2) ? BLACK : WHITE, (param_select == 2) ? WHITE : BLACK);
  display.setCursor(42, 0);
  switch (mode) {
    case MODE_LFO:
    case MODE_WAVE:
    case MODE_SHOT:    display.print(F("Time:")); break;
    case MODE_SPECTRUM:display.print(F("High:")); break;
    case MODE_TUNER:
      // Show auto-range indicator in param1 slot
      switch (tunerAutoRange) {
        case 1: display.print(F("LO")); break;
        case 2: display.print(F("MID")); break;
        case 3: display.print(F("HI")); break;
      }
      break;
  }
  if (mode != MODE_TUNER) {
    display.setCursor(72, 0); display.print(param1);
  }

  if (mode == MODE_SHOT) {
    display.setTextColor(trig ? BLACK : WHITE, trig ? WHITE : BLACK);
    display.setCursor(84, 0); display.print(F("TRIG"));
  } else if (mode != MODE_TUNER) {
    display.setTextColor((param_select == 3) ? BLACK : WHITE, (param_select == 3) ? WHITE : BLACK);
    display.setCursor(84, 0);
    switch (mode) {
      case MODE_LFO:     display.print(F("Offs:")); break;
      case MODE_WAVE:    display.print(F("Rfrs:")); break;
      case MODE_SPECTRUM:display.print(F("Filt:")); break;
    }
    display.setCursor(114, 0); display.print(param2);
  }
}

// ---------------- Config menu ----------------
void configMenu() {
  int newDirection = encoderDirection;
  newPosition = encoder.read();

  if (newPosition > (oldPosition + 3)) {
    oldPosition = newPosition;
    if (configMenuOption == 1) newDirection = 1;
    else if (configMenuOption == 2 && menuTimer > 1) menuTimer--;
    else if (configMenuOption == 3) { oledRotation = 0; display.setRotation(0); }
    else if (configMenuOption == 4) bootLogoEnabled = 1;
  } else if (newPosition < (oldPosition - 3)) {
    oldPosition = newPosition;
    if (configMenuOption == 1) newDirection = -1;
    else if (configMenuOption == 2 && menuTimer < 60) menuTimer++;
    else if (configMenuOption == 3) { oledRotation = 2; display.setRotation(2); }
    else if (configMenuOption == 4) bootLogoEnabled = 0;
  }

  if (newDirection != encoderDirection) encoderDirection = newDirection;
  
  // Short press cycles through options
  if (old_SW == 0 && SW == 1) configMenuOption = (configMenuOption % 4) + 1;

  // Track hold duration for exit
  static unsigned long holdStart = 0;
  if (SW && !old_SW) holdStart = millis();
  
  // Hold 2 seconds to save and exit
  if (SW && holdStart > 0 && (millis() - holdStart >= 2000)) {
    // Save all config settings
    EEPROM.put(ENCODER_DIR_ADDR, encoderDirection);
    EEPROM.write(OLED_ROT_ADDR, oledRotation);
    EEPROM.put(MENUTIMER_DIR_ADDR, (uint8_t)menuTimer);
    EEPROM.write(BOOTLOGO_ADDR, bootLogoEnabled);
    
    // Also save all mode settings
    saveCurrentModeToRAM();
    saveAllSettings();

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(16, 25);
    display.print(F("SETTINGS SAVED"));
    display.display();
    delay(800);

    configMenuActive = false;
    holdStart = 0;
    oldPosition = newPosition = encoder.read() * encoderDirection;
    return;
  }
  
  if (!SW) holdStart = 0;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.println(F("GLOBAL SETTINGS"));
  display.println();

  display.setTextColor(configMenuOption == 1 ? BLACK : WHITE, configMenuOption == 1 ? WHITE : BLACK);
  display.print(F("Encoder: "));
  display.setTextColor(WHITE);
  display.println(encoderDirection == 1 ? F("Normal") : F("Reversed"));

  display.setTextColor(configMenuOption == 2 ? BLACK : WHITE, configMenuOption == 2 ? WHITE : BLACK);
  display.print(F("Timer: "));
  display.setTextColor(WHITE);
  display.print(menuTimer); display.println(F("s"));

  display.setTextColor(configMenuOption == 3 ? BLACK : WHITE, configMenuOption == 3 ? WHITE : BLACK);
  display.print(F("OLED: "));
  display.setTextColor(WHITE);
  display.println(oledRotation == 0 ? F("0 deg") : F("180 deg"));

  display.setTextColor(configMenuOption == 4 ? BLACK : WHITE, configMenuOption == 4 ? WHITE : BLACK);
  display.print(F("Boot Logo: "));
  display.setTextColor(WHITE);
  display.println(bootLogoEnabled ? F("On") : F("Off"));

  display.setCursor(4, 56);
  display.print(F("Hold 2s to save+exit"));

  display.display();
}
