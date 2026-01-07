/**
 * @file SCOPE.ino
 * @author Modulove
 * @brief Eurorack scope + Tuner (Software SPI version)
 * @date 2026-01-17
 * 
 * MODIFICATIONS v2026
 * - Added waveform visualization below tuner bar
 * - Auto-scales waveform 
 * - use fastAnalogRead() in LFO mode (faster?)
 * - Removed SHOT mode - now 4 modes: LFO, WAVE, SPECTRUM, TUNER
 * - Reduced code size and RAM usage
 */

#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "fix_fft.h"
#include <Encoder.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Display (Software SPI) ----------------
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
#define MODE_SPECTRUM 3
#define MODE_TUNER    4
#define NUM_MODES     4

// ---------------- ADC Sampling System ----------------
#define ADC_BUFFER_SIZE   256

volatile uint16_t adcSampleIndex = 0;
volatile uint16_t adcTargetSamples = 128;
volatile bool adcBufferReady = false;
volatile bool adcSampling = false;
volatile uint8_t adcDelayCounter = 0;
volatile uint8_t adcDelayTarget = 0;

// Tuner state
uint32_t smoothedFreqX100 = 0;  // Fixed-point frequency * 100
uint32_t lastValidFreqX100 = 0;
uint8_t tunerSampleRate = 2;    // 1=slow(4.8kHz), 2=med(9.6kHz), 3=fast(19kHz)

// Wave mode state
uint8_t waveState = 0;
unsigned long waveLastUpdate = 0;
uint16_t waveDetectedFreq = 0;  // Integer Hz
uint8_t waveAutoPrescaler = 0x05;
uint8_t waveAutoDelay = 0;

// Spectrum mode state
uint8_t spectrumState = 0;
unsigned long spectrumLastUpdate = 0;

// Tuner mode state
uint8_t tunerState = 0;
unsigned long tunerLastUpdate = 0;

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

// ---------------- Shared buffer ----------------
union {
  struct { int8_t real[128]; int8_t imag[128]; } spectrum;
  uint8_t raw[256];
} buffer;

// ---------------- Mode Settings Storage ----------------
struct ModeSettings {
  uint8_t param_select;
  uint8_t param1;
  int8_t param2;  // Signed for offset values
  int8_t param3;  // Additional parameter (Wave mode Rfrs)
};
ModeSettings modeSettings[NUM_MODES];

uint8_t mode = MODE_LFO;
uint8_t old_mode = MODE_LFO;
uint8_t param_select = 0;
uint8_t param = 1;
uint8_t param1 = 2;
int8_t param2 = 1;  // Signed for offset values
int8_t param3 = 1;  // Additional parameter

bool SW = 0, old_SW = 0;

unsigned long hideTimer = 0;
unsigned long lastSaveTime = 0;
bool hide = 0;
int rfrs = 0;

int16_t oldPosition = -999;
int16_t newPosition = -999;

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
void runSpectrumMode(bool showParams);
void runTunerMode(bool showParams);
void drawParameterBar(bool showParams);
void configMenu();
void saveCurrentModeToRAM(uint8_t m);
void saveAllSettings();
void loadAllSettings();
void startADCSampling(uint16_t numSamples, uint8_t prescaler, uint8_t delaySkip);
void stopADCSampling();
uint8_t fastAnalogRead();
uint32_t detectFrequencyZC(uint16_t sampleRateX10);  // Returns freq * 100

// ================== ADC Interrupt ==================
ISR(ADC_vect) {
  if (!adcSampling) return;
  
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

void startADCSampling(uint16_t numSamples, uint8_t prescaler, uint8_t delaySkip) {
  cli();
  
  adcSampleIndex = 0;
  adcTargetSamples = min(numSamples, (uint16_t)ADC_BUFFER_SIZE);
  adcBufferReady = false;
  adcSampling = true;
  adcDelayCounter = 0;
  adcDelayTarget = delaySkip;
  
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (ANALOG_INPUT_PIN & 0x07);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (prescaler & 0x07);
  ADCSRB = 0;
  
  sei();
}

void stopADCSampling() {
  ADCSRA &= ~((1 << ADIE) | (1 << ADATE));
  adcSampling = false;
}

// Fast single ADC read (8-bit, ~15µs vs ~100µs for analogRead)
uint8_t fastAnalogRead() {
  // Set up: AVcc ref, left-adjust, channel A0
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (ANALOG_INPUT_PIN & 0x07);
  // Enable ADC, start conversion, prescaler 64 (~15µs conversion)
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (0x06);  // Prescaler 64
  // Wait for conversion
  while (ADCSRA & (1 << ADSC));
  return ADCH;
}

// ================== Buffer-Based Zero-Crossing Detection ==================
// Returns frequency * 100 (fixed-point), sampleRateX10 = sample rate / 10
uint32_t detectFrequencyZC(uint16_t sampleRateX10) {
  uint8_t minVal = 255, maxVal = 0;
  for (uint16_t i = 0; i < 256; i++) {
    if (buffer.raw[i] < minVal) minVal = buffer.raw[i];
    if (buffer.raw[i] > maxVal) maxVal = buffer.raw[i];
  }
  
  uint8_t range = maxVal - minVal;
  if (range < 30) return 0;
  
  uint8_t center = (minVal + maxVal) / 2;
  uint8_t hysteresis = range / 6;
  if (hysteresis < 5) hysteresis = 5;
  
  uint8_t threshHigh = center + hysteresis;
  uint8_t threshLow = center - hysteresis;
  
  uint8_t crossings = 0;
  uint16_t firstCrossing = 0;
  uint16_t lastCrossing = 0;
  bool aboveCenter = buffer.raw[0] > center;
  
  for (uint16_t i = 1; i < 256; i++) {
    if (!aboveCenter && buffer.raw[i] > threshHigh) {
      aboveCenter = true;
      crossings++;
      
      if (firstCrossing == 0) {
        firstCrossing = i;
      }
      lastCrossing = i;
    }
    else if (aboveCenter && buffer.raw[i] < threshLow) {
      aboveCenter = false;
    }
  }
  
  if (crossings < 2) return 0;
  
  uint16_t totalSamples = lastCrossing - firstCrossing;
  
  // freq * 100 = (sampleRate * 100) / avgPeriod
  // = (sampleRateX10 * 10 * 100 * (crossings-1)) / totalSamples
  // = (sampleRateX10 * 1000 * (crossings-1)) / totalSamples
  uint32_t freqX100 = ((uint32_t)sampleRateX10 * 1000UL * (crossings - 1)) / totalSamples;
  
  return freqX100;
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
  if (bootLogoEnabled > 1) bootLogoEnabled = 1;

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

  loadAllSettings();
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
      saveCurrentModeToRAM(mode);
      saveAllSettings();
      hasSaved = true;
    }
  }

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

  // Wave mode has 4 params, others have 3
  uint8_t maxParam = (mode == MODE_WAVE) ? 4 : 3;

  if (old_SW == 0 && SW == 1 && param_select == param) { param_select = 0; hideTimer = millis(); }
  else if (old_SW == 0 && SW == 1 && (param >= 1 && param <= maxParam)) { param_select = param; hideTimer = millis(); }

  mode = constrain(mode, 1, NUM_MODES);
  param = constrain(param, 1, maxParam);

  newPosition = encoderDirection * encoder.read();
  if ((newPosition - 3) / 4 > oldPosition / 4) {
    oldPosition = newPosition; hideTimer = millis();
    switch (param_select) {
      case 0: param--; break;
      case 1: mode--;  break;
      case 2: param1--;break;
      case 3: param2--;break;
      case 4: param3--;break;
    }
  } else if ((newPosition + 3) / 4 < oldPosition / 4) {
    oldPosition = newPosition; hideTimer = millis();
    switch (param_select) {
      case 0: param++; break;
      case 1: mode++;  break;
      case 2: param1++;break;
      case 3: param2++;break;
      case 4: param3++;break;
    }
  }

  // Parameter wrapping
  mode = constrain(mode, 1, NUM_MODES);
  if (mode != MODE_WAVE && param > 3) param = 3;
  
  switch (mode) {
    case MODE_LFO:
      if (param1 < 1) param1 = 8;
      if (param1 > 8) param1 = 1;
      if (param2 < -6) param2 = 10;
      if (param2 > 10) param2 = -6;
      break;
    case MODE_WAVE:
      // param1: 1-9 (1-8 = manual, 9 = AUTO)
      if (param1 < 1) param1 = 9;
      if (param1 > 9) param1 = 1;
      // param2: -6 to 10 (offset)
      if (param2 < -6) param2 = 10;
      if (param2 > 10) param2 = -6;
      // param3: 1-6 (refresh rate)
      if (param3 < 1) param3 = 6;
      if (param3 > 6) param3 = 1;
      break;
    case MODE_SPECTRUM:
      if (param1 < 1) param1 = 4;
      if (param1 > 4) param1 = 1;
      if (param2 < 1) param2 = 8;
      if (param2 > 8) param2 = 1;
      break;
    case MODE_TUNER:
      // No adjustable parameters - ZC only
      break;
  }

  byte currentParamSelect = param_select;
  if (old_mode != mode) {
    saveCurrentModeToRAM(old_mode);  // Save OLD mode's settings before switching
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

void saveCurrentModeToRAM(uint8_t m) {
  uint8_t idx = m - 1;
  if (idx < NUM_MODES) {
    modeSettings[idx].param_select = param_select;
    modeSettings[idx].param1 = param1;
    modeSettings[idx].param2 = param2;
    modeSettings[idx].param3 = param3;
  }
}

void loadAllSettings() {
  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int baseAddr = EEPROM_PARAM_SELECT_ADDR + (m * 4);  // 4 bytes per mode now
    modeSettings[m].param_select = EEPROM.read(baseAddr);
    modeSettings[m].param1 = EEPROM.read(baseAddr + 1);
    modeSettings[m].param2 = (int8_t)EEPROM.read(baseAddr + 2);
    modeSettings[m].param3 = (int8_t)EEPROM.read(baseAddr + 3);
    
    // Validate param_select based on mode
    uint8_t maxParamSelect = (m + 1 == MODE_WAVE) ? 4 : 3;
    if (modeSettings[m].param_select > maxParamSelect) modeSettings[m].param_select = 0;
    
    switch (m + 1) {
      case MODE_LFO:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 8);
        modeSettings[m].param2 = constrain(modeSettings[m].param2, -6, 10);
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 4;
        if (modeSettings[m].param2 == 0) modeSettings[m].param2 = 1;
        modeSettings[m].param3 = 1;
        break;
      case MODE_WAVE:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 9);  // 9 = AUTO
        modeSettings[m].param2 = constrain(modeSettings[m].param2, -6, 10);
        modeSettings[m].param3 = constrain(modeSettings[m].param3, 1, 6);
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 8;
        if (modeSettings[m].param2 == 0) modeSettings[m].param2 = 1;
        if (modeSettings[m].param3 == 0) modeSettings[m].param3 = 1;
        break;
      case MODE_SPECTRUM:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 4);
        modeSettings[m].param2 = constrain(modeSettings[m].param2, 1, 8);
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 1;
        if (modeSettings[m].param2 == 0) modeSettings[m].param2 = 8;
        modeSettings[m].param3 = 1;
        break;
      case MODE_TUNER:
        // No adjustable parameters - ZC only
        modeSettings[m].param1 = 1;
        modeSettings[m].param2 = 1;
        modeSettings[m].param3 = 1;
        break;
    }
  }
}

void saveAllSettings() {
  EEPROM.update(EEPROM_MODE_ADDR, mode);
  
  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int baseAddr = EEPROM_PARAM_SELECT_ADDR + (m * 4);  // 4 bytes per mode now
    EEPROM.update(baseAddr, modeSettings[m].param_select);
    EEPROM.update(baseAddr + 1, modeSettings[m].param1);
    EEPROM.update(baseAddr + 2, modeSettings[m].param2);
    EEPROM.update(baseAddr + 3, modeSettings[m].param3);
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
  
  uint8_t idx = m - 1;
  if (idx < NUM_MODES) {
    param_select = modeSettings[idx].param_select;
    param1 = modeSettings[idx].param1;
    param2 = modeSettings[idx].param2;
    param3 = modeSettings[idx].param3;
  }

  switch (m) {
    case MODE_LFO:
      pinMode(FILTER_PIN, INPUT);
      analogWrite(OFFSET_PIN, 0);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      break;

    case MODE_WAVE:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      // Reset wave mode state machine
      waveState = 0;
      waveLastUpdate = 0;
      waveDetectedFreq = 0;
      waveAutoPrescaler = 0x05;
      waveAutoDelay = 4;
      break;
      
    case MODE_SPECTRUM:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, OUTPUT);
      digitalWrite(FILTER_PIN, LOW);
      // Reset spectrum state machine
      spectrumState = 0;
      spectrumLastUpdate = 0;
      break;

    case MODE_TUNER:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      smoothedFreqX100 = 0;
      lastValidFreqX100 = 0;
      tunerSampleRate = 2;
      tunerState = 0;
      tunerLastUpdate = 0;
      break;
  }

  memset(&buffer, 0, sizeof(buffer));
  rfrs = 0;
}

// ---------------- LFO Mode ----------------
void runLFOMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);
  param2 = constrain(param2, -6, 10);

  uint8_t currentSample = fastAnalogRead() >> 2;  // 8-bit to 6-bit (0-63)

  memmove(&buffer.raw[1], &buffer.raw[0], 127);
  buffer.raw[0] = currentSample;

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 30) {
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

// ---------------- Wave Mode ----------------
// param1: 1-8 = manual timebase, 9 = AUTO
// param2: -6 to 10 = vertical offset
// param3: 1-6 = refresh rate

void runWaveMode(bool showParams) {
  param  = constrain(param, 1, 4);  // 4 menu items in Wave mode
  param1 = constrain(param1, 1, 9);  // 9 = AUTO mode
  param2 = constrain(param2, -6, 10);  // Offset
  param3 = constrain(param3, 1, 6);  // Refresh rate

  unsigned long interval = 20UL + (param3 - 1) * 10UL;  // 20-70ms based on Rfrs
  bool autoMode = (param1 == 9);
  int voff = (param2 - 1) * 4;  // Vertical offset (scaled for 64px height)
  
  switch (waveState) {
    case 0:  // Start sampling
      if (millis() - waveLastUpdate >= interval) {
        uint8_t prescaler, delaySkip;
        
        if (autoMode) {
          // Use auto-calculated values
          prescaler = waveAutoPrescaler;
          delaySkip = waveAutoDelay;
        } else {
          // Manual timebase: param1 1-8 controls speed
          if (param1 <= 5) {
            prescaler = 0x05;  // /32 = fast
            delaySkip = (6 - param1) * 2;
          } else {
            prescaler = 0x06;  // /64 = slower
            delaySkip = (param1 - 5) * 4;
          }
        }
        startADCSampling(128, prescaler, delaySkip);
        waveState = 1;
      }
      break;
      
    case 1:  // Wait for buffer
      if (adcBufferReady) {
        waveState = 2;
      }
      break;
      
    case 2:  // Process and display
      {
        // Find trigger point (rising edge through center)
        uint8_t minVal = 255, maxVal = 0;
        for (uint8_t i = 0; i < 128; i++) {
          if (buffer.raw[i] < minVal) minVal = buffer.raw[i];
          if (buffer.raw[i] > maxVal) maxVal = buffer.raw[i];
        }
        
        uint8_t center = (minVal + maxVal) / 2;
        uint8_t triggerIdx = 0;
        
        // Look for rising edge crossing center
        uint8_t hysteresis = (maxVal - minVal) / 8;
        if (hysteresis < 3) hysteresis = 3;
        uint8_t threshLow = center - hysteresis;
        uint8_t threshHigh = center + hysteresis;
        
        bool belowThresh = buffer.raw[0] < threshLow;
        for (uint8_t i = 1; i < 64; i++) {
          if (belowThresh && buffer.raw[i] > threshHigh) {
            triggerIdx = i;
            break;
          }
          if (buffer.raw[i] < threshLow) belowThresh = true;
        }
        
        // AUTO mode frequency detection and timebase adjustment
        if (autoMode) {
          // Count zero crossings to estimate frequency
          uint8_t crossings = 0;
          uint8_t firstCross = 0, lastCross = 0;
          bool above = buffer.raw[0] > center;
          
          for (uint8_t i = 1; i < 128; i++) {
            if (!above && buffer.raw[i] > threshHigh) {
              above = true;
              crossings++;
              if (firstCross == 0) firstCross = i;
              lastCross = i;
            } else if (above && buffer.raw[i] < threshLow) {
              above = false;
            }
          }
          
          // Calculate sample rate x10 based on current settings
          // Base rates: 0x05=38461, 0x06=19230, 0x07=9615
          uint16_t sampleRateX10;
          if (waveAutoPrescaler == 0x05) {
            sampleRateX10 = 38461 / (1 + waveAutoDelay);  // Approximation
          } else if (waveAutoPrescaler == 0x06) {
            sampleRateX10 = 19230 / (1 + waveAutoDelay);
          } else {
            sampleRateX10 = 9615 / (1 + waveAutoDelay);
          }
          
          if (crossings >= 2) {
            // avgPeriod = (lastCross - firstCross) / (crossings - 1)
            // freq = sampleRate / avgPeriod
            // freq = sampleRate * (crossings-1) / (lastCross - firstCross)
            uint16_t periodSamples = lastCross - firstCross;
            waveDetectedFreq = ((uint32_t)sampleRateX10 * (crossings - 1)) / periodSamples;
            
            // Target: 1.5-2.5 cycles visible (48-80 samples per cycle)
            // samplesPerCycle = periodSamples / (crossings - 1)
            uint16_t samplesPerCycle = periodSamples / (crossings - 1);
            
            if (samplesPerCycle < 30) {
              // Too fast - need slower sample rate
              if (waveAutoDelay < 15) {
                waveAutoDelay++;
              } else if (waveAutoPrescaler < 0x07) {
                waveAutoPrescaler++;
                waveAutoDelay = 0;
              }
            } else if (samplesPerCycle > 90) {
              // Too slow - need faster sample rate
              if (waveAutoDelay > 0) {
                waveAutoDelay--;
              } else if (waveAutoPrescaler > 0x05) {
                waveAutoPrescaler--;
                waveAutoDelay = 8;
              }
            }
          } else if (crossings < 1) {
            // No crossings detected - signal too slow, slow down sampling
            if (waveAutoDelay < 15) {
              waveAutoDelay += 2;
            } else if (waveAutoPrescaler < 0x07) {
              waveAutoPrescaler++;
              waveAutoDelay = 4;
            }
            waveDetectedFreq = 0;
          }
        }
        
        // Scale samples to display height
        display.clearDisplay();
        
        for (uint8_t i = 0; i < 128; i++) {
          buffer.raw[i] = buffer.raw[i] >> 2;  // Scale to 0-63
        }
        
        // Draw waveform from trigger point with offset
        uint8_t maxDraw = 127 - triggerIdx;
        if (maxDraw > 127) maxDraw = 127;
        
        for (uint8_t i = 0; i < maxDraw; i++) {
          uint8_t idx1 = triggerIdx + i;
          uint8_t idx2 = triggerIdx + i + 1;
          if (idx2 < 128) {
            int y1 = constrain(buffer.raw[idx1] + voff, 0, 63);
            int y2 = constrain(buffer.raw[idx2] + voff, 0, 63);
            display.drawLine(i, y1, i + 1, y2, WHITE);
          }
        }
        
        // Show frequency in AUTO mode
        if (autoMode && waveDetectedFreq > 1) {
          display.setTextSize(1);
          display.setTextColor(WHITE, BLACK);
          display.setCursor(80, 56);
          if (waveDetectedFreq < 1000) {
            display.print(waveDetectedFreq);
            display.print(F("Hz"));
          } else {
            display.print(waveDetectedFreq / 1000);
            display.print('.');
            display.print((waveDetectedFreq % 1000) / 100);
            display.print(F("k"));
          }
        }
        
        if (showParams) drawParameterBar(true);
        waveLastUpdate = millis();
        waveState = 0;
      }
      break;
  }
}

// ---------------- Spectrum Mode ----------------
void runSpectrumMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 4);
  param2 = constrain(param2, 1, 8);
  
  switch (spectrumState) {
    case 0:
      if (millis() - spectrumLastUpdate >= 50) {
        startADCSampling(128, 0x06, 0);
        spectrumState = 1;
      }
      break;
      
    case 1:
      if (adcBufferReady) {
        spectrumState = 2;
      }
      break;
      
    case 2:
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
      spectrumLastUpdate = millis();
      spectrumState = 0;
      break;
  }
}

// ================== TUNER MODE ==================

// Note frequencies * 100 for octave 4 (C4 to B4) - stored in PROGMEM
const uint16_t noteFreqX100[] PROGMEM = {
  26163,  // C4  = 261.63 Hz
  27718,  // C#4 = 277.18 Hz
  29366,  // D4  = 293.66 Hz
  31113,  // D#4 = 311.13 Hz
  32963,  // E4  = 329.63 Hz
  34923,  // F4  = 349.23 Hz
  36999,  // F#4 = 369.99 Hz
  39200,  // G4  = 392.00 Hz
  41530,  // G#4 = 415.30 Hz
  44000,  // A4  = 440.00 Hz
  46616,  // A#4 = 466.16 Hz
  49388   // B4  = 493.88 Hz
};

// Convert frequency (x100) to note name, octave, and cents deviation
void frequencyToNote(uint32_t freqX100, char* note, int8_t* octave, int8_t* cents) {
  static const char noteNames[] PROGMEM = "C C#D D#E F F#G G#A A#B ";
  
  if (freqX100 < 1635) {  // Below C0 (~16.35 Hz)
    note[0] = '-'; note[1] = '\0';
    *octave = 0; *cents = 0;
    return;
  }
  
  // Normalize to octave 4 (C4=261.63 to B4=493.88, freqX100: 26163-49388)
  *octave = 4;
  uint32_t normFreq = freqX100;
  
  while (normFreq < 26163 && *octave > 0) {
    normFreq *= 2;
    (*octave)--;
  }
  while (normFreq >= 52325 && *octave < 8) {  // 52325 = C5 * 100
    normFreq /= 2;
    (*octave)++;
  }
  
  // Find closest note by linear search
  uint8_t closestNote = 0;
  uint32_t minDiff = 0xFFFFFFFF;
  
  for (uint8_t i = 0; i < 12; i++) {
    uint16_t noteFreq = pgm_read_word(&noteFreqX100[i]);
    uint32_t diff = (normFreq > noteFreq) ? (normFreq - noteFreq) : (noteFreq - normFreq);
    if (diff < minDiff) {
      minDiff = diff;
      closestNote = i;
    }
  }
  
  // Get note name
  uint8_t idx = closestNote * 2;
  note[0] = pgm_read_byte(&noteNames[idx]);
  note[1] = pgm_read_byte(&noteNames[idx + 1]);
  note[2] = '\0';
  if (note[1] == ' ') note[1] = '\0';
  
  // Calculate cents deviation: cents = 1200 * log2(actual/target)
  // Approximation: cents ≈ (actual - target) * 1731 / target
  // (1731 ≈ 1200 / ln(2) * 0.01 for small deviations)
  uint16_t targetFreq = pgm_read_word(&noteFreqX100[closestNote]);
  int32_t diff = (int32_t)normFreq - (int32_t)targetFreq;
  *cents = (int8_t)((diff * 173L) / (int32_t)targetFreq);
  *cents = constrain(*cents, -50, 50);
}

void runTunerMode(bool showParams) {
  param = constrain(param, 1, 2);
  
  // Sample rate selection: sampleRateX10 values
  // 1=slow: 4808/10=481, 2=med: 9615/10=962, 3=fast: 19230/10=1923
  static const uint16_t sampleRates[] = {481, 962, 1923};
  
  switch (tunerState) {
    case 0:
      if (millis() - tunerLastUpdate >= 40) {
        uint8_t prescaler;
        uint8_t delaySkip;
        
        // Adaptive sample rate based on last detected frequency
        if (lastValidFreqX100 < 8000 || tunerSampleRate == 1) {  // < 80 Hz
          prescaler = 0x07;
          delaySkip = 1;
          tunerSampleRate = 1;
        }
        else if (lastValidFreqX100 < 20000 || tunerSampleRate == 2) {  // < 200 Hz
          prescaler = 0x07;
          delaySkip = 0;
          tunerSampleRate = 2;
        }
        else {
          prescaler = 0x06;
          delaySkip = 0;
          tunerSampleRate = 3;
        }
        
        startADCSampling(256, prescaler, delaySkip);
        tunerState = 1;
      }
      break;
      
    case 1:
      if (adcBufferReady) {
        tunerState = 2;
      }
      break;
      
    case 2:
      {
        uint32_t rawFreqX100 = detectFrequencyZC(sampleRates[tunerSampleRate - 1]);
        
        if (rawFreqX100 > 1500 && rawFreqX100 < 500000) {  // 15-5000 Hz
          lastValidFreqX100 = rawFreqX100;
          
          // Smoothing with fixed-point
          if (smoothedFreqX100 < 1000) {
            smoothedFreqX100 = rawFreqX100;
          } else {
            // Calculate difference percentage (x100)
            uint32_t diffPercent = 0;
            if (rawFreqX100 > smoothedFreqX100) {
              diffPercent = ((rawFreqX100 - smoothedFreqX100) * 100) / smoothedFreqX100;
            } else {
              diffPercent = ((smoothedFreqX100 - rawFreqX100) * 100) / smoothedFreqX100;
            }
            
            // alpha = 50% for large changes, 35% for small changes
            uint8_t alpha = (diffPercent > 10) ? 50 : 35;
            smoothedFreqX100 = (smoothedFreqX100 * (100 - alpha) + rawFreqX100 * alpha) / 100;
          }
          
          // Adjust sample rate for next reading
          if (smoothedFreqX100 < 6000) {
            tunerSampleRate = 1;
          } else if (smoothedFreqX100 < 15000) {
            tunerSampleRate = 2;
          } else {
            tunerSampleRate = 3;
          }
        } else {
          // Decay smoothed value when no signal
          smoothedFreqX100 = (smoothedFreqX100 * 85) / 100;
          if (smoothedFreqX100 < 1500) {
            smoothedFreqX100 = 0;
            tunerSampleRate = 2;
          }
        }
        
        tunerLastUpdate = millis();
        tunerState = 0;
      }
      break;
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw < 80) return;
  lastDraw = millis();
  
  display.clearDisplay();
  
  int yOff = showParams ? 10 : 0;
  
  if (smoothedFreqX100 > 1500) {
    char note[3];
    int8_t octave, cents;
    frequencyToNote(smoothedFreqX100, note, &octave, &cents);
    
    uint8_t noteLen = strlen(note);
    int noteWidth = (noteLen * 12) + 12;
    int noteX = (128 - noteWidth) / 2;
    
    display.setTextSize(2);
    display.setCursor(noteX, yOff + 4);
    display.print(note);
    display.print(octave);
    
    display.setTextSize(1);
    if (cents < 0) {
      display.setCursor(4, yOff + 8);
      display.print(cents);
      display.print('c');
    } else if (cents > 0) {
      int centsX = (cents < 10) ? 110 : 104;
      display.setCursor(centsX, yOff + 8);
      display.print('+');
      display.print(cents);
      display.print('c');
    } else {
      display.setCursor(110, yOff + 8);
      display.print(F("OK"));
    }
    
    // Display frequency
    display.setTextSize(1);
    uint16_t freqHz = smoothedFreqX100 / 100;
    uint8_t freqDec = (smoothedFreqX100 % 100) / 10;
    
    char hzStr[12];
    if (freqHz < 100) {
      sprintf(hzStr, "%d.%dHz", freqHz, freqDec);
    } else {
      sprintf(hzStr, "%dHz", freqHz);
    }
    int hzWidth = strlen(hzStr) * 6;
    int hzX = (128 - hzWidth) / 2;
    display.setCursor(hzX, yOff + 24);
    display.print(hzStr);
    
    // Tuning bar (moved up to make room for waveform)
    int barY = showParams ? 44 : 34;
    
    // Draw bar
    display.drawRect(14, barY, 100, 6, WHITE);
    display.drawFastVLine(64, barY - 2, 10, WHITE);
    
    // Draw tuning indicator
    int indicator = 64 + constrain(cents, -50, 50);
    display.fillRect(indicator - 2, barY + 1, 5, 4, WHITE);
    
    // Draw waveform below bar (full width, reuse ADC buffer)
    int waveY = barY + 8;  // 2 pixel gap below bar
    uint8_t waveH = showParams ? 10 : 14;  // Height available
    
    // Find min/max for scaling (use first 128 samples)
    uint8_t wMin = 255, wMax = 0;
    for (uint8_t i = 0; i < 128; i++) {
      if (buffer.raw[i] < wMin) wMin = buffer.raw[i];
      if (buffer.raw[i] > wMax) wMax = buffer.raw[i];
    }
    uint8_t wRange = wMax - wMin;
    if (wRange < 10) wRange = 10;  // Minimum range
    
    // Draw waveform as connected lines (128 pixels, 1:1 mapping)
    for (uint8_t i = 0; i < 127; i++) {
      uint8_t y1 = waveH - 1 - (((uint16_t)(buffer.raw[i] - wMin) * (waveH - 1)) / wRange);
      uint8_t y2 = waveH - 1 - (((uint16_t)(buffer.raw[i + 1] - wMin) * (waveH - 1)) / wRange);
      display.drawLine(i, waveY + y1, i + 1, waveY + y2, WHITE);
    }
    
  } else {
    display.setTextSize(2);
    display.setCursor(44, yOff + 8);
    display.print(F("---"));
    
    display.setTextSize(1);
    display.setCursor(34, yOff + 26);
    display.print(F("No signal"));
    
    // Tuning bar
    int barY = showParams ? 44 : 34;
    display.drawRect(14, barY, 100, 6, WHITE);
    display.drawFastVLine(64, barY - 2, 10, WHITE);
    
    // Draw waveform below bar (full width, shows noise/input activity)
    int waveY = barY + 8;
    uint8_t waveH = showParams ? 10 : 14;
    
    // Find min/max for scaling (use first 128 samples)
    uint8_t wMin = 255, wMax = 0;
    for (uint8_t i = 0; i < 128; i++) {
      if (buffer.raw[i] < wMin) wMin = buffer.raw[i];
      if (buffer.raw[i] > wMax) wMax = buffer.raw[i];
    }
    uint8_t wRange = wMax - wMin;
    if (wRange < 10) wRange = 10;
    
    // Draw waveform (128 pixels, 1:1 mapping)
    for (uint8_t i = 0; i < 127; i++) {
      uint8_t y1 = waveH - 1 - (((uint16_t)(buffer.raw[i] - wMin) * (waveH - 1)) / wRange);
      uint8_t y2 = waveH - 1 - (((uint16_t)(buffer.raw[i + 1] - wMin) * (waveH - 1)) / wRange);
      display.drawLine(i, waveY + y1, i + 1, waveY + y2, WHITE);
    }
  }
  
  display.setTextSize(1);
  if (showParams) drawParameterBar(true);
}

// ---------------- Parameter bar ----------------
void drawParameterBar(bool showParams) {
  if (!showParams) return;
  display.setTextSize(1);

  // Wave mode has 4 menu items, others have 3
  bool isWaveMode = (mode == MODE_WAVE);

  // Mode name - inverted if selected, underline if cursor here
  display.setTextColor((param_select == 1) ? BLACK : WHITE, (param_select == 1) ? WHITE : BLACK);
  display.setCursor(0, 0);
  switch (mode) {
    case MODE_LFO:      display.print(F("LFO")); break;
    case MODE_WAVE:     display.print(F("WAVE")); break;
    case MODE_SPECTRUM: display.print(F("SPEC")); break;
    case MODE_TUNER:    display.print(F("TUNE")); break;
  }
  if (param == 1) display.drawFastHLine(0, 8, 24, WHITE);

  // Param1 (Time/AUTO/High) - inverted if selected, underline if cursor here
  display.setTextColor((param_select == 2) ? BLACK : WHITE, (param_select == 2) ? WHITE : BLACK);
  if (isWaveMode) {
    display.setCursor(32, 0);
    if (param1 == 9) {
      display.print(F("AUTO"));
    } else {
      display.print(F("T:"));
      display.print(param1);
    }
    if (param == 2) display.drawFastHLine(32, 8, 24, WHITE);
  } else {
    display.setCursor(42, 0);
    switch (mode) {
      case MODE_LFO:       display.print(F("Time:")); break;
      case MODE_SPECTRUM:  display.print(F("High:")); break;
      case MODE_TUNER:     display.print(F("ZC")); break;  // ZC-only mode
    }
    if (mode != MODE_TUNER) {
      display.setCursor(72, 0); display.print(param1);
    }
    if (param == 2) display.drawFastHLine(42, 8, 36, WHITE);
  }

  // Param2 / Param3 area
  if (isWaveMode) {
    // Wave mode: Offs (param2) then Rfrs (param3)
    display.setTextColor((param_select == 3) ? BLACK : WHITE, (param_select == 3) ? WHITE : BLACK);
    display.setCursor(60, 0);
    display.print(F("O:"));
    display.print(param2);
    if (param == 3) display.drawFastHLine(60, 8, 24, WHITE);
    
    display.setTextColor((param_select == 4) ? BLACK : WHITE, (param_select == 4) ? WHITE : BLACK);
    display.setCursor(90, 0);
    display.print(F("R:"));
    display.print(param3);
    if (param == 4) display.drawFastHLine(90, 8, 24, WHITE);
  } else if (mode != MODE_TUNER) {
    // LFO: Offs, Spectrum: Filt
    display.setTextColor((param_select == 3) ? BLACK : WHITE, (param_select == 3) ? WHITE : BLACK);
    display.setCursor(84, 0);
    switch (mode) {
      case MODE_LFO:      display.print(F("Offs:")); break;
      case MODE_SPECTRUM:display.print(F("Filt:")); break;
    }
    display.setCursor(114, 0); display.print(param2);
    if (param == 3) display.drawFastHLine(84, 8, 36, WHITE);
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
  
  if (old_SW == 0 && SW == 1) configMenuOption = (configMenuOption % 4) + 1;

  static unsigned long holdStart = 0;
  if (SW && !old_SW) holdStart = millis();
  
  if (SW && holdStart > 0 && (millis() - holdStart >= 2000)) {
    EEPROM.put(ENCODER_DIR_ADDR, encoderDirection);
    EEPROM.write(OLED_ROT_ADDR, oledRotation);
    EEPROM.put(MENUTIMER_DIR_ADDR, (uint8_t)menuTimer);
    EEPROM.write(BOOTLOGO_ADDR, bootLogoEnabled);
    
    saveCurrentModeToRAM(mode);
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
