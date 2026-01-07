/**
 * @file SCOPE_v1_backported.ino
 * @author Modulove
 * @brief Eurorack scope + OLED flip (v1 hardware - unipolar)
 * @version 1.2
 * @date 2026-01-07
 * 
 * BACKPORTED IMPROVEMENTS FROM v2:
 * - Use fastAnalogRead() for speed
 * - Enhanced tuner mode with waveform visualization
 * - Improved frequency detection 
 * - Better settings management
 * - Config menu implementation
  */

#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "fix_fft.h"
#include <Encoder.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

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
  uint8_t waveform[256];  // Larger buffer for tuner mode
} buffer;

// ---------------- Mode Settings Storage ----------------
struct ModeSettings {
  uint8_t param_select;
  uint8_t param1;
  int8_t param2;
};
ModeSettings modeSettings[NUM_MODES];

// ---------------- Globals ----------------
uint8_t mode = MODE_LFO;
uint8_t old_mode = MODE_LFO;
uint8_t param_select = 0;
uint8_t param = 1;
uint8_t param1 = 2;
int8_t param2 = 1;

bool trig = 0, old_trig = 0;
bool SW = 0, old_SW = 0;

unsigned long hideTimer = 0;
unsigned long lastSaveTime = 0;
bool hide = 0;
int rfrs = 0;

int16_t oldPosition = -999;
int16_t newPosition = -999;

// config menu
bool configMenuActive = false;
byte configMenuOption = 1;
unsigned int menuTimer = 5;
int encoderDirection = 1;
uint8_t oledRotation = 0;
uint8_t bootLogoEnabled = 1;

// Tuner state (backported from v2)
uint32_t smoothedFreqX100 = 0;  // Fixed-point frequency * 100
uint32_t lastValidFreqX100 = 0;

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
void saveCurrentModeToRAM(uint8_t m);
void saveAllSettings();
void loadAllSettings();
uint8_t fastAnalogRead();
uint32_t detectFrequencyZC(uint16_t sampleRateX10);
void frequencyToNote(uint32_t freqX100, char* note, int8_t* octave, int8_t* cents);

// ============ BACKPORTED: Fast analog read ============
// Fast single ADC read (8-bit, ~15µs vs ~100µs for analogRead)
uint8_t fastAnalogRead() {
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (ANALOG_INPUT_PIN & 0x07);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (0x06);  // Prescaler 64
  while (ADCSRA & (1 << ADSC));
  return ADCH;
}

// ============ BACKPORTED: Buffer-based frequency detection ============
// Returns frequency * 100 (fixed-point), sampleRateX10 = sample rate / 10
uint32_t detectFrequencyZC(uint16_t sampleRateX10) {
  uint8_t minVal = 255, maxVal = 0;
  for (uint16_t i = 0; i < 256; i++) {
    if (buffer.waveform[i] < minVal) minVal = buffer.waveform[i];
    if (buffer.waveform[i] > maxVal) maxVal = buffer.waveform[i];
  }
  
  uint8_t range = maxVal - minVal;
  if (range < 30) return 0;  // Signal too weak
  
  uint8_t center = (minVal + maxVal) / 2;
  uint8_t hysteresis = range / 6;
  if (hysteresis < 5) hysteresis = 5;
  
  uint8_t threshHigh = center + hysteresis;
  uint8_t threshLow = center - hysteresis;
  
  uint8_t crossings = 0;
  uint16_t firstCrossing = 0;
  uint16_t lastCrossing = 0;
  bool aboveCenter = buffer.waveform[0] > center;
  
  for (uint16_t i = 1; i < 256; i++) {
    if (!aboveCenter && buffer.waveform[i] > threshHigh) {
      aboveCenter = true;
      crossings++;
      
      if (firstCrossing == 0) {
        firstCrossing = i;
      }
      lastCrossing = i;
    }
    else if (aboveCenter && buffer.waveform[i] < threshLow) {
      aboveCenter = false;
    }
  }
  
  if (crossings < 2) return 0;
  
  uint16_t totalSamples = lastCrossing - firstCrossing;
  
  // freq * 100 = (sampleRate * 100) / avgPeriod
  uint32_t freqX100 = ((uint32_t)sampleRateX10 * 1000UL * (crossings - 1)) / totalSamples;
  
  return freqX100;
}

// ============ BACKPORTED: Note frequency table ============
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

// ============ BACKPORTED: Improved note conversion ============
void frequencyToNote(uint32_t freqX100, char* note, int8_t* octave, int8_t* cents) {
  static const char noteNames[] PROGMEM = "C C#D D#E F F#G G#A A#B ";
  
  if (freqX100 < 1635) {  // Below C0 (~16.35 Hz)
    note[0] = '-'; note[1] = '\0';
    *octave = 0; *cents = 0;
    return;
  }
  
  // Normalize to octave 4
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
  
  // Find closest note
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
  
  // Get note name from PROGMEM
  uint8_t idx = closestNote * 2;
  note[0] = pgm_read_byte(&noteNames[idx]);
  note[1] = pgm_read_byte(&noteNames[idx + 1]);
  note[2] = '\0';
  if (note[1] == ' ') note[1] = '\0';
  
  // Calculate cents deviation
  uint16_t targetFreq = pgm_read_word(&noteFreqX100[closestNote]);
  int32_t diff = (int32_t)normFreq - (int32_t)targetFreq;
  *cents = (int8_t)((diff * 173L) / (int32_t)targetFreq);
  *cents = constrain(*cents, -50, 50);
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

  // Medium press: save (≥1s)
  if (isLongPressing && !hasSaved) {
    unsigned long d = millis() - buttonPressStart;
    if (d >= 1000) {
      saveCurrentModeToRAM(mode);
      saveAllSettings();
      hasSaved = true;
    }
  }

  // Very long press: enter config menu (≥3s)
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

  // ---------- Normal operation ----------
  newPosition = encoderDirection * encoder.read();

  // Button: cycle through parameter "slots"
  if (old_SW == 0 && SW == 1 && param_select == param) { param_select = 0; hideTimer = millis(); }
  else if (old_SW == 0 && SW == 1 && (param >= 1 && param <= 3)) { param_select = param; hideTimer = millis(); }

  mode = constrain(mode, 1, NUM_MODES);
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

  // Keep "mode selecting" sticky while switching modes
  byte currentParamSelect = param_select;
  if (old_mode != mode) {
    saveCurrentModeToRAM(old_mode);  // Save OLD mode before switching
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

// ============ BACKPORTED: Better settings management ============
void saveCurrentModeToRAM(uint8_t m) {
  uint8_t idx = m - 1;
  if (idx < NUM_MODES) {
    modeSettings[idx].param_select = param_select;
    modeSettings[idx].param1 = param1;
    modeSettings[idx].param2 = param2;
  }
}

void loadAllSettings() {
  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int baseAddr = EEPROM_PARAM_SELECT_ADDR + (m * 3);
    modeSettings[m].param_select = EEPROM.read(baseAddr);
    modeSettings[m].param1 = EEPROM.read(baseAddr + 1);
    modeSettings[m].param2 = (int8_t)EEPROM.read(baseAddr + 2);
    
    if (modeSettings[m].param_select > 3) modeSettings[m].param_select = 0;
    
    switch (m + 1) {
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
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 8);
        modeSettings[m].param2 = 1;
        if (modeSettings[m].param1 == 0) modeSettings[m].param1 = 4;
        break;
    }
  }
}

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
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      break;

    case MODE_WAVE:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      break;

    case MODE_SHOT:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      break;

    case MODE_SPECTRUM:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, OUTPUT);
      digitalWrite(FILTER_PIN, LOW);
      ADCSRA = (ADCSRA & 0xF8) | 0x07;
      break;

    case MODE_TUNER:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      smoothedFreqX100 = 0;
      lastValidFreqX100 = 0;
      break;
  }

  memset(&buffer, 0, sizeof(buffer));
  rfrs = 0;
}

// ============ BACKPORTED: LFO with fastAnalogRead ============
void runLFOMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);
  param2 = constrain(param2, -6, 10);

  uint8_t currentSample = fastAnalogRead() >> 2;  // 8-bit to 6-bit (0-63)

  memmove(&buffer.waveform[1], &buffer.waveform[0], 127);
  buffer.waveform[0] = currentSample;

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 30) {  // Slightly faster refresh
    lastDraw = millis();
    display.clearDisplay();

    int step = (9 - param1);
    int voff = (param2 - 1) * 6;
    int segments = 126 / step;

    for (int i = 0; i < segments; i++) {
      int x1 = 127 - (i * step);
      int y1 = buffer.waveform[i] + voff;
      int x2 = 127 - ((i + 1) * step);
      int y2 = buffer.waveform[i + 1] + voff;
      y1 = constrain(y1, 0, 63);
      y2 = constrain(y2, 0, 63);
      display.drawLine(x1, y1, x2, y2, WHITE);
    }

    if (showParams) drawParameterBar(true);
  }
}

// ---------------- Wave Mode (keep original) ----------------
void runWaveMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);
  param2 = constrain(param2, 1, 6);

  static unsigned long lastUpdate = 0;
  unsigned long interval = 50UL + (param2 - 1) * 100UL;
  if (millis() - lastUpdate < interval) return;
  lastUpdate = millis();

  display.clearDisplay();

  if (param1 > 5) {
    uint8_t d = (param1 - 5) * 20;
    for (int i = 127; i >= 0; i--) {
      buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) >> 4;
      if (d) delayMicroseconds(d);
    }
    for (int i = 127; i >= 1; i--) {
      display.drawLine(127 - i, 63 - buffer.waveform[i - 1],
                       127 - (i + 1), 63 - buffer.waveform[i], WHITE);
    }
  } else {
    int stride = (6 - param1);
    int count = 127 / stride;
    for (int i = count; i >= 0; i--) {
      buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) >> 4;
    }
    for (int i = count; i >= 1; i--) {
      display.drawLine(127 - i * stride,     63 - buffer.waveform[i - 1],
                       127 - (i + 1) * stride, 63 - buffer.waveform[i], WHITE);
    }
  }

  if (showParams) drawParameterBar(true);
}

// ---------------- Shot Mode (keep original) ----------------
void runShotMode(bool showParams) {
  param  = constrain(param, 1, 2);
  param1 = constrain(param1, 1, 4);

  old_trig = trig;
  trig = digitalRead(TRIGGER_PIN);

  static bool redrawNeeded = true;
  static unsigned long lastUpdate = 0;

  if (!old_trig && trig) {
    uint16_t us = (uint16_t)(25UL * (1UL << (param1 - 1)));
    for (int i = 10; i <= 127; i++) {
      buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) >> 4;
      if (us) delayMicroseconds(us);
    }
    for (int i = 0; i < 10; i++) buffer.waveform[i] = 32;
    redrawNeeded = true;
  }

  if (redrawNeeded || millis() - lastUpdate >= 200) {
    lastUpdate = millis();
    redrawNeeded = false;

    display.clearDisplay();
    for (int i = 1; i < 127; i++) {
      display.drawLine(i, 63 - buffer.waveform[i], i + 1, 63 - buffer.waveform[i + 1], WHITE);
    }
    if (showParams) drawParameterBar(true);
  }
}

// ---------------- Spectrum Mode (keep original - DO NOT TOUCH) ----------------
void runSpectrumMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 4);
  param2 = constrain(param2, 1, 8);

  static unsigned long lastUpdate = 0;
  const unsigned long interval = 100;
  if (millis() - lastUpdate < interval) return;
  lastUpdate = millis();

  for (uint8_t i = 0; i < 128; i++) {
    int spec = analogRead(ANALOG_INPUT_PIN);
    buffer.spectrum.real[i] = (spec >> 2) - 128;
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
      if (boosted > 63) boosted = 63;
      if (boosted < 0)  boosted = 0;
      display.fillRect(i * 2, 63 - boosted, 2, boosted, WHITE);
    }
  }

  if (showParams) drawParameterBar(true);
}

// ============ BACKPORTED: Enhanced Tuner Mode ============
void runTunerMode(bool showParams) {
  param  = constrain(param, 1, 2);
  param1 = constrain(param1, 1, 8);  // Averaging level

  static unsigned long lastMeasurement = 0;
  static unsigned long lastDisplay = 0;
  
  // Collect samples for frequency detection
  if (millis() - lastMeasurement >= 50) {
    lastMeasurement = millis();
    
    // Disable interrupts during sampling for accuracy
    noInterrupts();
    for (uint16_t i = 0; i < 256; i++) {
      buffer.waveform[i] = fastAnalogRead();
      delayMicroseconds(20);  // ~50kHz sample rate
    }
    interrupts();
    
    // Detect frequency (sample rate = ~5000 Hz, so sampleRateX10 = 500)
    uint32_t rawFreqX100 = detectFrequencyZC(500);
    
    if (rawFreqX100 > 1500 && rawFreqX100 < 500000) {  // 15-5000 Hz
      lastValidFreqX100 = rawFreqX100;
      
      // Smoothing based on param1
      if (smoothedFreqX100 < 1000) {
        smoothedFreqX100 = rawFreqX100;
      } else {
        float alpha = 1.0 / param1;
        smoothedFreqX100 = (uint32_t)(alpha * rawFreqX100 + (1.0 - alpha) * smoothedFreqX100);
      }
    } else {
      // Decay smoothed value when no signal
      smoothedFreqX100 = (smoothedFreqX100 * 90) / 100;
      if (smoothedFreqX100 < 1500) smoothedFreqX100 = 0;
    }
  }

  // Update display
  if (millis() - lastDisplay < 100) return;
  lastDisplay = millis();
  
  display.clearDisplay();
  
  int yOff = showParams ? 10 : 0;
  
  if (smoothedFreqX100 > 1500) {
    char note[3];
    int8_t octave, cents;
    frequencyToNote(smoothedFreqX100, note, &octave, &cents);
    
    // Display note name + octave (large text)
    uint8_t noteLen = strlen(note);
    int noteWidth = (noteLen * 12) + 12;
    int noteX = (128 - noteWidth) / 2;
    
    display.setTextSize(2);
    display.setCursor(noteX, yOff + 4);
    display.print(note);
    display.print(octave);
    
    // Display cents deviation
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
    
    // Tuning bar
    int barY = showParams ? 44 : 34;
    display.drawRect(14, barY, 100, 6, WHITE);
    display.drawFastVLine(64, barY - 2, 10, WHITE);
    
    // Draw tuning indicator
    int indicator = 64 + constrain(cents, -50, 50);
    display.fillRect(indicator - 2, barY + 1, 5, 4, WHITE);
    
    // ===== BACKPORTED: Waveform visualization below tuning bar =====
    int waveY = barY + 8;
    uint8_t waveH = showParams ? 10 : 14;
    
    // Find min/max for auto-scaling
    uint8_t wMin = 255, wMax = 0;
    for (uint8_t i = 0; i < 128; i++) {
      if (buffer.waveform[i] < wMin) wMin = buffer.waveform[i];
      if (buffer.waveform[i] > wMax) wMax = buffer.waveform[i];
    }
    uint8_t wRange = wMax - wMin;
    if (wRange < 10) wRange = 10;
    
    // Draw waveform (128 pixels, 1:1 mapping)
    for (uint8_t i = 0; i < 127; i++) {
      uint8_t y1 = waveH - 1 - (((uint16_t)(buffer.waveform[i] - wMin) * (waveH - 1)) / wRange);
      uint8_t y2 = waveH - 1 - (((uint16_t)(buffer.waveform[i + 1] - wMin) * (waveH - 1)) / wRange);
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
    
    // Waveform (shows noise/input activity)
    int waveY = barY + 8;
    uint8_t waveH = showParams ? 10 : 14;
    
    uint8_t wMin = 255, wMax = 0;
    for (uint8_t i = 0; i < 128; i++) {
      if (buffer.waveform[i] < wMin) wMin = buffer.waveform[i];
      if (buffer.waveform[i] > wMax) wMax = buffer.waveform[i];
    }
    uint8_t wRange = wMax - wMin;
    if (wRange < 10) wRange = 10;
    
    for (uint8_t i = 0; i < 127; i++) {
      uint8_t y1 = waveH - 1 - (((uint16_t)(buffer.waveform[i] - wMin) * (waveH - 1)) / wRange);
      uint8_t y2 = waveH - 1 - (((uint16_t)(buffer.waveform[i + 1] - wMin) * (waveH - 1)) / wRange);
      display.drawLine(i, waveY + y1, i + 1, waveY + y2, WHITE);
    }
  }
  
  display.setTextSize(1);
  if (showParams) drawParameterBar(true);
}

// ---------------- Parameter bar ----------------
void drawParameterBar(bool showParams) {
  if (!showParams) return;

  display.drawLine((param - 1) * 42, 8, (param - 1) * 42 + 36, 8, WHITE);

  display.setTextColor((param_select == 1) ? BLACK : WHITE, (param_select == 1) ? WHITE : BLACK);
  display.setCursor(0, 0); display.print(F("Mode:"));
  display.setCursor(30, 0); display.print(mode);

  display.setTextColor((param_select == 2) ? BLACK : WHITE, (param_select == 2) ? WHITE : BLACK);
  display.setCursor(42, 0);
  switch (mode) {
    case MODE_LFO:
    case MODE_WAVE:
    case MODE_SHOT:    display.print(F("Time:")); break;
    case MODE_SPECTRUM:display.print(F("High:")); break;
    case MODE_TUNER:   display.print(F("Avg: ")); break;
  }
  display.setCursor(72, 0); display.print(param1);

  if (mode == MODE_SHOT) {
    display.setTextColor(trig ? BLACK : WHITE, trig ? WHITE : BLACK);
    display.setCursor(84, 0); display.print(F("TRIG"));
  } else if (mode == MODE_TUNER) {
    // Tuner mode has no param2
  } else {
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

// ============ BACKPORTED: Cleaner config menu ============
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
  
  // Cycle menu item on short press
  if (old_SW == 0 && SW == 1) configMenuOption = (configMenuOption % 4) + 1;

  // Hold 2s to save and exit
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

  // Draw menu
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
