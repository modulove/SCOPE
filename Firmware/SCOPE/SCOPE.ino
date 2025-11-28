/**
 * @file SCOPE.ino
 * @author Modulove & friends (https://github.com/modulove/)
 * @brief Eurorack scope / OLED Display (new config menu (≥3s))
 * @version 1.1
 * @date 2025-10-15
 */

#include <EEPROM.h>
#include <avr/io.h>
#include "fix_fft.h"
#include <Encoder.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ---------------- Display configuration ----------------
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

// ---------------- EEPROM map ----------------
// add OLED rotation.
#define ENCODER_DIR_ADDR      0   // int8_t  (1 or -1)
#define OLED_ROT_ADDR         1   // uint8_t (0 or 2)
#define MENUTIMER_DIR_ADDR    2   // uint8_t (1..60)
const int EEPROM_MODE_ADDR = 4;   // uint8_t
const int EEPROM_PARAM_SELECT_ADDR = 5; // per-mode bank starts here (5..16 used for 4 modes)

// ---------------- Modes ----------------
#define MODE_LFO      1
#define MODE_WAVE     2
#define MODE_SHOT     3
#define MODE_SPECTRUM 4

// ---------------- Boot logo  ----------------
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

// ---------------- Globals ----------------
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

// Secret menu
bool encoderPressed = false;
bool secretMenuActive = false;
unsigned long enterPressStartTime = 0;
unsigned long exitPressStartTime  = 0;
byte secretMenuOption = 1; // 1=encoder dir, 2=menu timer, 3=OLED rotation
unsigned int menuTimer = 5; // seconds
int encoderDirection = 1;   // 1 or -1
uint8_t oledRotation = 0;   // 0 or 2 for SSD1306

// Data buffer union
union {
  struct { int8_t real[128]; int8_t imag[128]; } spectrum;
  uint8_t waveform[128];
} buffer;

// ------------- Optional: WebScope flag -------------
volatile bool webscopeEnabled = false; // set by serial command later

// ---------------- Prototypes ----------------
void drawBootAnimation();
void setupMode(uint8_t mode);
void runLFOMode(bool showParams);
void runWaveMode(bool showParams);
void runShotMode(bool showParams);
void runSpectrumMode(bool showParams);
void drawParameterBar(bool showParams);
void secretMenu();
void saveSettings();
void loadSettings();

// ---------------- Setup ----------------
void setup() {
  // Load encoder direction
  EEPROM.get(ENCODER_DIR_ADDR, encoderDirection);
  if (encoderDirection != 1 && encoderDirection != -1) encoderDirection = 1;

  // Load OLED rotation
  oledRotation = EEPROM.read(OLED_ROT_ADDR);
  if (!(oledRotation == 0 || oledRotation == 2)) oledRotation = 0;

  // Load menu timer
  EEPROM.get(MENUTIMER_DIR_ADDR, menuTimer);
  if (menuTimer < 1 || menuTimer > 60) menuTimer = 5;

  // Load last mode
  uint8_t lastMode = EEPROM.read(EEPROM_MODE_ADDR);
  if (lastMode >= MODE_LFO && lastMode <= MODE_SPECTRUM) mode = lastMode;

  // Display init
  display.begin(SSD1306_SWITCHCAPVCC);
  display.setRotation(oledRotation);   // runtime-applied
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  drawBootAnimation();
  delay(1000);

  // I/O
  pinMode(OFFSET_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FILTER_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);

  // Fast PWM on Timer2
  TCCR2B &= B11111000;
  TCCR2B |= B00000001;

  loadSettings();
  setupMode(mode);

  // (Optional) Serial for WebSerial features
  // Serial.begin(115200);
}

// ---------------- Loop ----------------
void loop() {
  old_SW = SW;
  old_mode = mode;
  SW = (digitalRead(BUTTON_PIN) == LOW);

  // Button timing
  static unsigned long buttonPressStart = 0;
  static bool isLongPressing = false;
  static bool hasSaved = false;

  if (SW && !old_SW) {
    buttonPressStart = millis();
    isLongPressing = true;
    hasSaved = false;
  }
  if (!SW && old_SW) isLongPressing = false;

  // Medium press: save (1–3s)
  if (isLongPressing && !hasSaved) {
    unsigned long d = millis() - buttonPressStart;
    if (d >= 1000 && d < 3000) {
      saveSettings();
      hasSaved = true;
    }
  }

  // Very long press: enter config menu (≥3s)
  if (SW && !secretMenuActive && (millis() - buttonPressStart >= 3000)) {
    secretMenuActive = true;
    encoderPressed = false;
    secretMenuOption = 1;
    oldPosition = newPosition = encoder.read();
  }

  if (secretMenuActive) {
    secretMenu();
    return;
  }

  // ---------- Normal operation ----------
  newPosition = encoderDirection * encoder.read();

  // Button: cycle through parameter “slots” like original
  if (old_SW == 0 && SW == 1 && param_select == param) { param_select = 0; hideTimer = millis(); }
  else if (old_SW == 0 && SW == 1 && (param >= 1 && param <= 3)) { param_select = param; hideTimer = millis(); }

  mode = constrain(mode, 1, 4);
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

  // Keep “mode selecting” sticky while switching modes
  byte currentParamSelect = param_select;
  if (old_mode != mode) {
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
  }

  display.display();

  // (Optional) web scope streaming could go here if enabled
  // if (webscopeEnabled) { /* stream a decimated frame over Serial */ }
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

// ---------------- Mode setup ----------------
void setupMode(uint8_t m) {
  int baseAddr = EEPROM_PARAM_SELECT_ADDR + ((m - 1) * 3);
  uint8_t saved_param_select = EEPROM.read(baseAddr);
  uint8_t saved_param1 = EEPROM.read(baseAddr + 1);
  uint8_t saved_param2 = EEPROM.read(baseAddr + 2);
  if (saved_param_select <= 3) param_select = saved_param_select;

  switch (m) {
    case MODE_LFO:
      param1 = (saved_param1 >= 1 && saved_param1 <= 8) ? saved_param1 : 4; // Time
      param2 = (saved_param2 >= 1 && saved_param2 <= 8) ? saved_param2 : 1; // Offset
      pinMode(FILTER_PIN, INPUT);
      analogWrite(OFFSET_PIN, 0);
      ADCSRA = (ADCSRA & 0xF8) | 0x04; // prescaler /16 (fast)
      break;

    case MODE_WAVE:
      param1 = (saved_param1 >= 1 && saved_param1 <= 8) ? saved_param1 : 8; // Time
      param2 = (saved_param2 >= 1 && saved_param2 <= 6) ? saved_param2 : 1; // Refresh
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      break;

    case MODE_SHOT:
      param1 = (saved_param1 >= 1 && saved_param1 <= 4) ? saved_param1 : 2; // Time
      param2 = 1;
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      break;

    case MODE_SPECTRUM:
      param1 = (saved_param1 >= 1 && saved_param1 <= 4) ? saved_param1 : 1; // HF amp
      param2 = (saved_param2 >= 1 && saved_param2 <= 8) ? saved_param2 : 8; // Noise filt
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, OUTPUT);
      digitalWrite(FILTER_PIN, LOW);
      ADCSRA = (ADCSRA & 0xF8) | 0x07; // prescaler /128 (accuracy)
      break;
  }

  memset(&buffer, 0, sizeof(buffer));
  rfrs = 0;
}

// ---------------- LFO Mode ----------------
void runLFOMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);  // Time scale
  param2 = constrain(param2, 1, 8);  // Vertical offset

  uint8_t currentSample = analogRead(ANALOG_INPUT_PIN) >> 4; // 0..63

  // Scroll waveform (faster than for-loop)
  memmove(&buffer.waveform[1], &buffer.waveform[0], 127);
  buffer.waveform[0] = currentSample;

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 50) {
    lastDraw = millis();
    display.clearDisplay();

    int step = (9 - param1);          // 8..1
    int voff = (param2 - 1) * 4;      // 0..28
    int segments = 126 / step;

    for (int i = 0; i < segments; i++) {
      int x1 = 127 - (i * step);
      int y1 = 63 - buffer.waveform[i] - voff;
      int x2 = 127 - ((i + 1) * step);
      int y2 = 63 - buffer.waveform[i + 1] - voff;
      if (y1 < 0) y1 = 0; if (y1 > 63) y1 = 63;
      if (y2 < 0) y2 = 0; if (y2 > 63) y2 = 63;
      display.drawLine(x1, y1, x2, y2, WHITE);
    }

    if (showParams) drawParameterBar(true);
  }
}

// ---------------- Wave Mode ----------------
void runWaveMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8); // Time scale
  param2 = constrain(param2, 1, 6); // Refresh

  static unsigned long lastUpdate = 0;
  unsigned long interval = 50UL + (param2 - 1) * 100UL;
  if (millis() - lastUpdate < interval) return;
  lastUpdate = millis();

  display.clearDisplay();

  if (param1 > 5) {
    // Mid frequency: slower sampling with small delay
    uint8_t d = (param1 - 5) * 20; // ~20..60 us
    for (int i = 127; i >= 0; i--) {
      buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) >> 4;
      if (d) delayMicroseconds(d);
    }
    for (int i = 127; i >= 1; i--) {
      display.drawLine(127 - i, 63 - buffer.waveform[i - 1],
                       127 - (i + 1), 63 - buffer.waveform[i], WHITE);
    }
  } else {
    // High frequency: fewer points
    int stride = (6 - param1); // 5..1
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

// ---------------- Shot (triggered) Mode ----------------
void runShotMode(bool showParams) {
  param  = constrain(param, 1, 2);
  param1 = constrain(param1, 1, 4); // Time scale

  old_trig = trig;
  trig = digitalRead(TRIGGER_PIN);

  static bool redrawNeeded = true;
  static unsigned long lastUpdate = 0;

  if (!old_trig && trig) {
    // Capture after trigger
    // Scale capture speed by param1 (lower param1 = faster)
    // ~25..200 us per sample gives nice time windows
    uint16_t us = (uint16_t)(25UL * (1UL << (param1 - 1))); // 25,50,100,200
    for (int i = 10; i <= 127; i++) {
      buffer.waveform[i] = analogRead(ANALOG_INPUT_PIN) >> 4;
      if (us) delayMicroseconds(us);
    }
    // trigger marker
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

// ---------------- Spectrum Mode ----------------
void runSpectrumMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 4); // HF “tilt”
  param2 = constrain(param2, 1, 8); // Noise threshold

  static unsigned long lastUpdate = 0;
  const unsigned long interval = 100;
  if (millis() - lastUpdate < interval) return;
  lastUpdate = millis();

  for (uint8_t i = 0; i < 128; i++) {
    int spec = analogRead(ANALOG_INPUT_PIN);
    buffer.spectrum.real[i] = (spec >> 2) - 128; // center
    buffer.spectrum.imag[i] = 0;
  }

  fix_fft(buffer.spectrum.real, buffer.spectrum.imag, 7, 0);

  display.clearDisplay();

  // Fast magnitude approximation: |re| + |im| (no sqrt)
  for (uint8_t i = 0; i < 64; i++) {
    int re = buffer.spectrum.real[i];
    int im = buffer.spectrum.imag[i];
    int level = abs(re) + abs(im); // 0..~512

    if (level >= (param2 * 2)) {
      // high-frequency “tilt”
      int boosted = level + ((int)i * (param1 - 1));
      if (boosted > 63) boosted = 63;
      if (boosted < 0)  boosted = 0;
      display.fillRect(i * 2, 63 - boosted, 2, boosted, WHITE);
    }
  }

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
  }
  display.setCursor(72, 0); display.print(param1);

  if (mode == MODE_SHOT) {
    display.setTextColor(trig ? BLACK : WHITE, trig ? WHITE : BLACK);
    display.setCursor(84, 0); display.print(F("TRIG"));
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

// ---------------- config menu (incl OLED rotation) ----------------
void secretMenu() {
  int newDirection = encoderDirection;
  newPosition = encoder.read();

  if (newPosition > (oldPosition + 3)) {
    oldPosition = newPosition;
    if (secretMenuOption == 1) {
      newDirection = 1;
    } else if (secretMenuOption == 2) {
      if (menuTimer > 1) menuTimer--;
    } else if (secretMenuOption == 3) {
      oledRotation = 0;                      // 0°
      display.setRotation(oledRotation);     // Apply immediately
      display.clearDisplay();
    }
  } else if (newPosition < (oldPosition - 3)) {
    oldPosition = newPosition;
    if (secretMenuOption == 1) {
      newDirection = -1;
    } else if (secretMenuOption == 2) {
      if (menuTimer < 60) menuTimer++;
    } else if (secretMenuOption == 3) {
      oledRotation = 2;                      // 180°
      display.setRotation(oledRotation);
      display.clearDisplay();
    }
  }

  if (newDirection != encoderDirection) encoderDirection = newDirection;

  // Cycle menu item on short press
  if (old_SW == 0 && SW == 1) {
    secretMenuOption = (secretMenuOption % 3) + 1; // 1→2→3→1
  }

  // Long hold to save/exit
  if (SW && !encoderPressed) { encoderPressed = true; exitPressStartTime = millis(); }
  else if (!SW && encoderPressed) { encoderPressed = false; exitPressStartTime = 0; }

  int exitProgress = 0;
  if (encoderPressed && exitPressStartTime > 0) {
    unsigned long d = millis() - exitPressStartTime;
    exitProgress = (int)constrain((long)(d * 100 / 3000), 0L, 100L);
  }

  if (exitProgress >= 100) {
    EEPROM.put(ENCODER_DIR_ADDR, encoderDirection);
    delay(5);
    EEPROM.write(OLED_ROT_ADDR, oledRotation);
    delay(5);
    EEPROM.put(MENUTIMER_DIR_ADDR, (uint8_t)menuTimer);
    delay(5);

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(16, 25);
    display.print(F("SETTINGS SAVED"));
    display.display();
    delay(700);

    secretMenuActive = false;
    encoderPressed = false;
    exitPressStartTime = 0;
    oldPosition = newPosition = encoder.read() * encoderDirection;
    display.clearDisplay();
    display.display();
    return;
  }

  // Draw menu
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.println(F("GLOBAL SETTINGS"));

  // 1) Encoder direction
  display.setTextColor(secretMenuOption == 1 ? BLACK : WHITE, secretMenuOption == 1 ? WHITE : BLACK);
  display.println(F("Encoder Direction:"));
  display.setTextColor(WHITE);
  display.println(encoderDirection == 1 ? F("Normal") : F("Reversed"));
  display.println();

  // 2) Menu timer
  display.setTextColor(secretMenuOption == 2 ? BLACK : WHITE, secretMenuOption == 2 ? WHITE : BLACK);
  display.println(F("Menu Timer:"));
  display.setTextColor(WHITE);
  display.print(menuTimer); display.println(F(" s"));
  display.println();

  // 3) OLED Rotation
  display.setTextColor(secretMenuOption == 3 ? BLACK : WHITE, secretMenuOption == 3 ? WHITE : BLACK);
  display.println(F("OLED Rotation:"));
  display.setTextColor(WHITE);
  display.println(oledRotation == 0 ? F("0 deg") : F("180 deg"));

  // Exit bar
  if (exitProgress > 0) {
    display.drawRect(0, 56, 128, 8, WHITE);
    display.fillRect(2, 58, (exitProgress * 124) / 100, 4, WHITE);
    display.setCursor(0, 48); display.print(F("Hold to save & exit"));
  } else {
    display.setCursor(0, 56); display.print(F("Hold button to exit"));
  }
  display.display();
}

// ---------------- Save/Load mode params ----------------
void saveSettings() {
  EEPROM.write(EEPROM_MODE_ADDR, mode);
  int baseAddr = EEPROM_PARAM_SELECT_ADDR + ((mode - 1) * 3);
  EEPROM.write(baseAddr,     param_select); delay(3);
  EEPROM.write(baseAddr + 1, param1);       delay(3);
  EEPROM.write(baseAddr + 2, param2);       delay(3);

  lastSaveTime = millis();
  display.fillRect(0, 54, 128, 10, WHITE);
  display.setTextColor(BLACK);
  display.setCursor(8, 55);
  display.print(F("MODE SETTINGS SAVED"));
  display.display();
  delay(600);
}

void loadSettings() {
  int baseAddr = EEPROM_PARAM_SELECT_ADDR + ((mode - 1) * 3);
  param_select = EEPROM.read(baseAddr);
  param1 = EEPROM.read(baseAddr + 1);
  param2 = EEPROM.read(baseAddr + 2);

  if (param_select > 3) param_select = 0;

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
      param2 = 1;
      break;
    case MODE_SPECTRUM:
      param1 = constrain(param1, 1, 4);
      param2 = constrain(param2, 1, 8);
      break;
  }
}
