/**
 * @file SCOPEv2.ino
 * @author Modulove
 * @brief Eurorack scope + Tuner + Function Generator
 * @version 3.7
 * @date 2025-02-15
 *
 * Modes: LFO / WAVE / TUNER / GEN (4 modes)
 *
 * v3.5: parameter bar underlines the slot under the cursor while browsing;
 *       the slot being edited stays inverted.
 * v3.6: WAVE mode gets a real trigger engine (external + software + auto),
 *       monotonic time base T:1 (slowest) .. T:8 (fastest), left-to-right sweep.
 * v3.7: zero-dead-time capture: the ISR re-arms itself after every sweep
 *       (ping-pong halves of buffer[]), so no trigger edge is ever missed;
 *       an external edge always re-locks the sweep; R: is display rate only.
 *
 * MODE_LFO: Continuous scrolling waveform. Trigger pin freezes a single capture.
 * MODE_WAVE: Triggered sweep, ISR-sampled. Cable in TRIGGER -> external edge
 *   trigger (last sweep held until the next edge). Otherwise software trigger
 *   on the signal's mid-level; free-runs after 100 ms without a crossing (auto).
 * MODE_TUNER: Frequency detection (ZC) with auto sample rate + waveform preview.
 * MODE_GEN: Function generator (v2.5 only).
 *   - LGT8F: Native DAC on D4 (50kHz sample rate)
 *   - ATmega328P: MCP4725 I2C DAC (~10kHz sample rate)
 *   - Waveforms: Sine, Triangle, Saw, Square, DC offset
 *   - Freq 0.1 Hz – 2 kHz, DC 0.0 – 5.0 V in 0.1 V steps (actual max = VCC)
 *   - Calibration offset + gain in EEPROM (use SCOPE_CAL.ino to calibrate)
 */

// ================== MCU Detection ==================
#if defined(__LGT8F__) || defined(__LGT8FX8P__) || defined(LARDUINO_HSP)
  #define IS_LGT8F 1
#else
  #define IS_LGT8F 0
#endif

#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>      // MCP4725 I2C DAC (ATmega328P v2.5)
#include <Encoder.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================== Display ==================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define V2_OLED_MOSI   9
#define V2_OLED_CLK   10
#define V2_OLED_DC    11
#define V2_OLED_CS    12
#define V2_OLED_RESET 13

#define V25_OLED_DC     9
#define V25_OLED_RESET  8
#define V25_OLED_MOSI  11
#define V25_OLED_CS    10
#define V25_OLED_CLK   13

Adafruit_SSD1306 *display = nullptr;
bool isHWv25 = false;

// ================== Pins ==================
#define V2_ENCODER_PIN_A   2
#define V2_ENCODER_PIN_B   4
#define V25_ENCODER_PIN_A  A2
#define V25_ENCODER_PIN_B  A3

#define IDENT_HW_PIN   A7
#define BUTTON_PIN      5
#define FILTER_PIN      6
#define TRIGGER_PIN     7
#define OFFSET_PIN      3
#define ANALOG_INPUT_PIN 0

Encoder *encoder = nullptr;

// ================== EEPROM ==================
#define EEPROM_MAGIC_ADDR         0
#define ENCODER_DIR_ADDR          1
#define OLED_ROT_ADDR             2
#define MENUTIMER_DIR_ADDR        3
#define EEPROM_MODE_ADDR          5
#define EEPROM_PARAM_SELECT_ADDR  6   // 4 modes x 3 bytes = 12 bytes (6..17)
#define EEPROM_CAL_OFFSET_ADDR   18  // int8_t
#define EEPROM_CAL_GAIN_ADDR     19  // uint8_t (128=1.00x)

#define EEPROM_MAGIC_VALUE  0xA9

// ================== Modes ==================
#define MODE_LFO    1   // Merged LFO + SHOT (trigger = single capture)
#define MODE_WAVE   2   // Fast ISR-sampled waveform
#define MODE_TUNER  3
#define MODE_GEN    4
#define NUM_MODES   4

// ================== ADC / Capture Engine ==================
#define ADC_BUFFER_SIZE 256
#define ADC_PS_WAVE     0x05   // prescaler 32: ~45 kSPS on LGT8F, ~38 kSPS on ATmega328P

// Capture states (advanced by the ADC ISR)
#define CAP_IDLE   0
#define CAP_ARMED  1   // waiting for trigger
#define CAP_RUN    2   // storing samples
#define CAP_DONE   3   // buffer complete

// Trigger sources
#define TRIG_NONE  0   // free-run (auto)
#define TRIG_SOFT  1   // level trigger on the signal itself
#define TRIG_EXT   2   // rising edge on TRIGGER_PIN

volatile uint8_t  capState = CAP_IDLE;
volatile uint8_t  capSrc = TRIG_NONE;
volatile uint16_t adcSampleIndex = 0;
volatile uint16_t adcTargetSamples = 128;
volatile uint8_t  adcDelayCounter = 0;
volatile uint8_t  adcDelayTarget = 0;
volatile uint8_t  trigLo = 0, trigHi = 0;   // software trigger window (ADC units)
volatile bool     trigArmed = false;        // signal seen on the far side of the window
volatile bool     extEdge = false;          // rising edge seen on TRIGGER_PIN
volatile bool     capReady = false;         // a sweep completed since the last copy
volatile bool     capRearm = false;         // ISR re-arms itself after each sweep (WAVE)
volatile uint8_t  capBase = 0, readyBase = 0;   // ping-pong halves of buffer[] (WAVE)
uint8_t           trigPinLast = 0;          // ISR-only
uint8_t           disp[128];                // main-loop copy of the last complete sweep

// WAVE time base: ADC decimation per step, T:1 slowest .. T:8 fastest
// sweep length on LGT8F ~ 364, 182, 91, 45, 23, 11, 5.7, 2.8 ms
const uint8_t waveSkip[8] PROGMEM = {127, 63, 31, 15, 7, 3, 1, 0};

// ================== Tuner state ==================
float   smoothedFrequency = 0;
uint8_t tunerSampleRate = 2;
float   sampleRateHz = 0;

// ================== ADC state machine ==================
uint8_t waveState = 0;   // reset in setupMode to prevent black screen
uint8_t tunerState = 0;

// ================== MCU ==================
bool isLGT8F = false;

// (Boot logo bitmap removed for flash savings — may add smaller one later)

// ================== Generator ==================
uint32_t genPhase = 0;
uint32_t genPhaseInc = 0;
bool     genAvailable = false;
bool     dacIsI2C = false;       // true = MCP4725, false = native LGT8F DAC
uint8_t  dacI2CAddr = 0;         // MCP4725 address (0 = not found)
int8_t   calOffset = 0;
uint8_t  calGain = 128;

const uint16_t genFreqTableX10[] PROGMEM = {
  1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000
};
#define GEN_NUM_FREQS 14

// Shared waveform names (used in generator mode + parameter bar)
const char genWaveNames[][4] PROGMEM = {"SIN","TRI","SAW","SQR","DC "};

// DC voltage range: 0–5.0V in 0.1V steps (50 steps)
// Both DAC types use VCC reference — actual max output = VCC
// (USB power: ~4.1V, Eurorack 5V rail: ~4.9V). Calibration compensates.
#define GEN_DC_MAX 50

// Sample rates depend on DAC type — set at runtime
uint32_t genSampleRate = 25000UL;
uint16_t genSamplePeriodUs = 40;

// Quarter-wave sine (64 entries)
const uint8_t sineQ[] PROGMEM = {
  128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
  176,179,182,184,187,190,192,195,197,200,202,204,207,209,211,213,
  215,217,219,221,223,224,226,228,229,231,232,233,234,236,237,238,
  239,240,240,241,242,242,243,243,244,244,244,245,245,245,245,245
};

uint8_t sineWave(uint8_t idx) {
  if (idx < 64)  return pgm_read_byte(&sineQ[idx]);
  if (idx < 128) return pgm_read_byte(&sineQ[127 - idx]);
  if (idx < 192) return 255 - pgm_read_byte(&sineQ[idx - 128]);
  return 255 - pgm_read_byte(&sineQ[255 - idx]);
}

// ================== Note Frequency Table ==================
// Replaces log() — octave 4 note frequencies × 10 (C4=2616 → 261.6 Hz)
const uint16_t noteFreqO4x10[] PROGMEM = {
  2616, 2772, 2937, 3111, 3296, 3492, 3700, 3920, 4153, 4400, 4662, 4939
};
const char noteNames[] PROGMEM = "C C#D D#E F F#G G#A A#B ";

// (Boot logo animation removed for flash savings)

// Format voltage × 100 as "X.XX" (integer math, no float)
void fmtDec2(char* buf, uint16_t vx100) {
  char* p = fmtInt(buf, vx100 / 100);
  *p++ = '.';
  uint8_t frac = vx100 % 100;
  *p++ = '0' + frac / 10;
  *p++ = '0' + frac % 10;
  *p++ = '\0';
}

// ================== Shared Buffer ==================
uint8_t buffer[ADC_BUFFER_SIZE];

// ================== Mode Settings ==================
struct ModeSettings { uint8_t param_select, param1, param2; };
ModeSettings modeSettings[NUM_MODES];

uint8_t mode = MODE_LFO, old_mode = MODE_LFO;
uint8_t param_select = 0, param = 1, param1 = 2, param2 = 1;
bool trig = 0, old_trig = 0, SW = 0, old_SW = 0;
unsigned long hideTimer = 0;
bool hide = 0;
int rfrs = 0;
float oldPosition = -999, newPosition = -999;
bool configMenuActive = false;
byte configMenuOption = 1;
unsigned int menuTimer = 5;
int encoderDirection = 1;
uint8_t oledRotation = 0;

// ================== Integer String Formatting ==================
// Replaces dtostrf — saves ~1.5KB by not linking vfprintf float

// Format integer into buffer, returns pointer past last char written
char* fmtInt(char* buf, int16_t val) {
  if (val < 0) { *buf++ = '-'; val = -val; }
  // Write digits in reverse, then reverse
  char tmp[6];
  uint8_t n = 0;
  if (val == 0) { tmp[n++] = '0'; }
  else { while (val > 0) { tmp[n++] = '0' + (val % 10); val /= 10; } }
  for (uint8_t i = n; i > 0; i--) *buf++ = tmp[i - 1];
  *buf = '\0';
  return buf;
}

// Format value with 1 decimal place: fmtDec1(buf, 1234) → "123.4"
char* fmtDec1(char* buf, int16_t valX10) {
  if (valX10 < 0) { *buf++ = '-'; valX10 = -valX10; }
  char* end = fmtInt(buf, valX10 / 10);
  // end points at the null terminator — append decimal
  *end++ = '.';
  *end++ = '0' + (valX10 % 10);
  *end = '\0';
  return end;
}

// Format frequency: smart format based on range
void fmtFreq(char* buf, uint16_t freqX10) {
  if (freqX10 < 10) {          // < 1.0 Hz
    fmtDec1(buf, freqX10);
  } else if (freqX10 < 1000) { // < 100 Hz: show 1 decimal
    fmtDec1(buf, freqX10);
  } else {                      // >= 100 Hz: integer
    fmtInt(buf, freqX10 / 10);
  }
  // Append "Hz"
  char* p = buf + strlen(buf);
  *p++ = 'H'; *p++ = 'z'; *p = '\0';
}

// ================== Note Detection (no log()!) ==================
void frequencyToNote(uint16_t freqX10, char* note, int8_t* octave, int8_t* cents) {
  // Normalize frequency to octave 4 range (261.6 – 523.2 Hz → 2616–5232 in x10)
  uint16_t fx10 = freqX10;
  int8_t oct = 4;

  // Scale down to octave 4 range
  while (fx10 >= 5232 && oct < 9) { fx10 = (fx10 + 1) >> 1; oct++; }
  // Scale up to octave 4 range
  while (fx10 < 2616 && oct > 0) { fx10 <<= 1; oct--; }

  // Find closest note in octave 4
  uint8_t bestNote = 0;
  int16_t bestDiff = 32767;
  for (uint8_t i = 0; i < 12; i++) {
    uint16_t nf = pgm_read_word(&noteFreqO4x10[i]);
    int16_t diff = (int16_t)fx10 - (int16_t)nf;
    int16_t absDiff = diff < 0 ? -diff : diff;
    if (absDiff < (bestDiff < 0 ? -bestDiff : bestDiff)) {
      bestDiff = diff;
      bestNote = i;
    }
  }

  // Handle wrap: if closest is B and we're closer to C of next octave
  if (bestNote == 11) {
    int16_t cNext = 5232;  // C5 x10 (one octave up from C4=2616)
    int16_t diffC = (int16_t)fx10 - (int16_t)cNext;
    if ((diffC < 0 ? -diffC : diffC) < (bestDiff < 0 ? -bestDiff : bestDiff)) {
      bestNote = 0; oct++; bestDiff = diffC;
    }
  }

  *octave = oct;

  // Cents: approximate cents = (diff / noteFreq) * 1731
  uint16_t nf = pgm_read_word(&noteFreqO4x10[bestNote]);
  *cents = (int8_t)(((int32_t)bestDiff * 1731L) / (int32_t)nf / 10);
  *cents = constrain(*cents, -50, 50);

  uint8_t idx = bestNote * 2;
  note[0] = pgm_read_byte(&noteNames[idx]);
  note[1] = pgm_read_byte(&noteNames[idx + 1]);
  note[2] = '\0';
  if (note[1] == ' ') note[1] = '\0';
}

// ================== Function Declarations ==================
void detectHardware();
void initDisplay();
void setupMode(uint8_t mode);
void runLFOMode(bool showParams);
void runWaveMode(bool showParams);
void runTunerMode(bool showParams);
void runGeneratorMode(bool showParams);
void drawParameterBar(bool showParams);
void configMenu();
void saveCurrentModeToRAM();
void saveAllSettings();
void loadAllSettings();
void resetEEPROMDefaults();
void startADCSampling(uint16_t numSamples, uint8_t prescaler, uint8_t delaySkip, uint8_t src, bool rearm);
void drawWave(bool showParams);
void stopADCSampling();
uint8_t fastAnalogRead();
float detectFrequencyZC();
inline void dacWrite(uint8_t value);
inline uint8_t dcVoltageToDac(uint8_t param2_x10);
void scanMCP4725();
void dacInit();
void dacStop();
uint8_t generateSample(uint8_t waveform, uint8_t idx);
uint32_t calcPhaseInc(uint8_t freqIdx, uint32_t sr);

// ================== ADC Interrupt ==================
// The ADC free-runs. The trigger is evaluated on EVERY conversion (sub-pixel
// trigger precision at slow time bases); samples are stored decimated once
// running. The analog front-end is inverting (U3A), so a rising input edge
// is a FALLING ADC value.
ISR(ADC_vect) {
  uint8_t s = ADCH;
  uint8_t pin = PIND & _BV(7);              // TRIGGER_PIN D7 (100k pull-down)
  bool rise = pin && !trigPinLast;
  trigPinLast = pin;
  if (rise) extEdge = true;

  switch (capState) {
    case CAP_ARMED:
      // An external edge is always a valid trigger point. Otherwise apply the
      // software level (falling ADC = rising input), or free-run (TRIG_NONE).
      if (!rise) {
        if (capSrc == TRIG_EXT) return;
        if (capSrc == TRIG_SOFT) {
          if (s >= trigHi) { trigArmed = true; return; }   // input below level
          if (!(trigArmed && s <= trigLo)) return;          // wait for the crossing
        }
      }
      capState = CAP_RUN;
      adcDelayCounter = 0;
      // fall through: the triggering conversion is the first stored sample
    case CAP_RUN:
      if (adcDelayCounter) { adcDelayCounter--; return; }
      adcDelayCounter = adcDelayTarget;
      buffer[capBase + adcSampleIndex++] = s;
      if (adcSampleIndex >= adcTargetSamples) {
        readyBase = capBase;
        capReady = true;
        if (capRearm) {   // WAVE: swap halves and wait for the next trigger at once
          capBase ^= 128; adcSampleIndex = 0; trigArmed = false; capState = CAP_ARMED;
        } else capState = CAP_DONE;
      }
      break;
    default: break;
  }
}

// Arm a capture. src = TRIG_NONE starts on the next conversion. With rearm the
// ISR alternates between buffer[0..127] and buffer[128..255] and never idles.
// The ADC keeps free-running after CAP_DONE so TRIGGER_PIN edges are still seen.
void startADCSampling(uint16_t numSamples, uint8_t prescaler, uint8_t delaySkip, uint8_t src, bool rearm) {
  cli();
  adcSampleIndex = 0;
  adcTargetSamples = min(numSamples, (uint16_t)ADC_BUFFER_SIZE);
  adcDelayTarget = delaySkip;
  trigArmed = false;
  capReady = false;
  capRearm = rearm;
  capBase = 0;
  capSrc = src;
  capState = CAP_ARMED;
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (ANALOG_INPUT_PIN & 0x07);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (prescaler & 0x07);
  ADCSRB = 0;
  sei();
}

void stopADCSampling() {
  ADCSRA &= ~((1 << ADIE) | (1 << ADATE));
  capState = CAP_IDLE;
}

uint8_t fastAnalogRead() {
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (ANALOG_INPUT_PIN & 0x07);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (0x05);
  while (ADCSRA & (1 << ADSC));
  return ADCH;
}

// ================== DAC Output ==================
// Two DAC paths depending on MCU:
//   LGT8F:     Native on-board DAC on D4, DEFAULT (VCC) reference
//   ATmega328: MCP4725 12-bit I2C DAC (400kHz I2C → ~10kHz sample rate)
// Calibration via separate SCOPE_CAL.ino firmware + dac-calibrator.html

#if IS_LGT8F
  #ifndef DAC0
    #define DAC0 4
  #endif
  #ifndef ANALOG
    #define ANALOG 2
  #endif
#endif

// Scan for MCP4725 at common addresses (0x60-0x63)
void scanMCP4725() {
  for (uint8_t addr = 0x60; addr <= 0x63; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      dacI2CAddr = addr;
      dacIsI2C = true;
      return;
    }
  }
  dacI2CAddr = 0;
  dacIsI2C = false;
}

void dacInit() {
  #if IS_LGT8F
    // LGT8F native DAC — VCC reference (proven linear per Wolles)
    analogReference(DEFAULT);
    pinMode(DAC0, ANALOG);
    analogWrite(DAC0, 0);
    genSampleRate = 50000UL;
    genSamplePeriodUs = 20;
  #else
    // ATmega328P: MCP4725 already scanned in detectHardware()
    if (dacIsI2C) {
      Wire.beginTransmission(dacI2CAddr);
      Wire.write(0x40);
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();
    }
    genSampleRate = 10000UL;
    genSamplePeriodUs = 100;
  #endif
}

void dacStop() {
  #if IS_LGT8F
    analogWrite(DAC0, 0);
    pinMode(DAC0, INPUT);
  #else
    if (dacIsI2C) {
      Wire.beginTransmission(dacI2CAddr);
      Wire.write(0x40);
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();
    }
  #endif
}

inline void dacWrite(uint8_t value) {
  int16_t cal = ((int16_t)value * calGain) >> 7;
  cal += calOffset;
  if (cal < 0) cal = 0;
  if (cal > 255) cal = 255;
  #if IS_LGT8F
    analogWrite(DAC0, (uint8_t)cal);
  #else
    if (dacIsI2C) {
      uint16_t val12 = (uint16_t)cal << 4;  // 8-bit → 12-bit
      Wire.beginTransmission(dacI2CAddr);
      Wire.write((uint8_t)(val12 >> 8));
      Wire.write((uint8_t)(val12 & 0xFF));
      Wire.endTransmission();
    }
  #endif
}

// Convert DC voltage param (0.1V steps) to DAC byte
// GEN_DC_MAX maps to DAC 255 (full scale = VCC for both DAC types)
inline uint8_t dcVoltageToDac(uint8_t param2_x10) {
  return (uint8_t)((uint16_t)param2_x10 * 255 / GEN_DC_MAX);
}

uint8_t generateSample(uint8_t waveform, uint8_t idx) {
  switch (waveform) {
    case 1: return sineWave(idx);
    case 2: return (idx < 128) ? idx * 2 : (255 - idx) * 2;
    case 3: return idx;
    case 4: return (idx < 128) ? 255 : 0;
    default: return 128;
  }
}

// Integer phase increment: avoids float multiply/divide
// freqX10 * (2^32 / 10) / sampleRate — all uint64 math
uint32_t calcPhaseInc(uint8_t freqIdx, uint32_t sr) {
  freqIdx = constrain(freqIdx, 1, GEN_NUM_FREQS);
  uint16_t fX10 = pgm_read_word(&genFreqTableX10[freqIdx - 1]);
  return (uint32_t)((uint64_t)fX10 * 429496730ULL / sr);
}

// ================== Hardware Detection ==================
void detectHardware() {
  #if IS_LGT8F
    isLGT8F = true;
  #else
    isLGT8F = false;
  #endif
  isHWv25 = (analogRead(IDENT_HW_PIN) < 100);

  // Determine DAC availability
  #if IS_LGT8F
    genAvailable = isHWv25;  // Native DAC on D4 (v2.5 only)
  #else
    if (isHWv25) {
      // ATmega328P on v2.5: scan for MCP4725 I2C DAC
      Wire.begin();
      Wire.setClock(400000UL);
      pinMode(SDA, INPUT_PULLUP);
      pinMode(SCL, INPUT_PULLUP);
      scanMCP4725();
      genAvailable = dacIsI2C;
    } else {
      genAvailable = false;
    }
  #endif

  encoder = isHWv25 ? new Encoder(V25_ENCODER_PIN_A, V25_ENCODER_PIN_B)
                     : new Encoder(V2_ENCODER_PIN_A, V2_ENCODER_PIN_B);
}

void initDisplay() {
  if (isHWv25) {
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT,
                                   &SPI, V25_OLED_DC, V25_OLED_RESET, V25_OLED_CS);
  } else {
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT,
                                   V2_OLED_MOSI, V2_OLED_CLK, V2_OLED_DC, V2_OLED_RESET, V2_OLED_CS);
  }
  display->begin(SSD1306_SWITCHCAPVCC);
  display->setRotation(oledRotation);
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(WHITE);
}

// ================== Zero-Crossing Detection ==================
float detectFrequencyZC() {
  uint8_t minV = 255, maxV = 0;
  for (uint16_t i = 0; i < 256; i++) {
    if (buffer[i] < minV) minV = buffer[i];
    if (buffer[i] > maxV) maxV = buffer[i];
  }
  uint8_t range = maxV - minV;
  if (range < 30) return 0;

  uint8_t center = (minV + maxV) / 2;
  uint8_t hyst = range / 6;
  if (hyst < 5) hyst = 5;

  uint8_t crossings = 0;
  uint16_t first = 0, last = 0;
  bool above = buffer[0] > center;

  for (uint16_t i = 1; i < 256; i++) {
    if (!above && buffer[i] > center + hyst) {
      above = true; crossings++;
      if (!first) first = i;
      last = i;
    } else if (above && buffer[i] < center - hyst) {
      above = false;
    }
  }
  if (crossings < 2) return 0;
  return sampleRateHz / ((float)(last - first) / (float)(crossings - 1));
}

// ================== EEPROM ==================
void resetEEPROMDefaults() {
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.update(ENCODER_DIR_ADDR, 1);
  EEPROM.update(OLED_ROT_ADDR, 0);
  EEPROM.update(MENUTIMER_DIR_ADDR, 5);
  EEPROM.update(EEPROM_MODE_ADDR, MODE_LFO);
  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int ba = EEPROM_PARAM_SELECT_ADDR + (m * 3);
    EEPROM.update(ba, 0);
    EEPROM.update(ba + 1, 2);
    EEPROM.update(ba + 2, 1);
  }
  EEPROM.update(EEPROM_CAL_OFFSET_ADDR, 0);
  EEPROM.update(EEPROM_CAL_GAIN_ADDR, 128);
}

void loadAllSettings() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) resetEEPROMDefaults();

  calOffset = (int8_t)EEPROM.read(EEPROM_CAL_OFFSET_ADDR);
  calGain = EEPROM.read(EEPROM_CAL_GAIN_ADDR);
  if (calGain < 32 || calGain > 255) calGain = 128;

  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int ba = EEPROM_PARAM_SELECT_ADDR + (m * 3);
    modeSettings[m].param_select = EEPROM.read(ba);
    modeSettings[m].param1 = EEPROM.read(ba + 1);
    modeSettings[m].param2 = EEPROM.read(ba + 2);
    if (modeSettings[m].param_select > 3) modeSettings[m].param_select = 0;

    switch (m + 1) {
      case MODE_LFO:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 8);
        modeSettings[m].param2 = constrain(modeSettings[m].param2, -6, 10);
        if (!modeSettings[m].param1) modeSettings[m].param1 = 4;
        if (!modeSettings[m].param2) modeSettings[m].param2 = 1;
        break;
      case MODE_WAVE:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 8);
        modeSettings[m].param2 = constrain(modeSettings[m].param2, 1, 6);
        if (!modeSettings[m].param1) modeSettings[m].param1 = 8;
        if (!modeSettings[m].param2) modeSettings[m].param2 = 1;
        break;
      case MODE_TUNER:
        modeSettings[m].param1 = 1;
        modeSettings[m].param2 = 1;
        break;
      case MODE_GEN:
        modeSettings[m].param1 = constrain(modeSettings[m].param1, 1, 5);
        if (!modeSettings[m].param1) modeSettings[m].param1 = 1;
        if (modeSettings[m].param1 == 5) modeSettings[m].param2 = constrain(modeSettings[m].param2, 0, GEN_DC_MAX);
        else { modeSettings[m].param2 = constrain(modeSettings[m].param2, 1, GEN_NUM_FREQS); if (!modeSettings[m].param2) modeSettings[m].param2 = 4; }
        break;
    }
  }
}

void saveCurrentModeToRAM() {
  uint8_t idx = mode - 1;
  if (idx < NUM_MODES) { modeSettings[idx].param_select = param_select; modeSettings[idx].param1 = param1; modeSettings[idx].param2 = param2; }
}

void saveAllSettings() {
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.update(EEPROM_MODE_ADDR, mode);
  for (uint8_t m = 0; m < NUM_MODES; m++) {
    int ba = EEPROM_PARAM_SELECT_ADDR + (m * 3);
    EEPROM.update(ba, modeSettings[m].param_select);
    EEPROM.update(ba + 1, modeSettings[m].param1);
    EEPROM.update(ba + 2, modeSettings[m].param2);
  }
  EEPROM.update(EEPROM_CAL_OFFSET_ADDR, (uint8_t)calOffset);
  EEPROM.update(EEPROM_CAL_GAIN_ADDR, calGain);

  display->fillRect(0, 54, 128, 10, WHITE);
  display->setTextColor(BLACK);
  display->setCursor(4, 55);
  display->print(F("SETTINGS SAVED"));
  display->display();
  delay(500);
}

// ================== Setup ==================
void setup() {
  detectHardware();

  EEPROM.get(ENCODER_DIR_ADDR, encoderDirection);
  if (encoderDirection != 1 && encoderDirection != -1) encoderDirection = 1;
  oledRotation = EEPROM.read(OLED_ROT_ADDR);
  if (!(oledRotation == 0 || oledRotation == 2)) oledRotation = 0;
  EEPROM.get(MENUTIMER_DIR_ADDR, menuTimer);
  if (menuTimer < 1 || menuTimer > 60) menuTimer = 5;

  uint8_t lastMode = EEPROM.read(EEPROM_MODE_ADDR);
  mode = (lastMode >= MODE_LFO && lastMode <= MODE_GEN) ? lastMode : MODE_LFO;
  if (mode == MODE_GEN && !genAvailable) mode = MODE_LFO;

  initDisplay();

  // Text splash
  display->clearDisplay();
  display->setTextSize(2);
  display->setCursor(16, 10);
  display->print(F("SCOPE"));
  display->setTextSize(1);
  display->setCursor(16, 32);
  display->print(F("Modulove v3.7"));
  display->setCursor(0, 56);
  display->print(isHWv25 ? F("v2.5") : F("v2"));
  display->print(isLGT8F ? F(" LGT") : F(" 328"));
  if (genAvailable) display->print(dacIsI2C ? F(" I2C") : F(" DAC"));
  display->display();
  delay(800);

  pinMode(OFFSET_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FILTER_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);

  TCCR2B = (TCCR2B & B11111000) | B00000001;

  loadAllSettings();
  setupMode(mode);
}

// ================== Main Loop ==================
void loop() {
  old_SW = SW;
  old_mode = mode;
  SW = (digitalRead(BUTTON_PIN) == LOW);

  static unsigned long bStart = 0;
  static bool isLP = false, hasSaved = false, hasConfig = false;

  if (SW && !old_SW) { bStart = millis(); isLP = true; hasSaved = false; hasConfig = false; }
  if (!SW && old_SW) isLP = false;
  if (isLP && !hasSaved && (millis() - bStart >= 1000)) { saveCurrentModeToRAM(); saveAllSettings(); hasSaved = true; }
  if (SW && !configMenuActive && !hasConfig && hasSaved && (millis() - bStart >= 3000)) {
    configMenuActive = true; hasConfig = true; configMenuOption = 1;
    oldPosition = newPosition = encoder->read();
  }

  if (configMenuActive) { configMenu(); return; }

  newPosition = encoderDirection * encoder->read();
  if (old_SW == 0 && SW == 1 && param_select == param) { param_select = 0; hideTimer = millis(); }
  else if (old_SW == 0 && SW == 1 && (param >= 1 && param <= 3)) { param_select = param; hideTimer = millis(); }

  newPosition = encoderDirection * encoder->read();
  int8_t enc = 0;
  if ((newPosition - 3) / 4 > oldPosition / 4) { oldPosition = newPosition; hideTimer = millis(); enc = -1; }
  else if ((newPosition + 3) / 4 < oldPosition / 4) { oldPosition = newPosition; hideTimer = millis(); enc = 1; }

  if (enc) {
    switch (param_select) {
      case 0: { // Param slot rollover
        uint8_t mx = (mode == MODE_TUNER) ? 1 : 3;
        param += enc;
        if (param < 1) param = mx;
        if (param > mx) param = 1;
      } break;
      case 1: // Mode rollover (skip GEN on v2)
        mode += enc;
        if (mode < 1) mode = NUM_MODES;
        if (mode > NUM_MODES) mode = 1;
        if (mode == MODE_GEN && !genAvailable) { mode += enc; if (mode < 1) mode = NUM_MODES; if (mode > NUM_MODES) mode = 1; }
        break;
      case 2: { // Param1 rollover
        int8_t mn = 1, mx;
        switch (mode) { case MODE_LFO: case MODE_WAVE: mx = 8; break; case MODE_TUNER: mx = 1; break; case MODE_GEN: mx = 5; break; default: mx = 8; }
        param1 += enc;
        if (param1 < mn) param1 = mx;
        if (param1 > mx) param1 = mn;
        if (mode == MODE_GEN) {
          if (param1 == 5) { if (param2 > GEN_DC_MAX) param2 = GEN_DC_MAX / 2; }
          else { if (param2 > GEN_NUM_FREQS || param2 == 0) param2 = 4; }
        }
      } break;
      case 3: { // Param2 rollover
        int8_t mn, mx;
        switch (mode) {
          case MODE_LFO: mn = -6; mx = 10; break;
          case MODE_WAVE: mn = 1; mx = 6; break;
          case MODE_GEN: mn = (param1 == 5) ? 0 : 1; mx = (param1 == 5) ? GEN_DC_MAX : GEN_NUM_FREQS; break;
          default: mn = 1; mx = 1; break;
        }
        param2 += enc;
        if (param2 < mn) param2 = mx;
        if (param2 > mx) param2 = mn;
      } break;
    }
  }

  if (old_mode != mode) {
    byte ps = param_select;
    saveCurrentModeToRAM();
    setupMode(mode);
    display->clearDisplay();
    if (ps == 1) param_select = 1;
    hideTimer = millis();
  }

  hide = (millis() - hideTimer >= (menuTimer * 1000UL));
  bool sp = !hide;

  switch (mode) {
    case MODE_LFO:   runLFOMode(sp);       break;
    case MODE_WAVE:  runWaveMode(sp);      break;
    case MODE_TUNER: runTunerMode(sp);     break;
    case MODE_GEN:   runGeneratorMode(sp); return;  // GEN handles its own display
  }
  display->display();
}

// ================== Mode Setup ==================
void setupMode(uint8_t m) {
  stopADCSampling();
  if (genAvailable) dacStop();  // Always release DAC pin when switching modes
  uint8_t idx = m - 1;
  if (idx < NUM_MODES) { param_select = modeSettings[idx].param_select; param1 = modeSettings[idx].param1; param2 = modeSettings[idx].param2; }

  switch (m) {
    case MODE_LFO:
      pinMode(FILTER_PIN, INPUT);
      analogWrite(OFFSET_PIN, 0);
      ADCSRA = (ADCSRA & 0xF8) | 0x04;
      break;
    case MODE_WAVE:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      extEdge = false;
      break;
    case MODE_TUNER:
      analogWrite(OFFSET_PIN, 127);
      pinMode(FILTER_PIN, INPUT);
      smoothedFrequency = 0;
      tunerSampleRate = 2;
      break;
    case MODE_GEN:
      analogWrite(OFFSET_PIN, 0);
      pinMode(FILTER_PIN, INPUT);
      dacInit();  // Enable DAC hardware on D4
      genPhase = 0;
      if (param1 != 5) { genPhaseInc = calcPhaseInc(param2, genSampleRate); }
      else genPhaseInc = 0;
      break;
  }
  memset(buffer, 0, sizeof(buffer));
  waveState = 0;
  tunerState = 0;
  rfrs = 0;
}

// ================== LFO Mode (merged with SHOT) ==================
// Continuous scrolling waveform. Trigger pin = single-shot freeze.
void runLFOMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);
  param2 = constrain(param2, -6, 10);

  old_trig = trig;
  trig = digitalRead(TRIGGER_PIN);

  static bool frozen = false;
  static unsigned long frozenAt = 0;

  // Trigger rising edge → freeze display with captured waveform
  if (!old_trig && trig) {
    frozen = true;
    frozenAt = millis();
  }
  // Unfreeze after 3 seconds or next trigger
  if (frozen && (millis() - frozenAt > 3000)) frozen = false;

  if (!frozen) {
    uint8_t s = fastAnalogRead() >> 2;
    memmove(&buffer[1], &buffer[0], 127);
    buffer[0] = s;
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 30) {
    lastDraw = millis();
    display->clearDisplay();

    int step = (9 - param1), voff = (param2 - 1) * 6, segs = 126 / step;
    for (int i = 0; i < segs; i++) {
      int x1 = 127 - (i * step), y1 = constrain(buffer[i] + voff, 0, 63);
      int x2 = 127 - ((i + 1) * step), y2 = constrain(buffer[i + 1] + voff, 0, 63);
      display->drawLine(x1, y1, x2, y2, WHITE);
    }

    // Trigger indicator
    if (frozen) {
      display->setCursor(0, 56);
      display->print(F("TRIG"));
    }

    if (showParams) drawParameterBar(true);
  }
}

// ================== Wave Mode ==================
// Triggered sweep, drawn left to right from the trigger point. The ISR captures
// continuously (see ISR); this function only picks the trigger source, copies
// finished sweeps and draws them at most every R: ms.
//   TRIGGER jack patched -> every rising edge on TRIGGER_PIN starts a sweep;
//                           the last sweep is held while no edge arrives
//   otherwise            -> software trigger at the mid-level of the last sweep
//                           (rising input); after AUTO_TRIG_WAIT_MS without a
//                           crossing one sweep free-runs (auto)
#define EXT_TRIG_HOLD_MS  10000UL  // external trigger counts as present this long after an edge
#define AUTO_TRIG_WAIT_MS 100UL
#define SWEEP_TIMEOUT_MS  600UL    // longest sweep is ~430 ms: longer in CAP_RUN = stalled, re-arm

void drawWave(bool showParams) {
  display->clearDisplay();
  for (uint8_t i = 1; i < 128; i++)
    display->drawLine(i - 1, disp[i - 1] >> 2, i, disp[i] >> 2, WHITE);
  if (showParams) drawParameterBar(true);
}

void runWaveMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 8);
  param2 = constrain(param2, 1, 6);

  static unsigned long lastDraw = 0, lastSweep = 0, runSince = 0;
  static unsigned long lastExtEdge = (unsigned long)0 - EXT_TRIG_HOLD_MS;   // stale at boot
  static uint16_t lastUI = 0xFFFF;
  static bool haveNew = false, softOK = false;
  unsigned long interval = 20UL + (param2 - 1) * 10UL;
  unsigned long now = millis();

  if (extEdge) { extEdge = false; lastExtEdge = now; }
  bool extActive = (now - lastExtEdge < EXT_TRIG_HOLD_MS) || digitalRead(TRIGGER_PIN);

  // Menu/param change: time base -> restart capture; anything else -> redraw
  // the held sweep at once so navigation never waits for the next trigger
  uint16_t ui = (showParams ? 0x8000 : 0) | ((uint16_t)param_select << 12) | ((uint16_t)param << 8)
              | ((uint16_t)param1 << 4) | (param2 & 0x0F);
  if (ui != lastUI) {
    bool timebaseChanged = ((ui ^ lastUI) & 0x00F0) != 0;
    lastUI = ui;
    if (timebaseChanged) { stopADCSampling(); waveState = 0; }
    else drawWave(showParams);
  }

  if (waveState == 0) {   // (re)start: first sweep free-runs to learn the signal level
    startADCSampling(128, ADC_PS_WAVE, pgm_read_byte(&waveSkip[param1 - 1]), extActive ? TRIG_EXT : TRIG_NONE, true);
    lastSweep = runSince = now; waveState = 1; softOK = false;
  }

  // Finished sweep -> display copy (ISR is already filling the other half)
  if (capReady) {
    capReady = false;
    memcpy(disp, buffer + readyBase, 128);
    haveNew = true; lastSweep = now;
    uint8_t mn = 255, mx = 0;
    for (uint8_t i = 0; i < 128; i++) { uint8_t v = disp[i]; if (v < mn) mn = v; if (v > mx) mx = v; }
    uint8_t range = mx - mn;
    softOK = (range >= 8);
    if (softOK) {
      uint8_t hyst = range >> 3; if (hyst < 3) hyst = 3;
      uint8_t mid = mn + (range >> 1);
      trigLo = mid - hyst; trigHi = mid + hyst;
    }
  }

  // Trigger source for the sweep the ISR is waiting on
  if (capState == CAP_ARMED) {
    runSince = now;
    if (extActive) capSrc = TRIG_EXT;
    else if (now - lastSweep > AUTO_TRIG_WAIT_MS) capSrc = TRIG_NONE;   // auto: free-run this sweep
    else capSrc = softOK ? TRIG_SOFT : TRIG_NONE;
  } else if (capState == CAP_RUN && now - runSince > SWEEP_TIMEOUT_MS) {
    stopADCSampling(); waveState = 0;                                    // safety: ADC stalled
  }

  if (haveNew && now - lastDraw >= interval) { lastDraw = now; haveNew = false; drawWave(showParams); }
}

// ================== Tuner (ZC-only, auto sample rate) ==================
void runTunerMode(bool showParams) {
  param = 1;  // Tuner has no adjustable params

  static unsigned long lastUp = 0, stateStart = 0;
  static float lastValid = 0;

  switch (tunerState) {
    case 0:
      if (millis() - lastUp >= 40) {
        uint8_t ps, ds;
        // Auto sample rate based on detected frequency
        // ATmega328P: 13 ADC clocks/conversion (free-running)
        // LGT8F:     ~22 ADC clocks/conversion (measured empirically)
        if (lastValid < 80 || tunerSampleRate == 1) {
          ps = 0x07; ds = 1; tunerSampleRate = 1;
          sampleRateHz = IS_LGT8F ? 5682.0f : 4808.0f;  // LGT8F: 32M/128/22/2
        } else if (lastValid < 200 || tunerSampleRate == 2) {
          ps = 0x07; ds = 0; tunerSampleRate = 2;
          sampleRateHz = IS_LGT8F ? 11364.0f : 9615.0f;  // LGT8F: 32M/128/22
        } else {
          ps = 0x06; ds = 0; tunerSampleRate = 3;
          sampleRateHz = IS_LGT8F ? 22727.0f : 19230.0f;  // LGT8F: 32M/64/22
        }
        startADCSampling(256, ps, ds, TRIG_NONE, false); tunerState = 1; stateStart = millis();
      } break;
    case 1:
      if (capState == CAP_DONE) tunerState = 2;
      else if (millis() - stateStart > 200) { stopADCSampling(); tunerState = 0; }  // timeout — retry
      break;
    case 2: {
      float raw = detectFrequencyZC();
      if (raw > 15 && raw < 5000) {
        lastValid = raw;
        if (smoothedFrequency < 10) smoothedFrequency = raw;
        else { float a = (abs(raw - smoothedFrequency) / smoothedFrequency > 0.1f) ? 0.5f : 0.35f; smoothedFrequency = smoothedFrequency * (1 - a) + raw * a; }
        tunerSampleRate = (smoothedFrequency < 60) ? 1 : (smoothedFrequency < 150) ? 2 : 3;
      } else { smoothedFrequency *= 0.85f; if (smoothedFrequency < 15) { smoothedFrequency = 0; tunerSampleRate = 2; } }
      lastUp = millis(); tunerState = 0;
    } break;
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw < 80) return;
  lastDraw = millis();

  display->clearDisplay();
  int yO = showParams ? 10 : 0;

  if (smoothedFrequency > 15) {
    uint16_t fx10 = (uint16_t)(smoothedFrequency * 10.0f);
    char note[3]; int8_t oct, cents;
    frequencyToNote(fx10, note, &oct, &cents);

    display->setTextSize(2);
    int nW = (strlen(note) * 12) + 12;
    display->setCursor((128 - nW) / 2, yO + 4);
    display->print(note); display->print((int)oct);

    display->setTextSize(1);
    if (cents < 0) { display->setCursor(4, yO + 8); display->print(cents); display->print('c'); }
    else if (cents > 0) { display->setCursor(cents < 10 ? 110 : 104, yO + 8); display->print('+'); display->print(cents); display->print('c'); }
    else { display->setCursor(110, yO + 8); display->print(F("OK")); }

    char hz[12];
    fmtFreq(hz, fx10);
    display->setCursor((128 - strlen(hz) * 6) / 2, yO + 24);
    display->print(hz);

    // Waveform preview
    int wY = showParams ? 36 : 34;
    for (uint8_t i = 1; i < 127; i++) {
      int y = constrain(wY + 11 - ((buffer[i] * 12) >> 8), wY, wY + 11);
      display->drawPixel(i, y, WHITE);
    }

    int bY = showParams ? 52 : 50;
    display->drawRect(14, bY, 100, 6, WHITE);
    display->drawFastVLine(64, bY - 2, 10, WHITE);
    display->fillRect(62 + constrain(cents, -50, 50), bY + 1, 5, 4, WHITE);
  } else {
    display->setTextSize(2); display->setCursor(44, yO + 8); display->print(F("---"));
    display->setTextSize(1); display->setCursor(34, yO + 26); display->print(F("No signal"));
    int bY = showParams ? 52 : 50;
    display->drawRect(14, bY, 100, 6, WHITE);
    display->drawFastVLine(64, bY - 2, 10, WHITE);
  }
  display->setTextSize(1);
  if (showParams) drawParameterBar(true);
}




// ================== Generator Mode ==================
void runGeneratorMode(bool showParams) {
  param  = constrain(param, 1, 3);
  param1 = constrain(param1, 1, 5);

  static unsigned long lastDraw = 0;
  static unsigned long lastSample = 0;

  if (!genAvailable) {
    if (millis() - lastDraw >= 200) {
      lastDraw = millis();
      display->clearDisplay();
      display->setTextSize(1);
      display->setCursor(10, 24);
      display->print(F("GEN needs v2.5"));
      if (showParams) drawParameterBar(true);
      display->display();
    }
    return;
  }

  // Recalculate phase inc
  if (param1 != 5) {
    param2 = constrain(param2, 1, GEN_NUM_FREQS);
    genPhaseInc = calcPhaseInc(param2, genSampleRate);
  } else {
    param2 = constrain(param2, 0, GEN_DC_MAX);
    genPhaseInc = 0;
  }

  // DC mode: just hold the value, no tight loop needed
  if (param1 == 5) {
    dacWrite(dcVoltageToDac(param2));
  } else {
    // Compensate phase for any gap since last sample
    // (loop overhead ~0.5ms + display update ~5-8ms every 80ms)
    unsigned long nowComp = micros();
    unsigned long gap = nowComp - lastSample;
    if (gap > genSamplePeriodUs && gap < 50000UL && genPhaseInc > 0) {
      uint32_t missed = gap / genSamplePeriodUs;
      genPhase += missed * genPhaseInc;
      lastSample += missed * genSamplePeriodUs;
    } else if (gap >= 50000UL) {
      lastSample = nowComp;  // Too long — resync without phase jump
    }

    // Waveform mode: output samples in tight loop for ~4ms
    unsigned long burstEnd = micros() + 4000;
    while ((long)(micros() - burstEnd) < 0) {
      unsigned long nowUs = micros();
      if (nowUs - lastSample >= genSamplePeriodUs) {
        lastSample += genSamplePeriodUs;
        dacWrite(generateSample(param1, genPhase >> 24));
        genPhase += genPhaseInc;
      }
    }
  }

  // Display update — skip entirely during waveform output when menu hidden
  // This eliminates the ~5ms SPI glitch from display->display()
  // DC mode always updates (no timing-sensitive output)
  static bool genDispDone = false;  // Track if we've drawn the "idle" screen

  if (param1 != 5 && !showParams && genDispDone) return;  // Waveform + menu hidden → no display updates
  if (showParams) genDispDone = false;  // User interacting → allow updates again

  if (millis() - lastDraw < 80) return;
  lastDraw = millis();
  genDispDone = !showParams;  // After drawing with menu hidden, stop updates

  display->clearDisplay();
  int yO = showParams ? 10 : 0;

  if (param1 == 5) {
    // DC mode display
    display->setTextSize(2);
    char vStr[8];
    fmtDec1(vStr, param2);  // param2 is already voltage × 10
    int vW = strlen(vStr) * 12 + 12;
    display->setCursor((128 - vW) / 2, yO + 6);
    display->print(vStr); display->print('V');

    int barW = map(param2, 0, GEN_DC_MAX, 0, 100);
    int barY = yO + 28;
    display->drawRect(14, barY, 100, 8, WHITE);
    if (barW > 0) display->fillRect(14, barY, barW, 8, WHITE);

    display->setTextSize(1);
    display->setCursor(0, yO + 42);
    display->print(F("DAC:")); display->print(dcVoltageToDac(param2));
    display->setCursor(60, yO + 42);
    display->print(F("CAL:"));
    if (calOffset >= 0) display->print('+');
    display->print(calOffset);
  } else {
    // Waveform mode — preview (drawPixel is ~100x faster than drawLine)
    int pH = 28, pY = yO + 2;
    for (int x = 0; x < 128; x++) {
      uint8_t idx = (uint8_t)((uint16_t)x * 512 / 128);
      int y = pY + pH - 1 - ((int)generateSample(param1, idx) * (pH - 1) / 255);
      display->drawPixel(x, constrain(y, pY, pY + pH - 1), WHITE);
    }

    // Freq readout
    uint16_t fX10 = pgm_read_word(&genFreqTableX10[constrain(param2, 1, GEN_NUM_FREQS) - 1]);
    char fStr[12];
    fmtFreq(fStr, fX10);
    display->setTextSize(1);
    display->setCursor((128 - strlen(fStr) * 6) / 2, yO + 34);
    display->print(fStr);

    // Wave name + cal
    char wn[4]; memcpy_P(wn, genWaveNames[param1 - 1], 4);
    display->setCursor(0, yO + 46);
    display->print(wn);
    display->setCursor(60, yO + 46);
    display->print(F("CAL:"));
    if (calOffset >= 0) display->print('+');
    display->print(calOffset);
  }

  display->setTextSize(1);
  if (showParams) drawParameterBar(true);

  // SPI display transfer (~5ms) — phase gap compensated at next burst start
  display->display();
}

// ================== Parameter Bar ==================
// Navigation feedback:
//   browsing (param_select == 0) -> slot under the cursor (param) is underlined
//   editing  (param_select == n) -> slot n is drawn inverted
void barSlotBegin(uint8_t slot, int16_t x) {
  bool editing = (param_select == slot);
  display->setTextColor(editing ? BLACK : WHITE, editing ? WHITE : BLACK);
  display->setCursor(x, 0);
}

void barSlotEnd(uint8_t slot, int16_t x) {
  if (param_select == 0 && param == slot)
    display->drawFastHLine(x, 8, display->getCursorX() - x - 1, WHITE);
}

void drawParameterBar(bool showParams) {
  if (!showParams) return;
  display->setTextSize(1);

  // Slot 1: Mode
  barSlotBegin(1, 0);
  switch (mode) {
    case MODE_LFO:   display->print(F("LFO"));  break;
    case MODE_WAVE:  display->print(F("WAVE")); break;
    case MODE_TUNER: display->print(F("TUNE")); break;
    case MODE_GEN:   display->print(F("GEN"));  break;
  }
  barSlotEnd(1, 0);

  // Slot 2: Param1
  barSlotBegin(2, 36);
  switch (mode) {
    case MODE_LFO: case MODE_WAVE:
      display->print(F("T:")); display->print(param1); break;
    case MODE_TUNER:
      display->print(F("ZC")); break;
    case MODE_GEN: {
      char w[4]; memcpy_P(w, genWaveNames[constrain(param1, 1, 5) - 1], 4);
      display->print(w);
    } break;
  }
  barSlotEnd(2, 36);

  // Slot 3: Param2
  if (mode == MODE_GEN) {
    barSlotBegin(3, 66);
    if (param1 == 5) {
      char v[6]; fmtDec1(v, param2);
      display->print(v); display->print('V');
    } else {
      uint16_t fX10 = pgm_read_word(&genFreqTableX10[constrain(param2, 1, GEN_NUM_FREQS) - 1]);
      char f[10]; fmtFreq(f, fX10);
      display->print(f);
    }
    barSlotEnd(3, 66);
  } else if (mode != MODE_TUNER) {
    barSlotBegin(3, 78);
    display->print(mode == MODE_LFO ? F("O:") : F("R:"));
    display->print(param2);
    barSlotEnd(3, 78);
  }

  // Restore transparent white text, otherwise the next frame of the mode
  // screen inherits the inverted colors of an edited slot 3
  display->setTextColor(WHITE);
}

// ================== Config Menu ==================
void configMenu() {
  int newDir = encoderDirection;
  newPosition = encoder->read();
  const uint8_t NC = 5;

  if (newPosition > oldPosition + 3) {
    oldPosition = newPosition;
    switch (configMenuOption) {
      case 1: newDir = 1; break;
      case 2: if (menuTimer > 1) menuTimer--; break;
      case 3: oledRotation = 0; display->setRotation(0); break;
      case 4: if (calOffset > -50) calOffset--; break;
      case 5: if (calGain > 32) calGain--; break;
    }
  } else if (newPosition < oldPosition - 3) {
    oldPosition = newPosition;
    switch (configMenuOption) {
      case 1: newDir = -1; break;
      case 2: if (menuTimer < 60) menuTimer++; break;
      case 3: oledRotation = 2; display->setRotation(2); break;
      case 4: if (calOffset < 50) calOffset++; break;
      case 5: if (calGain < 255) calGain++; break;
    }
  }
  if (newDir != encoderDirection) encoderDirection = newDir;
  if (old_SW == 0 && SW == 1) configMenuOption = (configMenuOption % NC) + 1;

  static unsigned long holdStart = 0;
  if (SW && !old_SW) holdStart = millis();
  if (SW && holdStart > 0 && (millis() - holdStart >= 2000)) {
    EEPROM.put(ENCODER_DIR_ADDR, encoderDirection);
    EEPROM.write(OLED_ROT_ADDR, oledRotation);
    EEPROM.put(MENUTIMER_DIR_ADDR, (uint8_t)menuTimer);
    saveCurrentModeToRAM();
    saveAllSettings();
    display->clearDisplay();
    display->setTextColor(WHITE);
    display->setCursor(16, 25);
    display->print(F("SETTINGS SAVED"));
    display->display();
    delay(800);
    configMenuActive = false; holdStart = 0;
    oldPosition = newPosition = encoder->read() * encoderDirection;
    return;
  }
  if (!SW) holdStart = 0;

  display->clearDisplay();
  display->setCursor(0, 0);
  display->setTextColor(WHITE);
  display->println(F("SETTINGS"));

  display->setTextColor(configMenuOption == 1 ? BLACK : WHITE, configMenuOption == 1 ? WHITE : BLACK);
  display->print(F("Enc: "));
  display->setTextColor(WHITE);
  display->println(encoderDirection == 1 ? F("Norm") : F("Rev"));

  display->setTextColor(configMenuOption == 2 ? BLACK : WHITE, configMenuOption == 2 ? WHITE : BLACK);
  display->print(F("Timer: "));
  display->setTextColor(WHITE);
  display->print(menuTimer); display->println('s');

  display->setTextColor(configMenuOption == 3 ? BLACK : WHITE, configMenuOption == 3 ? WHITE : BLACK);
  display->print(F("OLED: "));
  display->setTextColor(WHITE);
  display->println(oledRotation == 0 ? F("0") : F("180"));

  display->setTextColor(configMenuOption == 4 ? BLACK : WHITE, configMenuOption == 4 ? WHITE : BLACK);
  display->print(F("DAC Ofs: "));
  display->setTextColor(WHITE);
  if (calOffset >= 0) display->print('+');
  display->println(calOffset);

  display->setTextColor(configMenuOption == 5 ? BLACK : WHITE, configMenuOption == 5 ? WHITE : BLACK);
  display->print(F("DAC Gn: "));
  display->setTextColor(WHITE);
  uint16_t gainPct = ((uint16_t)calGain * 100) / 128;
  display->print(gainPct); display->println('%');

  display->setCursor(4, 56);
  display->print(F("Hold 2s "));
  display->print(isHWv25 ? F("v2.5") : F("v2"));
  display->print(isLGT8F ? F(" LGT") : F(" 328"));
  display->display();
}
