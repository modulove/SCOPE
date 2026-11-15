#include <Arduino.h>
// Polled mode: read() runs the library's quadrature state machine. It is called
// from the sound timer only, so the library never touches attachInterrupt()
// (the v2.5 pins A2/A3 have none) and never re-enables interrupts in the ISR.
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include "input.h"
#include "constants.h"

#ifdef HOST
  #define BUTTON_DOWN()  (digitalRead(PIN_BUTTON) == LOW)
  #define TRIG_HIGH()    (digitalRead(PIN_TRIGGER) == HIGH)
#else
  #define BUTTON_DOWN()  (!(PIND & _BV(5)))     // D5 = PD5 on Nano and LGT8F
  #define TRIG_HIGH()    (PIND & _BV(7))        // D7 = PD7, 100k pull-down
#endif

static Encoder *enc = nullptr;
static int32_t  encLast = 0;
static int8_t   encDir = 1;

// Shared with the ISR
static volatile int16_t freeCount = 0;    // counts, button up
static volatile int16_t heldCount = 0;    // counts, button held
static volatile uint8_t evFire = 0, evLong = 0, evTrig = 0;
static volatile uint16_t idleMs = 0;      // since the last turn or press

// ISR-only state
static bool     btnDown = false;          // debounced
static uint8_t  btnBounce = 0;
static uint16_t btnMs = 0;
static int8_t   btnTurn = 0;              // counts turned during this press (saturating)
static bool     btnUsed = false;          // turned or long-pressed: the release is no click
static bool     trigLast = false;

// Main-loop residuals (counts not yet worth a detent)
static int8_t menuRest = 0, throttleRest = 0;

void input_setup(bool encoderOnA2A3) {
  // A function-local static: constructed once, on the pins of the detected
  // profile, without pulling in the heap.
  static Encoder e(encoderOnA2A3 ? A2 : 2, encoderOnA2A3 ? A3 : 4);
  enc = &e;
  encLast = e.read();
  input_clear();
}

void input_set_direction(int8_t dir) { encDir = dir < 0 ? -1 : 1; }

void input_clear() {
  noInterrupts();
  freeCount = heldCount = 0;
  evFire = evLong = evTrig = 0;
  interrupts();
  menuRest = throttleRest = 0;
}

void input_isr_encoder() {
  if (!enc) return;
  int32_t p = enc->read();
  // On the SCOPE wiring the library counts down for a clockwise turn (the
  // SCOPE firmware maps a falling position to "increase"); flip it here so
  // clockwise is positive everywhere in the game.
  int8_t d = (int8_t)(encLast - p);
  if (!d) return;
  encLast = p;
  idleMs = 0;
  if (encDir < 0) d = -d;
  if (btnDown) {
    heldCount += d;
    int16_t t = btnTurn + d;
    btnTurn = t > 100 ? 100 : (t < -100 ? -100 : t);
    if (btnTurn >= 2 || btnTurn <= -2) btnUsed = true;   // a half detent: this press steers speed
  } else {
    int16_t f = freeCount + d;
    freeCount = f > 1000 ? 1000 : (f < -1000 ? -1000 : f);
  }
}

void input_isr_ms() {
  bool raw = BUTTON_DOWN();
  if (raw != btnDown) {
    if (++btnBounce >= 6) {            // stable for 6 ms
      btnBounce = 0;
      btnDown = raw;
      if (raw) {
        btnMs = 0; btnTurn = 0; btnUsed = false;
      } else if (!btnUsed) {
        evFire = 1;                    // a clean click
      }
    }
  } else {
    btnBounce = 0;
  }
  if (btnDown) {
    idleMs = 0;
    if (!btnUsed && ++btnMs >= LONG_PRESS_MS) {
      btnUsed = true;
      evLong = 1;
    }
  } else if (idleMs != 0xFFFF) {
    idleMs++;
  }

  bool t = TRIG_HIGH();
  if (t && !trigLast) evTrig = 1;
  trigLast = t;
}

static int16_t takeCount(volatile int16_t &c) {
  noInterrupts();
  int16_t v = c;
  c = 0;
  interrupts();
  return v;
}

int8_t input_turn() {
  INPUT_IDLE();
  int16_t v = takeCount(freeCount);
  return v > 127 ? 127 : (v < -127 ? -127 : v);
}

static int8_t detents(int16_t counts, int8_t &rest) {
  int16_t v = rest + counts;
  int8_t steps = v / COUNTS_PER_DETENT;
  rest = v - steps * COUNTS_PER_DETENT;
  return steps;
}

int8_t input_menu() {
  int8_t s = detents(input_turn(), menuRest);
  return s > 0 ? 1 : (s < 0 ? -1 : 0);
}

int8_t input_throttle() {
  INPUT_IDLE();
  return detents(takeCount(heldCount), throttleRest);
}

static bool takeFlag(volatile uint8_t &f) {
  INPUT_IDLE();
  if (!f) return false;
  f = 0;                               // single byte: atomic on AVR
  return true;
}

uint16_t input_idle_ms() {
  INPUT_IDLE();
  noInterrupts();
  uint16_t v = idleMs;
  interrupts();
  return v;
}

bool input_held() { INPUT_IDLE(); return *(volatile bool *)&btnDown; }   // the ISR changes it

void wait_ms(uint16_t ms) {
  uint16_t t0 = millis();                      // 16 bits are enough for the 1 s maximum
  while ((uint16_t)((uint16_t)millis() - t0) < ms) INPUT_IDLE();
}

bool input_fire() { return takeFlag(evFire); }
bool input_long() { return takeFlag(evLong); }
bool input_trig() { return takeFlag(evTrig); }
