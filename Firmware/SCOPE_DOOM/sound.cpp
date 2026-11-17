#include <Arduino.h>
#include "sound.h"
#include "input.h"
#include "hardware.h"

// ================== Voices ==================
// 16-bit phase accumulators at SOUND_RATE. A voice is silent when vol == 0.
// Noise is a 16-bit LFSR clocked each time the phase wraps, so its "pitch" is
// the clock rate: low = growl/rumble, high = hiss.
#define W_OFF   0      // silent (used as a timer layer)
#define W_SQR   1
#define W_SAW   2
#define W_NOISE 4
#define SQ(d)   (W_SQR | ((d) << 4))     // square, duty d/16 (0 = 50 %)

struct Voice { uint16_t ph, inc; uint8_t vol, wave, duty; int8_t nz; };
struct Env   { uint8_t hold, decay; int8_t slide; };

// 0,1 = effect layers, 2 = bass, 3 = fifth, 4 = drums
static Voice V[5];
static Env   E[5];
static uint16_t lfsr = 0xACE1;

// ================== Effects ==================
// pitch byte p -> phase increment (16 + (p & 15)) << (p >> 4): a 4-bit float,
// about a semitone per step. 0x40 = 62 Hz, 0x80 = 1 kHz, 0xB0+ = noise hiss.
// slide: increment changes by slide/256 per control tick (4 ms).
// vol 0..255 is held for 'hold' ticks, then falls by 'decay' per tick.
struct SfxLayer { uint8_t wave, pitch; int8_t slide; uint8_t vol, hold, decay; };
struct SfxDef   { SfxLayer a, b; uint8_t next, prio; };

#define NOL { W_OFF, 0, 0, 0, 0, 0 }

static const SfxDef SFX[SFX_COUNT - 1] PROGMEM = {
  // SFX_SHOTGUN: bright noise blast darkening + a falling low thump
  { { W_NOISE, 0xBC,  -5, 255,  3,  5 }, { SQ(0),   0x48, -10, 230,  2,  9 }, 0, 3 },
  // SFX_PUMP1: rack back - "shk"
  { { W_NOISE, 0xA6,  -4, 170,  3, 14 }, { SQ(4),   0x86, -12, 110,  1, 16 }, 0, 2 },
  // SFX_PUMP2: rack forward - "chk", lower and louder
  { { W_NOISE, 0x9C,  -4, 200,  4, 12 }, { SQ(4),   0x74, -10, 150,  2, 14 }, 0, 2 },
  // SFX_PUNCH: fist swing
  { { W_NOISE, 0x84,   6, 140,  2,  9 }, NOL,                                0, 2 },
  // SFX_PUNCH_HIT
  { { W_NOISE, 0x7A,  -4, 230,  1, 12 }, { SQ(0),   0x40,  -9, 220,  1, 11 }, 0, 2 },
  // SFX_ENEMY_DIE: falling screech + crunch
  { { W_SAW,   0x72,  -3, 190,  8,  3 }, { W_NOISE, 0x95,  -3, 130,  8,  3 }, 0, 2 },
  // SFX_ALERT: imp growl
  { { W_NOISE, 0x63,   0, 120, 20,  5 }, { SQ(3),   0x54,   2,  70, 20,  4 }, 0, 1 },
  // SFX_FIREBALL: rising whoosh
  { { W_NOISE, 0x88,   4, 150,  6,  4 }, NOL,                                0, 1 },
  // SFX_HURT: low buzz
  { { SQ(4),   0x4C,  -2, 230, 10,  8 }, { W_SAW,   0x55,  -2, 150, 10,  7 }, 0, 3 },
  // SFX_MEDKIT, 2, 3: rising major arpeggio
  { { SQ(0),   0x71,   0, 150, 10, 22 }, { SQ(4),   0x81,   0,  60, 10, 10 }, SFX_MEDKIT2, 2 },
  { { SQ(0),   0x75,   0, 150, 10, 22 }, { SQ(4),   0x85,   0,  60, 10, 10 }, SFX_MEDKIT3, 2 },
  { { SQ(0),   0x79,   0, 150, 16, 10 }, { SQ(4),   0x89,   0,  60, 16,  6 }, 0, 2 },
  // SFX_SHELLS, 2: click + "ching"
  { { SQ(0),   0x86,   0, 140,  4, 30 }, { W_NOISE, 0xA4,   0, 110,  0, 45 }, SFX_SHELLS2, 2 },
  { { SQ(0),   0x8B,   0, 140,  8, 14 }, NOL,                                0, 2 },
  // SFX_SECRET: rising chord
  { { W_SAW,   0x60,   2, 150, 40,  3 }, { SQ(0),   0x70,   2,  80, 40,  3 }, 0, 2 },
  // SFX_STEP: soft thud
  { { W_NOISE, 0x56,  -4,  90,  0, 18 }, { SQ(0),   0x3C,  -6,  80,  0, 14 }, 0, 0 },
  // SFX_MENU: blip
  { { SQ(0),   0x84,   0, 110,  1, 28 }, NOL,                                0, 1 },
  // SFX_CONFIRM, 2
  { { SQ(0),   0x80,   0, 140,  8, 30 }, NOL,                                SFX_CONFIRM2, 2 },
  { { SQ(0),   0x88,   0, 140, 14, 12 }, NOL,                                0, 2 },
  // SFX_DEATH: long fall
  { { W_SAW,   0x5C,  -1, 220, 30,  2 }, { W_NOISE, 0x70,  -1, 140, 30,  2 }, 0, 3 },
  // SFX_WIN, 2, 3: fanfare
  { { SQ(0),   0x71,   0, 160, 25, 12 }, { SQ(4),   0x61,   0, 100, 25, 12 }, SFX_WIN2, 3 },
  { { SQ(0),   0x79,   0, 160, 25, 12 }, { SQ(4),   0x69,   0, 100, 25, 12 }, SFX_WIN3, 3 },
  { { SQ(0),   0x81,   0, 170, 70,  3 }, { SQ(4),   0x71,   0, 110, 70,  3 }, 0, 3 },
  // SFX_SPEED: tick when the walking speed changes
  { { SQ(0),   0x90,   0,  70,  0, 50 }, NOL,                                0, 1 },
};

// ================== Music ==================
// 16 steps per bar. Step byte: MIDI note (>= 12) = strike, 1 = choke, 0 = let ring.
// The fifth voice doubles the bass a fifth up: a power chord.
// All original - a palm-muted E Phrygian riff, not a transcription.
static const uint8_t PATTERNS[][16] PROGMEM = {
  { 40,40, 0,40,  0,40,43, 0, 40,40, 0,40,  0,46,45,43 },   // 0 E riff, ends Bb A G
  { 40,40, 0,40,  0,40,43, 0, 40,40, 0,40,  0,50,49,47 },   // 1 E riff, ends D C# B
  { 45,45, 0,45,  0,45,48, 0, 45,45, 0,45,  0,51,50,48 },   // 2 A riff, ends Eb D C
  { 47,47, 0,47,  0,47,50, 0, 42,42, 0,43,  0,45,46,47 },   // 3 B, then F# G A Bb B climb
  { 40, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0 },   // 4 title: one E hit
  { 41, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0, 40, 0, 1, 0 },   // 5 title: F ... E
  { 40, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0, 38, 0,39, 0 },   // 6 title: E ... D Eb
};

struct SongDef {
  uint8_t  stepTicks;          // control ticks (4 ms) per 16th note
  uint16_t decay;              // bass volume fall per tick, 8.8 fixed point
  uint16_t kick, snare, hat;   // drum step masks, bit n = step n
  uint8_t  len;
  uint8_t  order[8];
};

static const SongDef SONGS[SONG_COUNT - 1] PROGMEM = {
  { 30,   56, 0x0001, 0x0000, 0x0000, 4, { 4, 6, 5, 6 } },                 // TITLE  ~125 BPM, ringing
  { 25, 1126, 0x0909, 0x1010, 0x4444, 8, { 0, 1, 0, 2, 0, 1, 2, 3 } },     // LEVEL  150 BPM chug
  { 21, 1300, 0x5555, 0x1010, 0x0000, 4, { 0, 2, 1, 3 } },                 // BOSS   ~180 BPM, double kick
};

// Phase increments for MIDI 120..131 (C9..B9) at 16 kHz; lower octaves shift right.
static const uint16_t NOTE_TOP[12] PROGMEM = {
  34292, 36331, 38491, 40780, 43205, 45774, 48496, 51380, 54435, 57672, 61101, 64734
};

#define BASS_VOL   110

// Drum voice presets: kick (falling square), snare (noise), hat (short hiss)
static const SfxLayer DRUMS[3] PROGMEM = {
  { SQ(0),   0x58, -14, 200, 0, 14 },
  { W_NOISE, 0xB4,  -2, 140, 1, 10 },
  { W_NOISE, 0xBF,   0,  50, 0, 25 },
};

// ================== State ==================
static bool     dacOn = false;           // hardware allows the DAC (set once)
static volatile bool dacEnabled = false; // DAC switched on (only while sound is on)

// The DAC is switched on only when sound is turned on, and off again once the
// output has glided back to 0 V - while sound is off D4 is left alone exactly
// as the SCOPE firmware leaves it outside its GEN/RND/CLK/REF modes.
static void dacPower(bool on) {
  dacEnabled = on;
#if !defined(HOST) && IS_LGT8F
  // on:  DAVS = 00 (VCC reference), DACEN + DAOE (DAC on, output to PD4) -
  //      what SCOPE's analogReference(DEFAULT) + pinMode(DAC0, ANALOG) do.
  // off: what the lgt8fx core's pinMode(DAC0, INPUT) does (SCOPE's dacStop()).
  if (on) { DAL0 = 0; DACON = 0x0C; }
  else    { DAL0 = 0; DACON = 0x03; }
#endif
}
static volatile uint8_t mode = 0;        // 0 off, 1 fx, 2 fx + music
static volatile bool muted = false;
static uint8_t  dcLevel = 0;             // output centre, ramps 0 <-> 128
static uint8_t  nextOut = 0;
static uint8_t  isrDiv = 0;

static volatile uint8_t sfxNext = 0, sfxPrio = 0;

static volatile uint8_t song = 0;
static uint8_t  stepTimer = 1, stepIdx = 0, orderPos = 0;
static uint16_t stepBit = 1;             // 1 << stepIdx
static uint16_t bassVol16 = 0;
static SongDef  cur;                     // the playing song, copied out of flash

static uint16_t pitchInc(uint8_t p) {
  uint8_t e = p >> 4;
  if (e >= 12) return 0xFFFF;
  return (uint16_t)(16 + (p & 15)) << e;
}

static uint16_t noteInc(uint8_t n) {
  uint8_t o = n / 12;
  uint16_t inc = pgm_read_word(&NOTE_TOP[n - o * 12]);
  for (uint8_t s = o; s < 10; s++) inc >>= 1;
  return inc;
}

static void loadLayer(uint8_t i, const SfxLayer *src) {
  SfxLayer l;
  memcpy_P(&l, src, sizeof(l));
  Voice &v = V[i];
  Env &e = E[i];
  v.wave = l.wave & 7;
  uint8_t d = l.wave >> 4;
  v.duty = d ? d << 4 : 128;
  v.inc = pitchInc(l.pitch);
  v.vol = l.vol;
  e.slide = l.slide; e.hold = l.hold; e.decay = l.decay;
}

static void startSfx(uint8_t id) {            // with interrupts off or from the ISR
  const SfxDef *d = &SFX[id - 1];
  loadLayer(0, &d->a);
  loadLayer(1, &d->b);
  sfxNext = pgm_read_byte(&d->next);
  sfxPrio = pgm_read_byte(&d->prio);
}

static inline bool sfxActive() { return V[0].vol || V[1].vol || sfxNext; }

static void envTick(uint8_t i) {
  Voice &v = V[i];
  Env &e = E[i];
  if (!v.vol) return;
  if (e.hold) e.hold--;
  else v.vol = v.vol > e.decay ? v.vol - e.decay : 0;
  if (e.slide) {
    uint8_t s = e.slide < 0 ? -e.slide : e.slide;
    uint16_t d = (uint16_t)((uint8_t)(v.inc >> 8) * s);   // ~ inc * s / 256 as 8x8 multiply
    if (!d) d = 1;
    if (e.slide > 0) v.inc = (v.inc > 0xFFFF - d) ? 0xFFFF : v.inc + d;
    else             v.inc = (v.inc > d + 8) ? v.inc - d : 8;
  }
}

static void loadSong(uint8_t s) {
  memcpy_P(&cur, &SONGS[s - 1], sizeof(cur));
  stepIdx = 0; orderPos = 0; stepTimer = 1; stepBit = 1;
  bassVol16 = 0;
}

static void musicStep() {
  uint8_t pat = cur.order[orderPos];
  uint8_t n = pgm_read_byte(&PATTERNS[pat][stepIdx]);
  if (n >= 12) {
    uint16_t inc = noteInc(n);
    V[2].inc = inc;   V[2].wave = W_SQR; V[2].duty = 112;
    V[3].inc = inc + (inc >> 1); V[3].wave = W_SAW;
    bassVol16 = (uint16_t)BASS_VOL << 8;
  } else if (n == 1) {
    bassVol16 = 0;
  }
  uint8_t drum = (cur.kick & stepBit) ? 0 : (cur.snare & stepBit) ? 1 : (cur.hat & stepBit) ? 2 : 3;
  if (drum < 3) loadLayer(4, &DRUMS[drum]);
  stepBit <<= 1;
  if (++stepIdx >= 16) {
    stepIdx = 0;
    stepBit = 1;
    if (++orderPos >= cur.len) orderPos = 0;
  }
}

// Runs every 64 samples (4 ms).
static void controlTick() {
  uint8_t target = mode ? 128 : 0;
  if (!mode && !dcLevel && dacEnabled) dacPower(false);
  if (dcLevel < target) dcLevel++;          // 0.5 s glide: no thump on the jack
  else if (dcLevel > target) dcLevel--;
  if (muted || !mode) return;

  envTick(0);
  envTick(1);
  if (!V[0].vol && !V[1].vol && sfxNext) startSfx(sfxNext);

  if (song && mode == 2) {
    if (--stepTimer == 0) { stepTimer = cur.stepTicks; musicStep(); }
    bassVol16 = bassVol16 > cur.decay ? bassVol16 - cur.decay : 0;
    uint8_t bv = bassVol16 >> 8;
    V[2].vol = bv;
    V[3].vol = bv - (bv >> 2);
    envTick(4);
  } else {
    V[2].vol = V[3].vol = V[4].vol = 0;
  }
}

static inline int8_t voiceSample(Voice &v) {
  uint16_t p = v.ph + v.inc;
  bool wrapped = p < v.ph;
  v.ph = p;
  uint8_t h = p >> 8;
  switch (v.wave) {
    case W_SQR:   return h < v.duty ? 127 : -127;
    case W_SAW:   return (int8_t)(h ^ 0x80);
    case W_NOISE:
      if (wrapped) {
        lfsr = (lfsr >> 1) ^ (-(int16_t)(lfsr & 1) & 0xB400);
        v.nz = (int8_t)lfsr;
      }
      return v.nz;
  }
  return 0;
}

// One loop over the voices (unrolled, the five copies cost ~1 KB of flash).
static int16_t mixVoices() {
  int16_t fx = 0, mus = 0;
  Voice *v = V;
  for (uint8_t i = 0; i < 5; i++, v++) {
    if (!v->vol) continue;
    int16_t s = ((int16_t)voiceSample(*v) * v->vol) >> 8;
    if (i < 2) fx += s; else mus += s;
  }
  if (V[0].vol | V[1].vol) mus >>= 1;     // duck the music under effects
  return (fx + mus) >> 1;
}

static inline void soundInterrupt() {
#if !defined(HOST) && IS_LGT8F
  if (dacEnabled) DAL0 = nextOut;       // fixed latency: write first, compute next
#endif
  uint8_t d = ++isrDiv;
  if (!dacOn) {                         // input only: the timer runs at 4 kHz
    input_isr_encoder();
    if (!(d & 3)) input_isr_ms();
    return;
  }
  if (!(d & 3))  input_isr_encoder();   // 4 kHz
  if (!(d & 15)) input_isr_ms();        // 1 kHz
  if (!(d & 63)) controlTick();

  int16_t out = dcLevel;
  if (mode && !muted) {
    out += mixVoices();
    if (out < 0) out = 0;
    if (out > 255) out = 255;
  }
  nextOut = out;
}

#ifdef HOST
void sound_isr() { soundInterrupt(); }
uint8_t sound_dac() { return nextOut; }
#else
ISR(TIMER2_COMPA_vect) { soundInterrupt(); }
#endif

// ================== API ==================
void sound_setup(bool dacAvailable) {
  dacOn = dacAvailable;
#ifndef HOST
  // Timer2, CTC, /8: 32 MHz -> OCR 249, 16 MHz -> OCR 124 (16 kHz either way).
  // Without a DAC only the inputs need the timer: /32 with the same OCR = 4 kHz,
  // which saves most of the interrupt's load on a 16 MHz Nano.
  cli();
  TCCR2A = _BV(WGM21);
  TCCR2B = dacOn ? _BV(CS21) : (_BV(CS21) | _BV(CS20));
  OCR2A  = (uint8_t)(F_CPU / 8 / SOUND_RATE - 1);
  TCNT2  = 0;
  TIMSK2 = _BV(OCIE2A);
  sei();
#endif
}

void sound_stop_timer() {
#ifndef HOST
  TIMSK2 = 0;
  #if IS_LGT8F
  // Off, not just 0 V: after the soft reset (a jump, not a hardware reset) the
  // DAC must not keep driving D4 while sound is off.
  if (dacEnabled) dacPower(false);
  #endif
#endif
}

void sound_set_mode(uint8_t m) {
  if (!dacOn) m = 0;
  noInterrupts();
  if (m && !dacEnabled) dacPower(true);   // glides up from 0 V
  mode = m;
  if (!m) { V[0].vol = V[1].vol = 0; sfxNext = 0; }
  interrupts();
}

void sound_play(uint8_t id) {
#ifdef HOST
  host_log("sfx", id);
#endif
  if (!mode || !id || id >= SFX_COUNT) return;
  uint8_t prio = pgm_read_byte(&SFX[id - 1].prio);
  noInterrupts();
  if (!sfxActive() || prio >= sfxPrio) startSfx(id);
  interrupts();
}

void sound_music(uint8_t s) {
  if (s >= SONG_COUNT) s = SONG_NONE;
#ifdef HOST
  if (s != song) host_log("music", s);
#endif
  noInterrupts();
  if (s != song) {
    song = s;
    if (s) loadSong(s);
    V[2].vol = V[3].vol = V[4].vol = 0;
  }
  interrupts();
}

void sound_pause(bool p) { muted = p; }
