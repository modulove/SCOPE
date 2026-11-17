#include <Arduino.h>
#include "hardware.h"
#include "constants.h"
#include "input.h"

#if IS_LGT8F && !defined(HOST)
// The lgt8fx E2PROM library cost about 230 bytes of flash for these few
// accesses. The same register sequences as its lgt_eeprom_init(1) /
// read_byte / write_byte: 1 KB emulation, where the address the sketch uses is
// the real address (below 1020).
static void eeInit() { ECCR = 0x80; ECCR = 0x4C; }
static uint8_t eeRead(uint16_t a) {
  EEARL = a;
  EEARH = a >> 8;
  EECR |= _BV(EERE);
  asm volatile ("nop\n\tnop");
  return EEDR;
}
static void eeUpdate(uint16_t a, uint8_t v) {
  if (eeRead(a) == v) return;          // EEAR stays set
  EEDR = v;
  uint8_t s = SREG;
  cli();
  EECR = 0x04;
  EECR = 0x02;
  SREG = s;
}
#else
#include <EEPROM.h>
static void eeInit() {}
#define eeRead(a)      EEPROM.read(a)
#define eeUpdate(a, v) EEPROM.update(a, v)
#endif

// Same byte and meaning as the SCOPE firmware (SCOPEv2.ino, v3.8):
// bit0 = v2.5 display wiring, bit1 = v2.5 encoder wiring, >3 = autodetect.
#define EEPROM_HWPROFILE_ADDR  20
#define EEPROM_DOOM_MAGIC_ADDR 1000
#define EEPROM_DOOM_SOUND_ADDR 1001
#define EEPROM_DOOM_STEER_ADDR 1002
#define EEPROM_DOOM_MAGIC      0xD0

#define PIN_IDENT A7

#ifdef HOST
  #define BUTTON_DOWN() (digitalRead(PIN_BUTTON) == LOW)
#else
  #define BUTTON_DOWN() (!(PIND & _BV(5)))   // D5 = PD5
#endif

bool    hwDisplayV25 = true;
bool    hwEncoderV25 = true;
uint8_t hwProfile = 0xFF;
bool    hwHasDac = false;
// Sound starts OFF until the player turns it on in SETUP (and START stores
// it): with the DAC solder jumper still on its default side, driving D4 resets
// the module at random (SCOPE README). Off = the DAC is never enabled.
uint8_t setSound = 0;
int8_t  setSteer = 1;

void hardware_detect() {
  // The scope front-end's offset PWM: a plain low output (what SCOPE's LFO mode
  // sets). Driving it as a digital pin keeps Timer2 free for the sound engine.
  // Port writes instead of pinMode/digitalWrite (saves their code):
  // D3 offset low output, D5 button with pull-up, D7 TRIG plain input.
#ifndef HOST
  PORTD &= ~(_BV(3) | _BV(7));
  DDRD  |= _BV(3);
  DDRD  &= ~(_BV(5) | _BV(7));
  PORTD |= _BV(5);
#endif

  // Hardware profile, exactly as in the SCOPE firmware. Holding the encoder
  // during power-up steps to the next profile and stores it - the way out when
  // the wrong display pins leave the screen dark. This byte is SCOPE's own: an
  // upload does not erase the LGT8F's emulated EEPROM (measured; the bootloader
  // only erases the flash pages it writes), so DOOM starts on the profile the
  // scope firmware used, and SCOPE gets back whatever is chosen here.
  eeInit();
  delayMicroseconds(200);
  hwProfile = eeRead(EEPROM_HWPROFILE_ADDR);
  if (BUTTON_DOWN()) {
    wait_ms(600);                                   // ignore a brushed encoder
    if (BUTTON_DOWN()) {
      hwProfile = (hwProfile > 3) ? 3 : ((hwProfile + 1) & 3);
      eeUpdate(EEPROM_HWPROFILE_ADDR, hwProfile);
      while (BUTTON_DOWN()) {}                     // the release must not fire
      wait_ms(50);
    }
  }
  if (hwProfile <= 3) {
    hwDisplayV25 = hwProfile & 1;
    hwEncoderV25 = (hwProfile & 2) != 0;
  } else {
    hwDisplayV25 = hwEncoderV25 = (analogRead(PIN_IDENT) < 100);
  }

  // CV OUT is the LGT8F's own DAC on D4. A solder jumper decides what D4
  // carries; when the encoder is wired there (v2 encoder profile) the DAC must
  // stay off or it fights the encoder contact and browns the board out.
  hwHasDac = IS_LGT8F && hwEncoderV25;

  if (eeRead(EEPROM_DOOM_MAGIC_ADDR) == EEPROM_DOOM_MAGIC) {
    setSound = eeRead(EEPROM_DOOM_SOUND_ADDR);
    if (setSound > 2) setSound = 2;
    setSteer = (eeRead(EEPROM_DOOM_STEER_ADDR) == 0xFF) ? -1 : 1;
  }
  if (!hwHasDac) setSound = 0;
}

void settings_save() {
  eeUpdate(EEPROM_DOOM_MAGIC_ADDR, EEPROM_DOOM_MAGIC);
  if (hwHasDac) eeUpdate(EEPROM_DOOM_SOUND_ADDR, setSound);  // keep the choice for a DAC profile
  eeUpdate(EEPROM_DOOM_STEER_ADDR, (uint8_t)setSteer);
}
