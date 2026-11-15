#include <Arduino.h>
#include <EEPROM.h>
#include "hardware.h"
#include "constants.h"

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
  // the wrong display pins leave the screen dark. The LGT8F's emulated EEPROM is
  // erased by every upload, so after flashing it starts on autodetect again.
  delayMicroseconds(200);
  hwProfile = EEPROM.read(EEPROM_HWPROFILE_ADDR);
  if (BUTTON_DOWN()) {
    delay(600);                                    // ignore a brushed encoder
    if (BUTTON_DOWN()) {
      hwProfile = (hwProfile > 3) ? 3 : ((hwProfile + 1) & 3);
      EEPROM.update(EEPROM_HWPROFILE_ADDR, hwProfile);
      while (BUTTON_DOWN()) {}                     // the release must not fire
      delay(50);
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

  if (EEPROM.read(EEPROM_DOOM_MAGIC_ADDR) == EEPROM_DOOM_MAGIC) {
    setSound = EEPROM.read(EEPROM_DOOM_SOUND_ADDR);
    if (setSound > 2) setSound = 2;
    setSteer = (EEPROM.read(EEPROM_DOOM_STEER_ADDR) == 0xFF) ? -1 : 1;
  }
  if (!hwHasDac) setSound = 0;
}

void settings_save() {
  EEPROM.update(EEPROM_DOOM_MAGIC_ADDR, EEPROM_DOOM_MAGIC);
  if (hwHasDac) EEPROM.update(EEPROM_DOOM_SOUND_ADDR, setSound);  // keep the choice for a DAC profile
  EEPROM.update(EEPROM_DOOM_STEER_ADDR, (uint8_t)setSteer);
}
