#ifndef SCOPE_DOOM_HARDWARE_H
#define SCOPE_DOOM_HARDWARE_H

#include <stdint.h>

#if defined(__LGT8F__) || defined(__LGT8FX8P__) || defined(LARDUINO_HSP)
  #define IS_LGT8F 1
#else
  #define IS_LGT8F 0
#endif

extern bool    hwDisplayV25;   // OLED on the hardware SPI pins (v2.5)
extern bool    hwEncoderV25;   // encoder on A2/A3 (v2.5) instead of D2/D4 (v2)
extern uint8_t hwProfile;      // 0..3 forced profile, 0xFF autodetect (same byte as SCOPE)
extern bool    hwHasDac;       // LGT8F with D4 free: CV OUT can carry sound

// Doom settings, kept apart from the SCOPE firmware's EEPROM bytes (0..52, at
// 1000..1002) so a board that runs both keeps its scope settings: uploads do
// not erase the EEPROM, on the LGT8F either.
extern uint8_t setSound;       // 0 off, 1 effects, 2 effects + music
extern int8_t  setSteer;       // +1 normal, -1 reversed

void hardware_detect();        // before anything else in setup()
void settings_save();

#endif
