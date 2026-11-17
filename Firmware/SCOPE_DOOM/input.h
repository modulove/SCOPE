#ifndef MODULOVE_SCOPE_DOOM_INPUT_H
#define MODULOVE_SCOPE_DOOM_INPUT_H

#include <stdint.h>

// One push encoder plus the TRIG jack. The encoder is read by the PJRC Encoder
// library; because the v2.5 encoder pins (A2/A3) have no external interrupt,
// the library's update is driven from the sound timer at 4 kHz, so no step is
// lost while a frame renders. Button and TRIG are sampled at 1 kHz.
//
//   turn                 look left / right (counts, proportional, accelerated)
//   hold + turn          walking speed: back / stop / walk / run / sprint
//   short click          fire (menus: select)
//   hold 0.7 s           pause + help (menus: ignored)
//   TRIG jack, rising    fire (gameplay only)

#define LONG_PRESS_MS   700
#define COUNTS_PER_DETENT 4

void   input_setup(bool encoderOnA2A3);
void   input_set_direction(int8_t dir);   // +1 normal, -1 reversed
void   input_clear();                     // drop pending events and counts

int8_t input_turn();       // counts turned with the button up since the last call (+ = clockwise)
int8_t input_menu();       // -1 / 0 / +1 per detent, button up
int8_t input_throttle();   // detents turned with the button held since the last call
bool   input_fire();       // a click: press and release without turning or holding long
bool   input_long();       // held LONG_PRESS_MS without turning (fires once per press)
bool   input_trig();       // rising edge on the TRIG jack since the last call

// Attract mode: milliseconds since the last turn or press, saturating at
// 65535. The TRIG jack does not count, so a patched clock cannot keep the demo
// from starting.
uint16_t input_idle_ms();
bool   input_held();       // the button is down (debounced)

void   wait_ms(uint16_t ms);   // delay() without micros(): 170 bytes less flash

// Called from the sound timer interrupt only.
void   input_isr_encoder();   // ~4 kHz
void   input_isr_ms();        // ~1 kHz

#ifdef HOST
void   host_idle();           // the PC test harness advances its clock here
void   host_reset();          // ... and ends the run where the board would reboot
void   host_state(double angle, double x, double y, int throttle, int health, int entities);
#define INPUT_IDLE() host_idle()
#else
#define INPUT_IDLE()
#endif

#endif
