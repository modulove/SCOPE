#ifndef _constants_h
#define _constants_h

#include <stdint.h>

// ================== SCOPE hardware ==================
// The wiring differs between hardware revisions; hardware.cpp picks it at run
// time exactly like the SCOPE firmware does (A7 ident pin, EEPROM byte 20,
// hold the encoder at power-up to step through the four profiles).
//
//   v2   (green PCB)      OLED bit-banged SPI: MOSI D9, CLK D10, DC D11, CS D12, RST D13
//                         encoder A/B on D2/D4
//   v2.5 (colourful PCB)  OLED hardware SPI:   MOSI D11, SCK D13, CS D10, DC D9, RST D8
//                         encoder A/B on A2/A3
//   both                  encoder switch D5, TRIG input D7, ident A7
//   LGT8F328P             8-bit DAC on D4 -> CV OUT (solder jumper on the LGT8F side)
#define PIN_BUTTON    5
#define PIN_TRIGGER   7
#define PIN_OFFSET    3     // input offset PWM of the scope front-end; held low here

// ================== Timing ==================
// The original build ran everything once per frame at 15 fps. Here the screen
// is drawn as fast as the hardware allows (capped at FRAME_MIN_MS) and the
// game logic (AI, timers, gun and reload animation) runs on a fixed TICK_MS
// tick, so enemies, ammo and difficulty behave the same at any frame rate.
#define FRAME_MIN_MS  33    // frame cap, ~30 fps
#define TICK_MS       67    // one game tick = one frame of the original 15 fps build
#define MAX_TICKS_PER_FRAME 3

// ================== Steering ==================
// Encoder counts (4 per detent) turn the view directly, like a mouse. Fast
// spins turn further per count (acceleration); a little easing spreads each
// step over two frames so single detents do not jump.
#define STEER_PER_COUNT  0.028   // rad per count (~6.4 deg per detent)
#define STEER_ACCEL      0.10    // extra gain per count in the same frame
#define THROTTLE_MIN     -1      // hold + turn sets the walking speed (cruise control)
#define THROTTLE_MAX      3

// Attract mode: the game plays itself after this long without a turn or press
#define DEMO_IDLE_MS     60000

// GFX settings
#define OPTIMIZE_SSD1306                // Optimizations for SSD1366 displays

#define RES_DIVIDER         2           // Higher values will result in lower horizontal resolution when rasterize and lower process and memory usage
                                        // Lower will require more process and memory, but looks nicer
#define Z_RES_DIVIDER       2           // Zbuffer resolution divider. We sacrifice resolution to save memory
#define DISTANCE_MULTIPLIER 20          // Distances are stored as uint8_t, multiplying the distance we can obtain more precision taking care
                                        // of keep numbers inside the type range. Max is 256 / MAX_RENDER_DEPTH
#define MAX_RENDER_DEPTH    12
#define MAX_SPRITE_DEPTH    8

#define ZBUFFER_SIZE        (SCREEN_WIDTH / Z_RES_DIVIDER)

// Level
#define LEVEL_WIDTH_BASE    6
#define LEVEL_WIDTH         (1 << LEVEL_WIDTH_BASE)
#define LEVEL_HEIGHT        57
#define LEVEL_SIZE          LEVEL_WIDTH / 2 * LEVEL_HEIGHT

// scenes
#define INTRO                 0
#define GAME_PLAY             1
#define DIFF                  2
#define MUS                   3     // now the SETUP screen (sound + steering)
#define MID                   4
#define SCORE                 5
// Game
#define GUN_TARGET_POS        18
#define GUN_SHOT_POS          GUN_TARGET_POS + 8

#define MOV_SPEED             .1
#define MOV_SPEED_INV         2.5           // 1 / MOV_SPEED

#define JOGGING_SPEED         .005
#define ENEMY_SPEED           .04
#define FIREBALL_SPEED        .2
#define FIREBALL_ANGLES       45          // Num of angles per PI

#define MAX_ENTITIES          12           // Max num of active entities

#define MAX_ENTITY_DISTANCE   200         // * DISTANCE_MULTIPLIER
#define MAX_ENEMY_VIEW        90          // * DISTANCE_MULTIPLIER
#define ITEM_COLLIDER_DIST    6           // * DISTANCE_MULTIPLIER
#define ENEMY_COLLIDER_DIST   4           // * DISTANCE_MULTIPLIER
#define FIREBALL_COLLIDER_DIST 3          // * DISTANCE_MULTIPLIER
#define ENEMY_MELEE_DIST      9           // * DISTANCE_MULTIPLIER
#define WALL_COLLIDER_DIST    .2

#define ENEMY_MELEE_DAMAGE    18
#define ENEMY_FIREBALL_DAMAGE 25
#define GUN_MAX_DAMAGE        80

// display
constexpr uint8_t SCREEN_WIDTH     =  128;
constexpr uint8_t SCREEN_HEIGHT    =  64;
constexpr uint8_t HALF_WIDTH       =  SCREEN_WIDTH/2;
constexpr uint8_t RENDER_HEIGHT    =  48;         // raycaster working height (the rest is the status bar; a multiple of 8)
constexpr uint8_t HALF_HEIGHT      =  SCREEN_HEIGHT/2;

#endif
