/**
 * SCOPE_DOOM.ino - Doom Nano Brutality for the Modulove SCOPE (v2 / v2.5)
 *
 * Based on the user-submitted "Modulove SCOPE Doom" adaptation of Doom Nano
 * Brutality (itself a fork of daveruiz/doom-nano). Game logic, levels and
 * sprites are kept; this port adds:
 *
 *   - LGT8F328P target (32 MHz) with the SCOPE v2.5 wiring: OLED on hardware
 *     SPI, encoder on A2/A3. v2 boards (bit-banged OLED, encoder on D2/D4) are
 *     picked at run time with the same profile logic as the SCOPE firmware.
 *   - Sound effects and music on CV OUT from the LGT8F's own DAC on D4.
 *   - Encoder steering through the PJRC Encoder library, clocked from a timer
 *     so no step is lost while a frame renders: proportional, accelerated and
 *     eased turning; hold + turn is cruise control, so you can turn while you
 *     walk; click = fire; TRIG jack = fire.
 *   - The screen renders as fast as the hardware allows (30 fps cap) while the
 *     game logic keeps the original 15 Hz tick, so difficulty is unchanged.
 *   - Attract mode: after a minute without input the game plays itself.
 *   - Fixes, see FIXES in the README.
 *
 * Controls: turn = look, hold + turn = walking speed, click = fire,
 * hold 0.7 s = pause (with help; hold again in pause = back to title).
 * In the demo, any turn or press goes back to the title.
 */

#include <Arduino.h>
#include "constants.h"
#include "hardware.h"
#include "level.h"
#include "sprites.h"
#include "input.h"
#include "sound.h"
#include "entities.h"
#include "types.h"
#include "display.h"

void softReset();
uint8_t getBlockAt(const uint8_t level[], uint8_t x, uint8_t y);
Coords translateIntoView(Coords *pos);
void updateHud();

// Useful macros
#define SWAP(a, b)            do { typeof(a) temp = a; a = b; b = temp; } while (0)

// One enemy step towards the player on one axis (was sign(a, b) * ENEMY_SPEED).
// The sign is read from the float's bits: one subtraction instead of two
// float compares.
static double __attribute__((noinline)) toward(double target, double from) {
  float dlt = target - from;                     // float: double is 64-bit on the PC harness
  uint32_t u;
  memcpy(&u, &dlt, sizeof(u));
  if (!(u << 1)) return 0;                       // +0 or -0: same axis position
  return (u >> 31) ? -ENEMY_SPEED : ENEMY_SPEED;
}

void setup(void) {
  hardware_detect();                 // profile, settings (reads the button first)
  setupDisplay(hwDisplayV25);
  input_setup(hwEncoderV25);
  input_set_direction(setSteer);
  sound_setup(hwHasDac);             // timer: sound + encoder + button
  sound_set_mode(setSound);
  sound_music(SONG_TITLE);
}

uint8_t scene = INTRO;
bool exit_scene = false;
bool invert_screen = false;
uint8_t flash_screen = 0;
uint8_t z = 6;                       // HUD message, see updateHud()
uint8_t difficulty = 1;
uint8_t rc1 = 0;                     // gun sprite: 0 gun, 1/2 reload frames, 3 none
int16_t a = 0;                       // score count-up
uint8_t enemyCount2 = 0;             // boss arena kills
Player player;
Entity entity[MAX_ENTITIES];
uint8_t num_entities = 0;
uint8_t enemyCount = 0;              // level 1 kills
bool del = false;
bool levelID = false;                // false = E1M1, true = E1M2
bool fade_e = true;
uint8_t r = 0;                       // reload animation step
bool reload1 = false;
int16_t score;
uint8_t k;                           // shells left at the end of level 1 (score)
int8_t mid = 1;                      // story page
bool bss = false;                    // boss arena active
bool EnemySpawn = false;
uint8_t trapCount = 0;               // level 1 ambush (was folded into player.secret)

int8_t  throttle = 0;                // cruise control, THROTTLE_MIN..THROTTLE_MAX
uint8_t throttleMsg = 0;             // ticks left to show the speed name
double  steerPending = 0;            // rotation still to apply (easing)

static inline const uint8_t *curLevel() { return levelID ? E1M2 : E1M1; }

// xorshift16 for item drops (avr-libc's rand() costs 150 bytes of flash)
static uint16_t rngState = 0xACE1;
static uint8_t rnd() {
  rngState ^= rngState << 7;
  rngState ^= rngState >> 9;
  rngState ^= rngState << 8;
  return rngState;
}

// ================== Attract mode ==================
// After DEMO_IDLE_MS without a turn or a press, on any screen, the game plays
// itself: E1M1 and E1M2 in turn, a new demo after each death or level end, so
// an unused module always shows something (and the OLED does not burn in).
// Any turn or press goes back to the title. The demo writes no EEPROM.
bool demo = false;
static bool demoLevel = true;
static int16_t aiSeenX;              // screen x of the nearest living imp in view
static uint8_t aiSeenDist = 0;       // its distance (* DISTANCE_MULTIPLIER), 0 = none
static bool    aiFresh = false;      // a new frame was drawn since the last aim correction
static bool    aiHit = false;        // the demo player ran into something this tick
static uint8_t aiBump = 0;           // ticks in a row it did
static uint8_t aiBlind = 0;          // ticks to ignore imps after getting stuck

static void startDemo() {
  demo = true;
  levelID = demoLevel = !demoLevel;
  num_entities = enemyCount = enemyCount2 = trapCount = r = rc1 = flash_screen = 0;
  bss = EnemySpawn = del = reload1 = false;
  z = 6;
  difficulty = 1;
  fade_e = true;
  scene = GAME_PLAY;
  exit_scene = true;
#ifdef HOST
  host_log("demo start, level", levelID);
#endif
}

// Every waiting loop calls this.
static void demoIfIdle() {
  if (input_idle_ms() >= DEMO_IDLE_MS) startDemo();
}

// One game tick of the demo player. It aims with the screen position of the
// nearest imp (renderEntities records it), avoids walls with the z-buffer of
// the last frame, steers through steerPending (the encoder's easing) and sets
// the cruise speed. Returns true to fire.
static bool __attribute__((noinline)) aiTick() {
  int8_t rot = 0;                                   // in 0.01 rad (one float op at the end)
  int8_t thr = 3;                                   // explore at SPRINT
  bool fire = false;
  uint8_t d = aiSeenDist;
  aiBump = aiHit ? aiBump + 1 : 0;
  aiHit = false;
  if (aiBump > 6) { aiBlind = 30; aiBump = 0; }     // stuck: wander off for 2 s
  if (aiBlind) { aiBlind--; d = 0; }
  // Turn only on a freshly drawn frame: below 15 fps several ticks share one
  // frame, and correcting again from the same picture overshoots (at 5 fps the
  // aim never settled and the demo player never fired).
  bool fresh = aiFresh;
  aiFresh = false;
  if (d) {
    int16_t err = aiSeenX - HALF_WIDTH;             // + = right of the crosshair
    if (fresh) rot = constrain(-err, -25, 25);      // 0.01 rad per pixel
    bool gun = player.keys;                         // shotgun: close in to 2 units, back off
    uint8_t range = gun ? 40 : ENEMY_MELEE_DIST;    // inside 0.8; fist: run into melee range
    thr = d > range ? 2 : (gun && d < 16 ? -1 : 0);
    // Shotgun: fire when fire() would do real damage. Its damage is
    // 16 / (|x offset| * distance), x1.5 on the demo's skill; with the offset
    // err * depth / 64 and distance ~ 20 * depth, "at least 6" is
    // |err| * d * d < 3200: centred up to 5 units, 1 px off up to 2.8, 3 px up
    // to 1.5. Fist: in melee range and roughly in front.
    fire = gun ? d < 100 && (uint32_t)abs(err) * d * d < 3200 : d <= ENEMY_MELEE_DIST && abs(err) < 10;
  } else if (fresh && (zbuffer[ZBUFFER_SIZE / 2] < 26 || aiBump > 2)) {
    rot = zbuffer[2] > zbuffer[ZBUFFER_SIZE - 3] ? 30 : -30;   // turn to the open side
  } else if (fresh && rnd() < 6) {
    rot = (rnd() & 1) ? 80 : -80;                    // now and then a new heading
  }
  steerPending += rot * 0.01;
  throttle = thr;
  return fire;
}

// Jump to another scene
void jumpTo(uint8_t target_scene) {
  scene = target_scene;
  exit_scene = true;
  // Only reached after death: the game restarts from the title.
  if (target_scene == INTRO) softReset();
}

// Finds the player in the map
void initializeLevel(const uint8_t level[]) {
  // (The submission's uint8_t y >= 0 loop could never end.)
  for (int8_t y = LEVEL_HEIGHT - 1; y >= 0; y--) {
    for (uint8_t x = 0; x < LEVEL_WIDTH; x++) {
      if (getBlockAt(level, x, y) == E_PLAYER) {
        // what create_player() set: facing +x, 100 health, 10 shells
        memset(&player, 0, sizeof(player));
        player.pos.x = x + 0.5;
        player.pos.y = y + 0.5;
        player.dir.x = 1;
        player.plane.y = -0.66;
        player.health = 100;
        player.keys = 10;
        return;
      }
    }
  }
}

uint8_t getBlockAt(const uint8_t level[], uint8_t x, uint8_t y) {
  if (x >= LEVEL_WIDTH || y >= LEVEL_HEIGHT) {
    return E_FLOOR;
  }

  // y is read in inverse order
  return pgm_read_byte(level + (((LEVEL_HEIGHT - 1 - y) * LEVEL_WIDTH + x) / 2))
         >> (!(x % 2) * 4)       // displace part of wanted bits
         & 0b1111;               // mask wanted bits
}

bool isSpawned(UID uid) {
  for (uint8_t i = 0; i < num_entities; i++) {
    if (entity[i].uid == uid) return true;
  }
  return false;
}

void spawnEntity(uint8_t type, uint8_t x, uint8_t y) {
  // Limit the number of spawned entities
  if (num_entities >= MAX_ENTITIES) {
    return;
  }

  // enemies start with 100 health; keys (shells) and medkits with 0
  if (type == E_ENEMY || type == E_KEY || type == E_MEDIKIT) {
    entity[num_entities++] = create_entity(type, x, y, S_STAND, type == E_ENEMY ? 100 : 0);
  }
}

void spawnFireball(double x, double y) {
  // Limit the number of spawned entities
  if (num_entities >= MAX_ENTITIES) {
    return;
  }

  UID uid = create_uid(E_FIREBALL, x, y);
  // Remove if already exists, don't throw anything. Not the best, but shouldn't happen too often
  if (isSpawned(uid)) return;

  // Calculate direction. 32 angles
  int16_t dir = FIREBALL_ANGLES + atan2(y - player.pos.y, x - player.pos.x) * (FIREBALL_ANGLES / PI);
  if (dir < 0) dir += FIREBALL_ANGLES * 2;
  entity[num_entities] = create_fireball(x, y, dir);
  num_entities++;
  sound_play(SFX_FIREBALL);
}

// Remove by index. The boss arena spawns enemies on the same two cells, so
// several entities share a UID; removing "the first with this UID" (as the
// submission did) could delete a living enemy instead of the intended one.
void removeEntityAt(uint8_t i) {
  if (i >= num_entities) return;
  num_entities--;
  for (; i < num_entities; i++) entity[i] = entity[i + 1];
}

// Boss arena: free a slot for the next enemy by dropping a corpse.
void clearEntities() {
  for (uint8_t i = num_entities; i-- > 0; ) {
    if (uid_get_type(entity[i].uid) == E_ENEMY && entity[i].state == S_DEAD) {
      removeEntityAt(i);
      return;
    }
  }
}

UID detectCollision(const uint8_t level[], Coords *pos, double relative_x, double relative_y, bool only_walls = false) {
  // Wall collision
  uint8_t round_x = int(pos->x + relative_x);
  uint8_t round_y = int(pos->y + relative_y);
  uint8_t block = getBlockAt(level, round_x, round_y);

  if (block == E_WALL || block == E_COLL) {
    return create_uid(block, round_x, round_y);
  }

  // Secret doors look like walls but can be walked through. Each one counts
  // once, and only for the player (the submission set the same flag for all
  // three doors, and enemies walking into a door triggered the message too).
  if (pos == &player.pos) {
    uint8_t *flag = nullptr;
    if (block == E_DOOR)  flag = &player.secret;
    if (block == E_DOOR2) flag = &player.secret2;
    if (block == E_DOOR3) flag = &player.secret3;
    if (flag && !*flag) {
      *flag = 1;
      z = 8;
      sound_play(SFX_SECRET);
    }
  }

  if (only_walls) {
    return UID_null;
  }

  // Entity collision
  for (uint8_t i = 0; i < num_entities; i++) {
    // Don't collide with itself
    if (&(entity[i].pos) == pos) {
      continue;
    }

    uint8_t type = uid_get_type(entity[i].uid);

    // Only ALIVE enemy collision
    if (type != E_ENEMY || entity[i].state == S_DEAD || entity[i].state == S_HIDDEN) {
      continue;
    }

    Coords new_coords = { entity[i].pos.x - relative_x, entity[i].pos.y - relative_y };
    uint8_t distance = coords_distance(pos, &new_coords);

    // Check distance and if it's getting closer
    if (distance < ENEMY_COLLIDER_DIST && distance < entity[i].distance) {
      return entity[i].uid;
    }
  }

  return UID_null;
}

// Shoot. With shells: shotgun at every enemy in front, damage falling with
// distance and offset. Without: a punch at enemies in melee range.
// Damage per difficulty is the submission's (x1.5 / x1 / x0.7 gun,
// x1.25 / x0.9 / x0.6 fist, then divided by the difficulty), as percentages.
static const uint8_t GUN_PCT[3]   PROGMEM = { 150, 100, 70 };
static const uint8_t PUNCH_PCT[3] PROGMEM = { 125,  90, 60 };

void fire() {
  bool gun = player.keys != 0;
  bool hit = false;
  z = 3;
  for (uint8_t i = 0; i < num_entities; i++) {
    // Shoot only ALIVE enemies
    if (uid_get_type(entity[i].uid) != E_ENEMY || entity[i].state == S_DEAD || entity[i].state == S_HIDDEN) {
      continue;
    }
    if (!gun && entity[i].distance > ENEMY_MELEE_DIST) continue;

    Coords transform = translateIntoView(&(entity[i].pos));
    if (abs(transform.x) < 20 && transform.y > 0) {
      uint8_t damage = (double) min(GUN_MAX_DAMAGE, (GUN_MAX_DAMAGE / 5.0) / (abs(transform.x) * entity[i].distance));
      uint8_t pct = pgm_read_byte((gun ? GUN_PCT : PUNCH_PCT) + difficulty - 1);
      damage = ((uint16_t)damage * pct / 100) / difficulty;
      if (damage > 0) {
        entity[i].health = max(0, entity[i].health - damage);
        entity[i].state = S_HIT;
        entity[i].timer = 2;
        hit = true;
      }
    }
  }
  sound_play(gun ? SFX_SHOTGUN : (hit ? SFX_PUNCH_HIT : SFX_PUNCH));
}

// Update coords if possible. Return whether something blocked the move.
bool updatePosition(const uint8_t level[], Coords *pos, double relative_x, double relative_y, bool only_walls = false) {
  UID collide_x = detectCollision(level, pos, relative_x, 0, only_walls);
  UID collide_y = detectCollision(level, pos, 0, relative_y, only_walls);

  if (!collide_x) pos->x += relative_x;
  if (!collide_y) pos->y += relative_y;

  return collide_x || collide_y;
}

static void hurtPlayer(uint8_t amount) {
  player.health = max(0, player.health - amount * difficulty);
  flash_screen = 1;
  sound_play(SFX_HURT);
}

// One game tick (the original 15 fps frame).
void updateEntities(const uint8_t level[]) {
  uint8_t drop = (rnd() & 3) + 1;
  uint8_t i = 0;
  while (i < num_entities) {
    Entity &e = entity[i];
    // update distance
    e.distance = coords_distance(&(player.pos), &(e.pos));

    if (e.timer > 0) e.timer--;

    // too far away. put it in doze mode
    if (e.distance > MAX_ENTITY_DISTANCE) {
      removeEntityAt(i);
      continue;                 // the next entity moved into slot i
    }
    // bypass render if hidden
    if (e.state == S_HIDDEN) {
      i++;
      continue;
    }

    switch (uid_get_type(e.uid)) {
      case E_ENEMY: {
          // Enemy "IA"
          if (e.health == 0) {
            if (e.a == false) {
              // drops: 50 % shells, 25 % medkit, 25 % nothing
              if (drop <= 2)      spawnEntity(E_KEY, e.pos.x, e.pos.y);
              else if (drop == 3) spawnEntity(E_MEDIKIT, e.pos.x, e.pos.y);
              e.a = true;
              if (bss) { if (enemyCount2 < 8) enemyCount2++; EnemySpawn = true; }
              else if (enemyCount < 20) enemyCount++;   // one blast can kill several
              z = 7;
              sound_play(SFX_ENEMY_DIE);
            }
            if (e.state != S_DEAD) {
              e.state = S_DEAD;
              e.timer = 6;
            }
          } else if (e.state == S_HIT) {
            if (e.timer == 0) {
              // Back to alert state
              e.state = S_ALERT;
              e.timer = 40;     // delay next fireball thrown
            }
          } else if (e.state == S_FIRING) {
            if (e.timer == 0) {
              // Back to alert state
              e.state = S_ALERT;
              e.timer = 40;     // delay next fireball thrown
            }
          } else {
            // ALERT STATE
            if (e.distance > ENEMY_MELEE_DIST && e.distance < MAX_ENEMY_VIEW) {
              if (e.state != S_ALERT) {
                if (e.state == S_STAND) sound_play(SFX_ALERT);   // first sight
                e.state = S_ALERT;
                e.timer = 20;   // used to throw fireballs
              } else {
                if (e.timer == 0) {
                  // Throw a fireball
                  spawnFireball(e.pos.x, e.pos.y);
                  e.state = S_FIRING;
                  e.timer = 6;
                } else {
                  // move towards to the player (one tick = the original frame)
                  updatePosition(
                    level,
                    &(e.pos),
                    toward(player.pos.x, e.pos.x),
                    toward(player.pos.y, e.pos.y),
                    true
                  );
                }
              }
            } else if (e.distance <= ENEMY_MELEE_DIST) {
              if (e.state != S_MELEE) {
                // Preparing the melee attack
                e.state = S_MELEE;
                e.timer = 10;
              } else if (e.timer == 0) {
                // Melee attack
                hurtPlayer(ENEMY_MELEE_DAMAGE);
                e.timer = 14;
              }
            } else {
              // stand
              e.state = S_STAND;
            }
          }
          break;
        }

      case E_FIREBALL: {
          if (e.distance < FIREBALL_COLLIDER_DIST) {
            // Hit the player and disappear
            hurtPlayer(ENEMY_FIREBALL_DAMAGE);
            removeEntityAt(i);
            continue; // continue in the loop
          } else {
            // Move. Only collide with walls.
            // Note: using health to store the angle of the movement
            bool collided = updatePosition(
              level,
              &(e.pos),
              cos(e.health * (PI / FIREBALL_ANGLES)) * FIREBALL_SPEED,
              sin(e.health * (PI / FIREBALL_ANGLES)) * FIREBALL_SPEED,
              true
            );

            if (collided) {
              removeEntityAt(i);
              continue; // continue in the entity check loop
            }
          }
          break;
        }

      case E_MEDIKIT: {
          if (e.distance < ITEM_COLLIDER_DIST && player.health != 100) {
            // pickup
            e.state = S_HIDDEN;
            static const uint8_t HEAL[3] PROGMEM = { 65, 50, 25 };
            player.health = min(100, player.health + pgm_read_byte(HEAL + difficulty - 1));
            flash_screen = 1;
            z = 2;
            sound_play(SFX_MEDKIT);
          }
          break;
        }

      case E_KEY: {
          if (e.distance < ITEM_COLLIDER_DIST && player.keys < 240) {
            // pickup
            e.state = S_HIDDEN;
            static const uint8_t SHELLS[3] PROGMEM = { 13, 10, 9 };
            player.keys = min(240, player.keys + pgm_read_byte(SHELLS + difficulty - 1));
            z = 1;
            sound_play(SFX_SHELLS);
          }
          break;
        }
    }

    i++;
  }
}

// The map raycaster. Based on https://lodev.org/cgtutor/raycasting.html
void renderMap(const uint8_t level[], double view_height) {
  UID last_uid = 0;

  for (uint8_t x = 0; x < SCREEN_WIDTH; x += RES_DIVIDER) {
    double camera_x = 2 * (double) x / SCREEN_WIDTH - 1;
    double ray_x = player.dir.x + player.plane.x * camera_x;
    double ray_y = player.dir.y + player.plane.y * camera_x;
    uint8_t map_x = uint8_t(player.pos.x);
    uint8_t map_y = uint8_t(player.pos.y);
    Coords map_coords = { player.pos.x, player.pos.y };
    double delta_x = abs(1 / ray_x);
    double delta_y = abs(1 / ray_y);

    int8_t step_x;
    int8_t step_y;
    double side_x;
    double side_y;

    if (ray_x < 0) {
      step_x = -1;
      side_x = (player.pos.x - map_x) * delta_x;
    } else {
      step_x = 1;
      side_x = (map_x + 1.0 - player.pos.x) * delta_x;
    }

    if (ray_y < 0) {
      step_y = -1;
      side_y = (player.pos.y - map_y) * delta_y;
    } else {
      step_y = 1;
      side_y = (map_y + 1.0 - player.pos.y) * delta_y;
    }

    // Wall detection
    uint8_t depth = 0;
    bool hit = 0;
    bool side = 0;
    bool pillar = 0;
    while (!hit && depth < MAX_RENDER_DEPTH) {
      if (side_x < side_y) {
        side_x += delta_x;
        map_x += step_x;
        side = 0;
      } else {
        side_y += delta_y;
        map_y += step_y;
        side = 1;
      }

      uint8_t block = getBlockAt(level, map_x, map_y);

      if (block == E_WALL || block == E_DOOR || block == E_DOOR2 || block == E_DOOR3 || block == E_COLL) {
        hit = 1;
        if (block == E_COLL) pillar = 1;
      } else {
        // Spawning entities here, as soon they are visible for the
        // player. Not the best place, but would be a very performance
        // cost scan for them in another loop
        if (block == E_ENEMY || (block & 0b00001000) /* all collectable items */) {
          // Check that it's close to the player
          if (coords_distance(&(player.pos), &map_coords) < MAX_ENTITY_DISTANCE) {
            UID uid = create_uid(block, map_x, map_y);
            if (last_uid != uid && !isSpawned(uid)) {
              spawnEntity(block, map_x, map_y);
              last_uid = uid;
            }
          }
        }
      }

      depth++;
    }

    if (hit) {
      double distance;

      if (side == 0) {
        distance = max(1, (map_x - player.pos.x + (1 - step_x) / 2) / ray_x);
      } else {
        distance = max(1, (map_y - player.pos.y + (1 - step_y) / 2) / ray_y);
      }

      // store zbuffer value for the column
      zbuffer[x / Z_RES_DIVIDER] = min(distance * DISTANCE_MULTIPLIER, 255);

      // rendered line height
      uint8_t line_height = RENDER_HEIGHT / distance - 1;

      drawVLine(
        x,
        view_height / distance - line_height / 2 + RENDER_HEIGHT / 2 - (pillar ? 17 : 0),
        view_height / distance + line_height / 2 + RENDER_HEIGHT / 2,
        GRADIENT_COUNT - int(distance / MAX_RENDER_DEPTH * GRADIENT_COUNT) - side * 2
      );
    } else {
      // Nothing within the render depth: nothing hides sprites here. (The
      // submission kept the previous frame's value for such columns.)
      zbuffer[x / Z_RES_DIVIDER] = 255;
    }
  }
}

// Sort entities from far to close (insertion sort: at most 12 entries, and
// nearly sorted from the last frame)
void sortEntities() {
  for (uint8_t i = 1; i < num_entities; i++) {
    for (uint8_t j = i; j > 0 && entity[j - 1].distance < entity[j].distance; j--) {
      SWAP(entity[j], entity[j - 1]);
    }
  }
}

Coords translateIntoView(Coords *pos) {
  //translate sprite position to relative to camera
  double sprite_x = pos->x - player.pos.x;
  double sprite_y = pos->y - player.pos.y;

  // 1 / det(plane, dir). Rotation keeps the determinant at its start value
  // 0.66 (dir (1,0), plane (0,-0.66)), so it is a constant. The plane is
  // always 0.66 * (dir.y, -dir.x), so the depth (inv_det * (-plane.y * sx +
  // plane.x * sy)) is simply dir . s.
  const double inv_det = 1.0 / 0.66;
  double transform_x = inv_det * (player.dir.y * sprite_x - player.dir.x * sprite_y);
  double transform_y = player.dir.x * sprite_x + player.dir.y * sprite_y; // Z in screen

  return { transform_x, transform_y };
}

void renderEntities(double view_height) {
  sortEntities();
  aiSeenDist = 0;
  aiFresh = true;

  for (uint8_t i = 0; i < num_entities; i++) {
    if (entity[i].state == S_HIDDEN) continue;

    Coords transform = translateIntoView(&(entity[i].pos));

    // don´t render if behind the player or too far away
    if (transform.y <= 0.1 || transform.y > MAX_SPRITE_DEPTH) {
      continue;
    }

    int16_t sprite_screen_x = HALF_WIDTH * (1.0 + transform.x / transform.y);
    int8_t sprite_screen_y = RENDER_HEIGHT / 2 + view_height / transform.y;
    uint8_t type = uid_get_type(entity[i].uid);

    // don´t try to render if outside of screen
    if (sprite_screen_x < - HALF_WIDTH || sprite_screen_x > SCREEN_WIDTH + HALF_WIDTH) {
      continue;
    }
    // for the demo player: sorted far to near, so the last one is the nearest
    if (type == E_ENEMY && entity[i].health) {
      aiSeenX = sprite_screen_x;
      aiSeenDist = entity[i].distance;
    }

    // All sprites are square: imp 32 px, fireball and items 16 px.
    const uint8_t *bits = bmp_items_bits, *mask = bmp_items_mask;
    uint8_t size = BMP_ITEMS_WIDTH;
    int8_t yoff = 5;                         // items stand on the floor
    uint8_t sprite = (type == E_KEY);
    if (type == E_ENEMY) {
      bits = bmp_imp_bits; mask = bmp_imp_mask;
      size = BMP_IMP_WIDTH; yoff = -8;
      uint8_t st = entity[i].state;
      if (st == S_ALERT)       sprite = int(millis() / 500) % 2;          // walking
      else if (st == S_FIRING) sprite = 2;                                // fireball
      else if (st == S_HIT)    sprite = 3;                                // hit
      else if (st == S_MELEE)  sprite = entity[i].timer > 10 ? 2 : 1;     // melee attack
      else if (st == S_DEAD)   sprite = entity[i].timer > 0 ? 3 : 4;      // dying
      else                     sprite = 0;                                // stand
    } else if (type == E_FIREBALL) {
      bits = bmp_fireball_bits; mask = bmp_fireball_mask;
      size = BMP_FIREBALL_WIDTH; yoff = -8;
      sprite = 0;
    }
    drawSprite(
      sprite_screen_x - (size / 2) / transform.y,
      sprite_screen_y + yoff / transform.y,
      bits, mask, size, size, sprite, transform.y
    );
  }
}

// r1: 0 = shotgun, 1 / 2 = reload frames, 3 = nothing (dead)
void renderGun(uint8_t gun_pos, double amount_jogging, bool gun_fired, uint8_t r1) {
  if (r1 > 2) return;
  // jogging
  double t = (double) millis() * JOGGING_SPEED;
  char x = 48 + sin(t) * 10 * amount_jogging - 9;
  char y = RENDER_HEIGHT - gun_pos + abs(cos(t)) * 8 * amount_jogging - 3;
  if (gun_pos > GUN_SHOT_POS - 2 && player.keys > 0 && gun_fired) {
    display.drawBitmap(x + 14, y - 11, bmp_fire_bits, BMP_FIRE_WIDTH, BMP_FIRE_HEIGHT, 1);
  }
  const uint8_t *bits = bmp_gun_bits, *mask = bmp_gun_mask;
  uint8_t w = BMP_GUN_WIDTH, h = BMP_GUN_HEIGHT;
  int8_t ox = 0, oy = 0;
  if (r1) {                                  // reload frames sit 10 px left, 22 px up
    bits = r1 == 1 ? bmp_re1_bits : bmp_re2_bits;
    mask = r1 == 1 ? bmp_re1_mask : bmp_re2_mask;
    w = r1 == 1 ? BMP_RE1_WIDTH : BMP_RE2_WIDTH;
    h = BMP_RE1_HEIGHT;
    ox = -10; oy = -22;
  }
  // Don't draw over the hud
  int16_t clip_height = max(0, min(y + h, RENDER_HEIGHT) - y - oy);
  display.drawBitmap(x + ox, y + oy, mask, w, clip_height, 0);
  display.drawBitmap(x + ox, y + oy, bits, w, clip_height, 1);
}

// Render values for the HUD
void updateHud() {
  display.clearRect(0, 58, 128, 6);

  const __FlashStringHelper *msg = nullptr;
  uint8_t mx = 31;
  switch (z) {
    case 1: {
        static const uint8_t SHELLS_X[3] PROGMEM = { 57, 57, 60 };
        static const uint8_t SHELLS_N[3] PROGMEM = { 13, 10, 9 };
        drawText(31, 58, F("FOUND "));
        drawText(65, 58, F(" SHELLS"));
        drawText(pgm_read_byte(SHELLS_X + difficulty - 1), 58, (int16_t)pgm_read_byte(SHELLS_N + difficulty - 1));
      } break;
    case 2: msg = F("FOUND A MEDKIT"); break;
    case 4: mx = 38; msg = F("GAME OVER"); break;
    case 5: mx = 44; msg = F("YOU WIN"); break;
    case 6: mx = 33; msg = F("GOAL-20 KILLS"); break;
    case 7:
      if (levelID && bss) {
        drawText(37, 58, (int16_t)enemyCount2);
        mx = 52; msg = F("OUT OF 8");
      } else if (!levelID) {
        drawText(37, 58, (int16_t)enemyCount);
        mx = 52; msg = F("OUT OF 20");
      }
      break;
    case 8: mx = 35; msg = F("SECRET FOUND"); break;
    case 9: msg = F("GOAL-FIND EXIT"); break;
    default: break;              // 3 = no message
  }
  if (msg) drawText(mx, 58, msg);

  drawText(2, 58, F("{}"), 0);   // health symbol
  drawText(103, 58, F("[]"), 0); // shells symbol
  drawText(12, 58, (int16_t)player.health);
  drawText(113, 58, (int16_t)player.keys);
}

void softReset() {
  // Wait for the release: a button still held at the restart would reach the
  // power-up profile check in hardware_detect() (hold 0.6 s = next profile).
  while (input_held()) {}
  sound_stop_timer();                // also switches the DAC off
  noInterrupts();
#ifdef HOST
  host_reset();
#elif IS_LGT8F
  // The app's own reset vector. The LGT8F bootloader moves it to 0x18 and puts
  // a jump to itself at 0, where a soft reset would wait for its watchdog
  // (1-2 s of frozen screen) before the game starts again.
  asm volatile ("jmp 0x18");
#else
  asm volatile ("jmp 0");
#endif
}

void loopMid() {
  display.clearDisplay();
  if (mid == 1) {
    drawLines(1, 8, F("YEAR 2027. HUMANS REACHED\n"
                      "OTHER PLANETS, BUT WE ARE\n"
                      "NOT ALONE, THERE IS ALSO\n"
                      "HOSTILE ALIENS HERE. YOU\n"
                      "ARE UNKNOWN MARINE,\n"
                      "WHO FIGHT IN OLD LAB FOR\n"
                      "REMAINTS OF EARTH. RESIST\n"
                      "ALIENS TO ESCAPE."));
  }
  else if (mid == 2) {
    drawLines(1, 8, F("AFTER KILLING SOME\n"
                      "ALIENS, LIGHTS TURNED OFF\n"
                      "AND THE FLOOR COLLAPSED\n"
                      "AND YOU FELL INTO THE\n"
                      "UTILITY ROOMS. YOU HAVE\n"
                      "NO CHOICE BUT TO START\n"
                      "LOOKING FOR EXIT, WHILE\n"
                      "FIGHT ALIENS."));
    levelID = true;
  }
  else {
    drawLines(1, 8, F("AFTER HARD FIGHT YOU WENT\n"
                      "TO EXIT. AND AS SOON\n"
                      "YOU STEP OUT, ALIEN\n"
                      "ATTACKS YOU FROM BEHIND\n"
                      "AND KILLS YOU. YOU DIDNT\n"
                      "EXPECT THIS. YOUR FIGHT\n"
                      "CANT END LIKE THIS...\n"
                      "  THE END (MAYBE...)"));
  }
  drawText(46, 0, F("STORY"));
  drawText(37, 58, F("PRESS FIRE"));

  display.display();
  sound_music(SONG_TITLE);
  while (!exit_scene) {
    if (input_fire()) {
      sound_play(SFX_CONFIRM);
      fade_e = true;
      jumpTo(mid < 3 ? GAME_PLAY : SCORE);
    }
    demoIfIdle();
  }
}

void loopScore() {
  static uint8_t clicks = 0;
  score = player.keys / 2;
  score += player.health;
  score *= 43;
  score *= difficulty;
  // +69 per secret door found (the submission tested secret2/3 with "+= 0")
  score += 69 * (player.secret + player.secret2 + player.secret3);
  score += k;

  display.clearDisplay();

  display.drawBitmap(
    (SCREEN_WIDTH - BMP_LOGO_WIDTH) / 2 - 27,
    (SCREEN_HEIGHT - BMP_LOGO_HEIGHT) / 6,
    bmp_logo_bits,
    BMP_LOGO_WIDTH,
    BMP_LOGO_HEIGHT,
    1
  );
  drawText(SCREEN_WIDTH / 2.36 - 52, SCREEN_HEIGHT * .79, F("NANO BRUTALITY"));
  drawText(SCREEN_WIDTH / 0.99 - 45, SCREEN_HEIGHT * .2, F("YOU WIN"));
  drawText(SCREEN_WIDTH / 0.99 - 40, SCREEN_HEIGHT * .4, F("SCORE"));
  if (a < score) {
    a = min(score, a + 155);
    if (!(++clicks & 3)) sound_play(SFX_MENU);       // tally clicks
    if (a == score) sound_play(SFX_WIN);
  } else {
    a = score;
    drawText(SCREEN_WIDTH / 0.99 - 52, SCREEN_HEIGHT * .91, F("PRESS FIRE"));
    if (input_fire()) {
      display.clearDisplay();
      display.display();
      wait_ms(1000);
      softReset();
    }
  }
  drawText(SCREEN_WIDTH / 0.99 - 40, SCREEN_HEIGHT * .5, a);
  display.display();
  wait_ms(30);

  fade_e = false;
  jumpTo(SCORE);
  demoIfIdle();
}

// Intro screen
void loopIntro() {
  display.drawBitmap(
    (SCREEN_WIDTH - BMP_LOGO_WIDTH) / 2,
    (SCREEN_HEIGHT - BMP_LOGO_HEIGHT) / 6,
    bmp_logo_bits,
    BMP_LOGO_WIDTH,
    BMP_LOGO_HEIGHT,
    1
  );

  drawText(SCREEN_WIDTH / 4.6 - 25, SCREEN_HEIGHT * .3, F("PRESS"));
  drawText(SCREEN_WIDTH / 0.99 - 25, SCREEN_HEIGHT * .3, F("FIRE"));
  display.display();

  levelID = false;
  while (!exit_scene) {
    if (input_fire()) {
      rngState ^= millis() | 1;       // the moment of the click seeds the drops
      sound_play(SFX_CONFIRM);
      jumpTo(DIFF);
    }
    demoIfIdle();
  }
}

void loopDiff() {
  display.clearDisplay();

  drawText(SCREEN_WIDTH / 2.66 - 25, SCREEN_HEIGHT * .05, F("CHOOSE SKILL LEVEL"));

  drawText(SCREEN_WIDTH / 2.75 - 25, SCREEN_HEIGHT * .3, F("I"));
  drawText(SCREEN_WIDTH / 2.4 - 25, SCREEN_HEIGHT * .3, F("M TOO YOUNG TO DIE."));
  drawText(SCREEN_WIDTH / 2.6 - 25, SCREEN_HEIGHT * .26, F(","));

  drawText(SCREEN_WIDTH / 2.66 - 25, SCREEN_HEIGHT * .43, F("HURT ME PLENTY."));

  drawText(SCREEN_WIDTH / 2.66 - 25, SCREEN_HEIGHT * .56, F("NIGHTMARE."));

  drawText(SCREEN_WIDTH / 3.32 - 25, SCREEN_HEIGHT * .77, F("TURN TO CHOOSE,"));
  drawText(SCREEN_WIDTH / 4.1 - 25, SCREEN_HEIGHT * .9, F("CLICK TO CONFIRM"));

  static const uint8_t CURSOR_Y[3] PROGMEM = { 19, 27, 35 };   // .3 .43 .56 of 64
  drawText(SCREEN_WIDTH / 3 - 25, pgm_read_byte(CURSOR_Y + difficulty - 1), F("#"));

  display.display();
  while (!exit_scene) {
    int8_t m = input_menu();
    if (m) {
      difficulty = (difficulty + 2 + m) % 3 + 1;     // 1..3, wraps both ways
      sound_play(SFX_MENU);
      fade_e = false;
      jumpTo(DIFF);
    }
    else if (input_fire()) {
      sound_play(SFX_CONFIRM);
      fade_e = true;
      jumpTo(MUS);
    }
    demoIfIdle();
  }
}

// SETUP: sound output and steering direction. Turn = move, click = change.
void loopSetup() {
  static uint8_t row = 0;
  display.clearDisplay();
  drawText(52, 4, F("SETUP"));
  drawText(14, 20, F("SOUND"));
  drawText(14, 30, F("STEER"));
  drawText(14, 44, F("START"));
  if (!hwHasDac)          drawText(50, 20, F("NO DAC"));
  else if (setSound == 0) drawText(50, 20, F("OFF"));
  else if (setSound == 1) drawText(50, 20, F("EFFECTS"));
  else                    drawText(50, 20, F("FX AND MUSIC"));
  drawText(50, 30, setSteer > 0 ? F("NORMAL") : F("REVERSED"));
  static const uint8_t ROW_Y[3] PROGMEM = { 20, 30, 44 };
  drawText(6, pgm_read_byte(ROW_Y + row), F("#"));
  drawText(2, 56, F("CV OUT. JUMPER TO LGT DAC"));
  display.display();

  while (!exit_scene) {
    int8_t m = input_menu();
    if (m) {
      row = (row + 3 + m) % 3;
      sound_play(SFX_MENU);
      fade_e = false;
      jumpTo(MUS);
    } else if (input_fire()) {
      fade_e = false;
      if (row == 0 && hwHasDac) {
        setSound = (setSound + 1) % 3;
        sound_set_mode(setSound);
        sound_music(SONG_NONE);       // restart the title theme if music is on
        sound_music(SONG_TITLE);
      } else if (row == 1) {
        setSteer = -setSteer;
        input_set_direction(setSteer);
      } else if (row == 2) {
        settings_save();
        fade_e = true;
        sound_play(SFX_CONFIRM);
        jumpTo(MID);
        return;
      }
      sound_play(SFX_CONFIRM);
      jumpTo(MUS);
    }
    demoIfIdle();
  }
}

static void __attribute__((noinline)) rotateCoords(Coords *v, double c, double s) {
  double ox = v->x;
  v->x = ox * c - v->y * s;
  v->y = ox * s + v->y * c;
}

static void rotatePlayer(double rot) {
  double c = cos(rot), s = sin(rot);
  rotateCoords(&player.dir, c, s);
  rotateCoords(&player.plane, c, s);
}

// Pause with the controls; click resumes, holding again quits to the title.
static void pauseGame() {
  sound_pause(true);
  display.clearDisplay();
  drawText(49, 2, F("PAUSED"));
  drawLines(4, 14, F("TURN\tLOOK\n"
                     "HOLD AND TURN\tSPEED\n"
                     "CLICK\tFIRE\n"
                     "TRIG IN\tFIRE\n"
                     "HOLD\tQUIT"), 8);
  drawText(26, 57, F("CLICK TO RESUME"));
  display.invertDisplay(false);
  display.display();
  input_clear();
  while (!exit_scene) {
    if (input_fire()) break;
    if (input_long()) softReset();
    demoIfIdle();                    // the gameplay loop then ends too
  }
  sound_pause(false);
  input_clear();
  resetFrameClock();
}

static const int8_t THROTTLE_SPEED[THROTTLE_MAX - THROTTLE_MIN + 1] PROGMEM = { -50, 0, 40, 70, 100 };  // % of MOV_SPEED

// The speed name after a change; in the demo the box says DEMO instead.
static const char SPEED_NAMES[] PROGMEM = "BACK\0\0\0STOP\0\0\0WALK\0\0\0RUN\0\0\0\0SPRINT\0DEMO";
static void drawThrottle() {
  display.clearRect(0, 0, 32, 8);
  uint8_t i = demo ? 5 : throttle - THROTTLE_MIN;
  drawText(1, 1, reinterpret_cast<const __FlashStringHelper *>(SPEED_NAMES + 7 * i));
}

void loopGamePlay() {
  bool gun_fired = false;
  uint8_t gun_pos = 0;
  double view_height = 0;            // the submission read these two uninitialised
  double jogging = 0;
  uint8_t fade = GRADIENT_COUNT - 1;
  uint16_t tickAcc = 0;
  uint8_t fireLatch = 0;             // a click is kept for a few ticks until the gun is ready
  bool stepLatch = false;
  bool dead = false;
  bool lastInvert = false;
  uint8_t demoEnd = 0;

  initializeLevel(curLevel());
  if (levelID) z = 9;                // "GOAL-FIND EXIT" (never shown before)
  throttle = 0;
  steerPending = 0;
  sound_music(SONG_LEVEL);
  input_clear();
  resetFrameClock();

  do {
    uint16_t dt = fps();
    const uint8_t *level = curLevel();

    // Clear only the 3d view
    memset(display_buf, 0, SCREEN_WIDTH * (RENDER_HEIGHT / 8));

    // ---------- input, every frame ----------
    if (demo) {
      // any turn or press ends the demo (TRIG does not: the demo player fires)
      if (input_idle_ms() < DEMO_IDLE_MS) softReset();
    } else {
      if (input_fire() | input_trig()) fireLatch = 6;
      demoIfIdle();                  // before the pause: a demo started there must not restart here
      if (input_long() && !dead) pauseGame();
    }

    if (player.health > 0) {
      // Steering: counts turn the view directly, faster spins turn further.
      int8_t turn = input_turn();
      if (turn) {
        // Acceleration follows the turning speed (counts per 67 ms tick), not
        // counts per frame, so a flick turns as far at 14 fps as at 30 fps.
        int8_t at = turn < 0 ? -turn : turn;
        steerPending -= turn * STEER_PER_COUNT * (1 + at / delta * STEER_ACCEL);   // clockwise = right
      }
      if (steerPending > 0.0005 || steerPending < -0.0005) {
        // ease over ~2 frames at 30 fps, none at 15 fps or slower
        double f = delta >= 1 ? 1.0 : 0.35 + 0.65 * delta;
        double rot = steerPending * f;
        steerPending -= rot;
        rotatePlayer(rot);
      } else {
        steerPending = 0;
      }

      // Hold + turn: cruise control
      int8_t th = input_throttle();
      if (th) {
        int8_t t = constrain(throttle + th, THROTTLE_MIN, THROTTLE_MAX);
        if (t != throttle) {
          throttle = t;
          throttleMsg = 20;
          sound_play(SFX_SPEED);
        }
      }
    } else {
      input_turn();
      input_throttle();
    }

    // ---------- game logic, fixed ticks ----------
    uint8_t ticks = 0;
    tickAcc += dt;
    while (tickAcc >= TICK_MS && ticks < MAX_TICKS_PER_FRAME) { tickAcc -= TICK_MS; ticks++; }
    if (tickAcc >= TICK_MS) tickAcc = 0;

    for (uint8_t t = 0; t < ticks && !exit_scene; t++) {
      if (demo && player.health > 0 && aiTick()) fireLatch = 6;
      bool fireReq = fireLatch > 0;
      if (fireLatch) fireLatch--;
      bool hasShells = player.keys != 0;
      if (!levelID) k = player.keys;
      // The trigger zones below test the map cell the player stands in
      // (integer compares; the float compares cost ~400 bytes of flash).
      uint8_t cx = player.pos.x, cy = player.pos.y;

      // Level 1 ambush at the shells in the bottom-left room
      if (cx == 2 && cy == 54 && z == 1 && trapCount < 2) {
        spawnEntity(E_ENEMY, 1, 51);
        spawnEntity(E_ENEMY, 3, 51);
        trapCount++;
      }

      // Level 2: the boss arena
      if (levelID && cx == 46 && cy == 35) {
        player.pos.x = 12.5;
        player.pos.y = 33.5;
        enemyCount = 0;
        spawnEntity(E_ENEMY, 10, 38);
        spawnEntity(E_ENEMY, 13, 38);
        bss = true;
        sound_music(SONG_BOSS);
      }
      // Level 2: the exit
      if (levelID && cy == 55 && cx >= 12 && cx <= 22) {
        if (demo) startDemo();       // the demo skips story and score
        else { mid = 3; jumpTo(MID); }
        break;
      }
      if (bss) {
        if (EnemySpawn) {
          // next imp, alternating sides - but not after the 8th kill (the
          // submission spawned a 9th that stayed behind and read "9 OUT OF 8")
          if (enemyCount2 < 8) {
            clearEntities();
            spawnEntity(E_ENEMY, (enemyCount2 % 2 == 1) ? 13 : 10, 38);
          }
          EnemySpawn = false;
        }
        if (enemyCount2 == 8 && enemyCount != 69) {
          player.pos.y += 12;          // out of the arena
          enemyCount = 69;
          sound_music(SONG_LEVEL);
          sound_play(SFX_SECRET);
        }
      }

      if (player.health > 0) {
        // Player speed: ease towards the cruise speed (as the original did
        // towards MOV_SPEED while a direction button was held)
        double target = (int8_t)pgm_read_byte(THROTTLE_SPEED + throttle - THROTTLE_MIN) * (MOV_SPEED / 100);
        if (target != 0) player.velocity += (target - player.velocity) * .4;
        else player.velocity *= .5;
        jogging = abs(player.velocity) * (MOV_SPEED_INV * 2);

        // Update gun
        if (gun_pos > GUN_TARGET_POS) {
          // Right after fire
          gun_pos -= 2;
        } else if (gun_pos < GUN_TARGET_POS) {
          // Showing up
          gun_pos += 2;
        } else if (gun_fired) {
          // back in position: rack the shotgun
          gun_fired = false;
          reload1 = true;
        } else if (fireReq && !reload1 && (player.keys == 0 || levelID || enemyCount < 20)) {
          // ready to fire and fire pressed. No shells are spent after the 20th
          // kill of level 1 (the submission applied this on level 2 as well,
          // where the boss arena sets enemyCount to 69 - no shotgun afterwards).
          gun_pos = GUN_SHOT_POS;
          gun_fired = true;
          fireLatch = 0;
          fire();
          if (player.keys) player.keys--;
        }
      } else {
        // The player is dead
        if (!dead) {
          dead = true;
          fireLatch = 0;               // a shot latched before death must not skip GAME OVER
          throttle = 0;
          sound_music(SONG_NONE);
          sound_play(SFX_DEATH);
        }
        z = 4;
        player.velocity = 0;
        jogging = 0;
        if (view_height > -5) view_height--;
        else if (fireReq) {
          jumpTo(INTRO);
        }
        if (gun_pos > 0) gun_pos -= 2;
        else rc1 = 3;
      }

      // Level 1 complete
      // (The submission tested enemyCount == 20 with an unclamped counter: a
      // shotgun blast that killed two imps at 19 kills skipped 20 and the level
      // could never be finished.)
      if (enemyCount == 20 && !levelID) {
        z = 5;
        if (!del) {
          del = true;
          fireLatch = 0;
          sound_music(SONG_NONE);
          sound_play(SFX_WIN);
        } else if (fireReq && !demo) {
          player.pos.x = 230;
          player.pos.y = 50;
          mid = 2;
          enemyCount = 0;
          z = 3;
          jumpTo(MID);
          break;
        }
      }

      updateEntities(level);

      // Reload animation (pump: back on step 1, forward on step 5)
      if (reload1) {
        r++;
        if (r == 1 && hasShells) sound_play(SFX_PUMP1);
        if (r == 5 && hasShells) sound_play(SFX_PUMP2);
      }
      if (!hasShells) r = 7;
      if (r == 1) rc1 = 1;
      else if (r == 3) rc1 = 2;
      else if (r == 5) rc1 = 1;
      else if (r == 7) {
        r = 0;
        reload1 = false;
        rc1 = player.health > 0 ? 0 : rc1;
      }

      if (fade > 0) fade--;
      if (throttleMsg) throttleMsg--;
      // the demo starts over 3 s after a death or the end of E1M1
      if (demo && (dead || del) && ++demoEnd > 45) startDemo();

      // flash screen
      if (flash_screen > 0) {
        invert_screen = !invert_screen;
        flash_screen--;
      } else if (invert_screen) {
        invert_screen = 0;
      }
    }

    // ---------- movement, every frame ----------
    if (player.health > 0) {
      if (abs(player.velocity) > 0.003) {
        double step = player.velocity * delta;
        bool bumped = updatePosition(level, &(player.pos), player.dir.x * step, player.dir.y * step);
        if (bumped) aiHit = true;
      } else {
        player.velocity = 0;
      }
      double bob = abs(sin((double) millis() * JOGGING_SPEED));
      view_height = bob * 6 * jogging;
      bool top = bob > 0.9;
      if (jogging > 0.15 && top && !stepLatch) sound_play(SFX_STEP);
      stepLatch = top;
    }

    // ---------- render ----------
    renderMap(level, view_height);
    renderEntities(view_height);
    renderGun(gun_pos, jogging, gun_fired, rc1);
    if (fade > 0) fadeScreen(fade);
    if (throttleMsg || demo) drawThrottle();
    updateHud();

    // Draw the frame
    if (invert_screen != lastInvert) {
      display.invertDisplay(invert_screen);
      lastInvert = invert_screen;
    }
    display.display();
#ifdef HOST
    host_state(atan2(player.dir.y, player.dir.x), player.pos.x, player.pos.y, throttle, player.health, num_entities);
#endif
  } while (!exit_scene);

  display.invertDisplay(false);
}

void loop(void) {
  // Drop stale clicks and turns when a new scene starts (not when a scene
  // re-enters itself to redraw, like the score count-up).
  static uint8_t lastScene = 0xFF;
  if (scene != lastScene) {
    input_clear();
    lastScene = scene;
  }
  switch (scene) {
    case INTRO:     loopIntro();    break;
    case SCORE:     loopScore();    break;
    case MUS:       loopSetup();    break;
    case DIFF:      loopDiff();     break;
    case GAME_PLAY: loopGamePlay(); break;
    case MID:       loopMid();      break;
  }

  if (fade_e == true) { // fade out effect
    for (uint8_t i = 0; i < GRADIENT_COUNT; i++) {
      fadeScreen(i, 0);
      display.display();
      wait_ms(40);
    }
  }
  exit_scene = false;
}
