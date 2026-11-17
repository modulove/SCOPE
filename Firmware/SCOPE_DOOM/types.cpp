#include <stdint.h>
#include <math.h>
#include "types.h"
#include "constants.h"

template <class T>
inline T sq(T value) {
    return value * value;
}

Coords create_coords(double x, double y) {
  return { x, y };
}

// Distance * DISTANCE_MULTIPLIER, saturated at 255. Without the clamp anything
// farther than 12.75 units wrapped around and looked close: after a teleport
// (the boss arena moves the player 34 units) distant imps could melee you.
// The root is an 8-bit integer square root of (distance * 20)^2: floor(sqrt(n))
// equals floor(sqrt(x)) for n = floor(x), so the result is the same as with
// libm's sqrt (which cost 140 bytes of flash).
uint8_t coords_distance(Coords* a, Coords* b) {
  double d2 = (sq(a->x - b->x) + sq(a->y - b->y)) * (DISTANCE_MULTIPLIER * DISTANCE_MULTIPLIER);
  if (d2 >= 65025) return 255;
  uint16_t n = d2;
  uint8_t r = 0;
  for (uint8_t bit = 0x80; bit; bit >>= 1) {
    uint8_t t = r | bit;
    if ((uint16_t)t * t <= n) r = t;
  }
  return r;
}

UID create_uid(uint8_t type, uint8_t x, uint8_t y) {
  return ((y << LEVEL_WIDTH_BASE) | x) << 4 | type;
}
  
uint8_t uid_get_type(UID uid) {
  return uid & 0x0F;
}
