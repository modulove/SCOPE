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
uint8_t coords_distance(Coords* a, Coords* b) {
  double d = sqrt(sq(a->x - b->x) + sq(a->y - b->y)) * DISTANCE_MULTIPLIER;
  return d > 255 ? 255 : d;
}

UID create_uid(uint8_t type, uint8_t x, uint8_t y) {
  return ((y << LEVEL_WIDTH_BASE) | x) << 4 | type;
}
  
uint8_t uid_get_type(UID uid) {
  return uid & 0x0F;
}
