/* 
todo: Moving this to CPP looks like it takes more Flash storage. Figure out why.
*/
#include "SSD1306.h"
#include "constants.h"

// Reads a char from an F() string
#define F_char(ifsh, ch)    pgm_read_byte(reinterpret_cast<PGM_P>(ifsh) + ch)

// This is slightly faster than bitRead (also bits are read from left to right)
const static uint8_t PROGMEM bit_mask[8] = { 128, 64, 32, 16, 8, 4, 2, 1 };
#define read_bit(b, n)      b & pgm_read_byte(bit_mask + n) ? 1 : 0

#include "input.h"

void setupDisplay(bool v25);
uint16_t fps();
bool getGradientPixel(uint8_t x, uint8_t y, uint8_t i);
void fadeScreen(uint8_t intensity, bool color);
void drawByte(uint8_t x, uint8_t y, uint8_t b);
uint8_t getByte(uint8_t x, uint8_t y);
void drawPixel(int8_t x, int8_t y, bool color, bool raycasterViewport);
void drawVLine(uint8_t x, int8_t start_y, int8_t end_y, uint8_t intensity);
void drawSprite(int16_t x, int8_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h, uint8_t sprite, double distance);
void drawChar(int8_t x, int8_t y, char ch, uint8_t s = 1);
void drawText(int8_t x, int8_t y, char *txt, uint8_t space = 1, uint8_t s = 1);
void drawText(int8_t x, int8_t y, const __FlashStringHelper txt, uint8_t space = 1);

// SCOPE OLED, SPI (see SSD1306.cpp for the two wirings)
Adafruit_SSD1306<SCREEN_WIDTH, SCREEN_HEIGHT> display;

// FPS control. delta is the frame time relative to the original 15 fps frame
// (TICK_MS), so per-frame movement keeps its original speed.
double delta = 1;
uint32_t lastFrameTime = 0;

#ifdef OPTIMIZE_SSD1306
// Optimizations for SSD1306 handles buffer directly
uint8_t *display_buf;
#endif

// Wall distance per column pair, distance * DISTANCE_MULTIPLIER capped at 255:
// a byte is enough (the submission used uint16_t and cleared only half of it).
uint8_t zbuffer[ZBUFFER_SIZE];

void setupDisplay(bool v25) {
  oled_use_pins(v25);
  display.begin(SSD1306_SWITCHCAPVCC, 0);

#ifdef OPTIMIZE_SSD1306
  display_buf = display.getBuffer();
#endif

  memset(zbuffer, 0xFF, sizeof(zbuffer));
}

// Waits for the frame cap and returns the milliseconds since the last frame.
// delta is clamped, so a long stall (pause screen, slow frame) cannot throw the
// player through a wall.
uint16_t fps() {
  uint32_t now;
  while ((now = millis()) - lastFrameTime < FRAME_MIN_MS) { INPUT_IDLE(); }
  uint32_t dt = now - lastFrameTime;
  lastFrameTime = now;
  if (dt > 200) dt = 200;
  delta = (double)dt / TICK_MS;
  return dt;
}

// Restart frame timing after a pause or a scene change.
void resetFrameClock() { lastFrameTime = millis() - FRAME_MIN_MS; }

// Faster way to render vertical bits
void drawByte(uint8_t x, uint8_t y, uint8_t b) {
#ifdef OPTIMIZE_SSD1306
  display_buf[(y / 8)*SCREEN_WIDTH + x] = b;
#endif
}

boolean getGradientPixel(uint8_t x, uint8_t y, uint8_t i) {
  if (i == 0) return 0;
  if (i >= GRADIENT_COUNT - 1) return 1;

  uint8_t index = max(0, min(GRADIENT_COUNT - 1, i)) * GRADIENT_WIDTH * GRADIENT_HEIGHT // gradient index
                  + y * GRADIENT_WIDTH % (GRADIENT_WIDTH * GRADIENT_HEIGHT)             // y byte offset
                  + x / GRADIENT_HEIGHT % GRADIENT_WIDTH;                               // x byte offset

  // return the bit based on x
  return read_bit(pgm_read_byte(gradient + index), x % 8);
}

void fadeScreen(uint8_t intensity, bool color = 0) {
  for (uint8_t x = 0; x < SCREEN_WIDTH; x++) {
    for (uint8_t y = 0; y < SCREEN_HEIGHT; y++) {
      if (getGradientPixel(x, y, intensity)) 
        drawPixel(x, y, color, false);
    }
  }
}

// Faster drawPixel than display.drawPixel.
// Avoids some checks to make it faster.
void drawPixel(int8_t x, int8_t y, bool color, bool raycasterViewport = false) {
  // prevent write out of screen buffer
  if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= (raycasterViewport ? RENDER_HEIGHT : SCREEN_HEIGHT)) {
    return;
  }

#ifdef OPTIMIZE_SSD1306
  if (color) {
    // white
    display_buf[x + (y / 8)*SCREEN_WIDTH] |= (1 << (y & 7));
  } else {
    // black
    display_buf[x + (y / 8)*SCREEN_WIDTH] &= ~(1 << (y & 7));
  }
#else
  display.drawPixel(x, y, color);
#endif
}

// For raycaster only
// Custom draw Vertical lines that fills with a pattern to simulate
// different brightness. Affected by RES_DIVIDER
void drawVLine(uint8_t x, int8_t start_y, int8_t end_y, uint8_t intensity) {
  int8_t y;
  int8_t lower_y = max(min(start_y, end_y), 0);
  int8_t higher_y = min(max(start_y, end_y), RENDER_HEIGHT - 1);
  uint8_t c;

#ifdef OPTIMIZE_SSD1306
  uint8_t bp = 7;   // nothing to flush if the column is empty (was read uninitialised)
  uint8_t b;
  for (c = 0; c < RES_DIVIDER; c++) {
    y = lower_y;
    b = 0;
    while (y <= higher_y) {
      bp = y % 8;
      b = b | getGradientPixel(x + c, y, intensity) << bp;

      if (bp == 7) {
        // write the whole byte
        drawByte(x + c, y, b);
        b = 0;
      }

      y++;
    }

    // draw last byte
    if (bp != 7) {
      drawByte(x + c, y - 1, b);
    }
  }
#else
  y = lower_y;
  while (y <= higher_y) {
    for (c = 0; c < RES_DIVIDER; c++) {
      // bypass black pixels
      if (getGradientPixel(x + c, y, intensity)) {
        drawPixel(x + c, y, 1, true);
      }
    }

    y++;
  }
#endif
}

// Custom drawBitmap method with scale support, mask, zindex and pattern filling
void drawSprite(
  int16_t x, int8_t y,
  const uint8_t bitmap[], const uint8_t mask[],
  int16_t w, int16_t h,
  uint8_t sprite,
  double distance
) {
  int16_t tw = (double) w / distance;    // uint8_t overflowed at point blank
  int16_t th = (double) h / distance;
  uint8_t byte_width = w / 8;
  uint8_t pixel_size = max(1, 1.0 / distance);
  uint16_t sprite_offset = byte_width * h * sprite;

  bool pixel;
  bool maskPixel;

  // Don't draw the whole sprite if its centre is hidden by the z buffer.
  // Not checked per pixel for performance reasons. (The submission clamped the
  // screen x to the z buffer size before dividing, so every sprite right of the
  // centre was tested against a column left of the centre.)
  int16_t cx = x + tw / 2;
  if (cx < 0) cx = 0;
  if (cx > SCREEN_WIDTH - 1) cx = SCREEN_WIDTH - 1;
  if (zbuffer[cx / Z_RES_DIVIDER] < distance * DISTANCE_MULTIPLIER) {
    return;
  }

  for (int16_t ty = 0; ty < th; ty += pixel_size) {
    // Don't draw out of screen
    if (y + ty < 0 || y + ty >= RENDER_HEIGHT) {
      continue;
    }

    uint8_t sy = ty * distance; // The y from the sprite

    for (int16_t tx = 0; tx < tw; tx += pixel_size) {
      uint8_t sx = tx * distance; // The x from the sprite
      uint16_t byte_offset = sprite_offset + sy * byte_width + sx / 8;

      // Don't draw out of screen
      if (x + tx < 0 || x + tx >= SCREEN_WIDTH) {
        continue;
      }

      maskPixel = read_bit(pgm_read_byte(mask + byte_offset), sx % 8);

      if (maskPixel) {
        pixel = read_bit(pgm_read_byte(bitmap + byte_offset), sx % 8);
        for (uint8_t ox = 0; ox < pixel_size; ox++) {
          for (uint8_t oy = 0; oy < pixel_size; oy++) {
            drawPixel(x + tx + ox, y + ty + oy, pixel, true);
          }
        }
      }
    }
  }
}

// Draw a single character.
// Made for a custom font with some useful sprites. Char size 4 x 6
// Uses less memory than display.print(). s = scale: 1, or 2 (8 x 12, status bar).
void drawChar(int8_t x, int8_t y, char ch, uint8_t s) {
  uint8_t c = 0;

  // Find the character
  while (CHAR_MAP[c] != ch && CHAR_MAP[c] != '\0') c++;

  for (uint8_t line = 0; line < CHAR_HEIGHT; line++) {
    // two characters per byte, the left one in the high nibble
    uint8_t b = pgm_read_byte(bmp_font + (line * bmp_font_width + c / 2));
    if (c & 1) b <<= 4;
    b &= 0xF0;
    for (int8_t px = x; b; b <<= 1, px += s)       // MSB = leftmost pixel
      if (b & 0x80)
        for (uint8_t k = 0; k < s * s; k++)
          drawPixel(px + (k & 1), y + line * s + (k >> 1), 1, false);
  }
}

// Draw a string
void drawText(int8_t x, int8_t y, char *txt, uint8_t space, uint8_t s) {
  uint8_t pos = x;
  uint8_t i = 0;
  char ch;
  while ((ch = txt[i]) != '\0') {
    drawChar(pos, y, ch, s);
    i++;
    pos += (CHAR_WIDTH + space) * s;

    // shortcut on end of screen
    if (pos > SCREEN_WIDTH) return;
  }
}

// Draw a F() string
void drawText(int8_t x, int8_t y, const __FlashStringHelper *txt_p, uint8_t space = 1) {
  uint8_t pos = x;
  uint8_t i = 0;
  char ch;
  while (pos < SCREEN_WIDTH && (ch = F_char(txt_p, i)) != '\0') {
    drawChar(pos, y, ch);
    i++;
    pos += CHAR_WIDTH + space;
  }
}

// Draw an F() text block; '\n' starts a new line 'dy' pixels lower. One call
// per screen instead of one per line (each call costs flash).
void drawLines(int8_t x, int8_t y, const __FlashStringHelper *txt_p, uint8_t dy = 6) {
  PGM_P p = reinterpret_cast<PGM_P>(txt_p);
  int8_t cx = x;
  char ch;
  while ((ch = pgm_read_byte(p++)) != '\0') {
    if (ch == '\n') { y += dy; cx = x; continue; }
    if (ch == '\t') { cx = x + 15 * (CHAR_WIDTH + 1); continue; }   // second column
    if (cx < SCREEN_WIDTH) drawChar(cx, y, ch);
    cx += CHAR_WIDTH + 1;
  }
}

// Draw an integer. The submission's 4-byte buffer ("3 digit max") overflowed
// the stack on the score screen, where scores have 4-5 digits.
void drawText(uint8_t x, uint8_t y, int16_t num, uint8_t s = 1) {
  char buf[7]; // "-32768" + \0
  itoa(num, buf, 10);
  drawText(x, y, buf, 1, s);
}
