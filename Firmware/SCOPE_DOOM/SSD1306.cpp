/*!
 * @file Adafruit_SSD1306.cpp
 *
 * @mainpage Arduino library for monochrome OLEDs based on SSD1306 drivers.
 *
 * @section intro_sec Introduction
 *
 * This is documentation for Adafruit's SSD1306 library for monochrome
 * OLED displays: http://www.adafruit.com/category/63_98
 *
 * These displays use I2C or SPI to communicate. I2C requires 2 pins
 * (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
 * select, data/command) and optionally a reset pin. Hardware SPI or
 * 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit-GFX-Library">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * @section license License
 *
 * BSD license, all text above, and the splash screen included below,
 * must be included in any redistribution.
 *
 */

#include <Arduino.h>
#include <SPI.h>
#include "SSD1306.h"
#include "constants.h"
#include "input.h"

// ================== SCOPE OLED transport ==================
// All OLED pins are on port B on both revisions, so the bit-bang path and the
// chip-select/data-command lines are direct port writes (no digitalWrite()).
//
//   v2.5: hardware SPI at 8 MHz. DC D9 = PB1, CS D10 = PB2, MOSI D11, SCK D13,
//         RST D8 = PB0. (SCOPE's Adafruit driver also runs this display at 8 MHz.)
//   v2:   bit-banged. MOSI D9 = PB1, CLK D10 = PB2, DC D11 = PB3, CS D12 = PB4,
//         RST D13 = PB5.
// A full frame is 1 KB: about 1 ms on hardware SPI, a few ms bit-banged - the
// submitted digitalWrite() version needed close to 100 ms.
static bool    oledHw = true;
static uint8_t oledDC = _BV(1), oledCS = _BV(2), oledRST = _BV(0);

void oled_use_pins(bool v25) {
  oledHw = v25;
  if (v25) { oledDC = _BV(1); oledCS = _BV(2); oledRST = _BV(0); }
  else     { oledDC = _BV(3); oledCS = _BV(4); oledRST = _BV(5); }
}

#ifdef HOST
static inline void oledByte(uint8_t) {}
#define OLED_PORT(x)
#else
static inline void oledByte(uint8_t v) {
  if (oledHw) { SPI.transfer(v); return; }
  for (uint8_t m = 0x80; m; m >>= 1) {
    PORTB &= ~_BV(2);                              // CLK low (D10)
    if (v & m) PORTB |= _BV(1); else PORTB &= ~_BV(1);   // MOSI (D9)
    PORTB |= _BV(2);                               // CLK high: SSD1306 samples here
  }
}
#define OLED_PORT(x) x
#endif

static inline void oledStart(bool data) {
  OLED_PORT(PORTB = data ? (PORTB | oledDC) : (PORTB & ~oledDC));
  OLED_PORT(PORTB &= ~oledCS);
}
static inline void oledEnd() { OLED_PORT(PORTB |= oledCS); }

// LOW-LEVEL UTILS ---------------------------------------------------------

template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::ssd1306_command1(uint8_t c) {
  oledStart(false);
  oledByte(c);
  oledEnd();
}

template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::ssd1306_commandList(const uint8_t *c, uint8_t n) {
  oledStart(false);
  while (n--) oledByte(pgm_read_byte(c++));
  oledEnd();
}

// ALLOCATE & INIT DISPLAY -------------------------------------------------

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
    @param  vcs
            VCC selection. Pass SSD1306_SWITCHCAPVCC to generate the display
            voltage (step up) from the 3.3V source, or SSD1306_EXTERNALVCC
            otherwise. Most situations with Adafruit SSD1306 breakouts will
            want SSD1306_SWITCHCAPVCC.
    @param  addr
            I2C address of corresponding SSD1306 display (or pass 0 to use
            default of 0x3C for 128x32 display, 0x3D for all others).
            SPI displays (hardware or software) do not use addresses, but
            this argument is still required (pass 0 or any value really,
            it will simply be ignored). Default if unspecified is 0.
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple SSD1306 displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @param  periphBegin
            If true, and if a hardware peripheral is being used (I2C or SPI,
            but not software SPI), call that peripheral's begin() function,
            else (false) it has already been done in one's sketch code.
            Cases where false might be used include multiple displays or
            other devices sharing a common bus, or situations on some
            platforms where a nonstandard begin() function is available
            (e.g. a TwoWire interface on non-default pins, as can be done
            on the ESP8266 and perhaps others).
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
bool Adafruit_SSD1306<WIDTH, HEIGHT>::begin(uint8_t vcs, uint8_t addr) {

  clearDisplay();

  vccstate = vcs;

  (void)addr; // SPI display does not use an I2C address.
#ifndef HOST
  if (oledHw) {
    // What SPI.begin() + SPISettings(8 MHz, MSBFIRST, MODE0) do, without the
    // pinMode/digitalWrite code they pull in. SPI belongs to the OLED alone.
    PORTB |= _BV(2);                           // CS high (D10 = SS: must be an output)
    DDRB |= _BV(0) | _BV(1) | _BV(2);          // RST, DC, CS
    SPCR = _BV(SPE) | _BV(MSTR);               // master, mode 0, MSB first, f/4
  #if F_CPU > 16000000L
    SPSR = 0;                                  // 32 MHz / 4 = 8 MHz
  #else
    SPSR = _BV(SPI2X);                         // 16 MHz / 2 = 8 MHz
  #endif
    DDRB |= _BV(3) | _BV(5);                   // MOSI D11, SCK D13 (after SPE, as SPI.begin)
  } else {
    DDRB |= _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5);
    PORTB |= _BV(2) | _BV(4);                  // CLK idle high, CS high
  }
  PORTB &= ~oledRST;
  wait_ms(10);
  PORTB |= oledRST;
  wait_ms(10);
#endif

  // Init sequence for the SCOPE's 128x64 panel, internal charge pump
  // (the Adafruit sequence for SSD1306_SWITCHCAPVCC, as one list).
  static const uint8_t PROGMEM init[] = {
    SSD1306_DISPLAYOFF, SSD1306_SETDISPLAYCLOCKDIV, 0x80,
    SSD1306_SETMULTIPLEX, HEIGHT - 1,
    SSD1306_SETDISPLAYOFFSET, 0x00, SSD1306_SETSTARTLINE | 0x0,
    SSD1306_CHARGEPUMP, 0x14,
    SSD1306_MEMORYMODE, 0x00, SSD1306_SEGREMAP | 0x1, SSD1306_COMSCANDEC,
    SSD1306_SETCOMPINS, 0x12, SSD1306_SETCONTRAST, 0xCF,
    SSD1306_SETPRECHARGE, 0xF1, SSD1306_SETVCOMDETECT, 0x40,
    SSD1306_DISPLAYALLON_RESUME, SSD1306_NORMALDISPLAY,
    SSD1306_DEACTIVATE_SCROLL, SSD1306_DISPLAYON };
  ssd1306_commandList(init, sizeof(init));

  return true; // Success
}

// DRAWING FUNCTIONS -------------------------------------------------------

/*!
    @brief  Set/clear/invert a single pixel. This is also invoked by the
            Adafruit_GFX library in generating many higher-level graphics
            primitives.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  color
            Pixel color, one of: SSD1306_BLACK, SSD1306_WHITE or SSD1306_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if((x >= 0) && (x < WIDTH) && (y >= 0) && (y < HEIGHT)) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch(color) {
     case SSD1306_WHITE:   buffer[x + (y/8)*WIDTH] |=  (1 << (y&7)); break;
     case SSD1306_BLACK:   buffer[x + (y/8)*WIDTH] &= ~(1 << (y&7)); break;
     case SSD1306_INVERSE: buffer[x + (y/8)*WIDTH] ^=  (1 << (y&7)); break;
    }
  }
}

/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::clearDisplay(void) {
  memset(buffer, 0, WIDTH * ((HEIGHT + 7) / 8));
}

/*!
    @brief  Draw a vertical line. This is also invoked by the Adafruit_GFX
            library in generating many higher-level graphics primitives.
    @param  x
            Column of display -- 0 at left to (screen width -1) at right.
    @param  y
            Topmost row -- 0 at top to (screen height - 1) at bottom.
    @param  h
            Height of line, in pixels.
    @param  color
            Line color, one of: SSD1306_BLACK, SSD1306_WHITE or SSD1306_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::drawFastVLine(
  int16_t x, int16_t y, int16_t h, uint16_t color) {
  drawFastVLineInternal(x, y, h, color);
}

template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::drawFastVLineInternal(
  int16_t x, int16_t __y, int16_t __h, uint16_t color) {

  if((x >= 0) && (x < WIDTH)) { // X coord in bounds?
    if(__y < 0) { // Clip top
      __h += __y;
      __y = 0;
    }
    if((__y + __h) > HEIGHT) { // Clip bottom
      __h = (HEIGHT - __y);
    }
    if(__h > 0) { // Proceed only if height is now positive
      // this display doesn't need ints for coordinates,
      // use local byte registers for faster juggling
      uint8_t  y = __y, h = __h;
      uint8_t *pBuf = &buffer[(y / 8) * WIDTH + x];

      // do the first partial byte, if necessary - this requires some masking
      uint8_t mod = (y & 7);
      if(mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;
        // note - lookup table results in a nearly 10% performance
        // improvement in fill* functions
        // uint8_t mask = ~(0xFF >> mod);
        static const uint8_t PROGMEM premask[8] =
          { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        uint8_t mask = pgm_read_byte(&premask[mod]);
        // adjust the mask if we're not going to reach the end of this byte
        if(h < mod) mask &= (0XFF >> (mod - h));

        switch(color) {
         case SSD1306_WHITE:   *pBuf |=  mask; break;
         case SSD1306_BLACK:   *pBuf &= ~mask; break;
         case SSD1306_INVERSE: *pBuf ^=  mask; break;
        }
        pBuf += WIDTH;
      }

      if(h >= mod) { // More to go?
        h -= mod;
        // Write solid bytes while we can - effectively 8 rows at a time
        if(h >= 8) {
          if(color == SSD1306_INVERSE) {
            // separate copy of the code so we don't impact performance of
            // black/white write version with an extra comparison per loop
            do {
              *pBuf ^= 0xFF;  // Invert byte
              pBuf  += WIDTH; // Advance pointer 8 rows
              h     -= 8;     // Subtract 8 rows from height
            } while(h >= 8);
          } else {
            // store a local value to work with
            uint8_t val = (color != SSD1306_BLACK) ? 255 : 0;
            do {
              *pBuf = val;    // Set byte
              pBuf += WIDTH;  // Advance pointer 8 rows
              h    -= 8;      // Subtract 8 rows from height
            } while(h >= 8);
          }
        }

        if(h) { // Do the final partial byte, if necessary
          mod = h & 7;
          // this time we want to mask the low bits of the byte,
          // vs the high bits we did above
          // uint8_t mask = (1 << mod) - 1;
          // note - lookup table results in a nearly 10% performance
          // improvement in fill* functions
          static const uint8_t PROGMEM postmask[8] =
            { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
          uint8_t mask = pgm_read_byte(&postmask[mod]);
          switch(color) {
           case SSD1306_WHITE:   *pBuf |=  mask; break;
           case SSD1306_BLACK:   *pBuf &= ~mask; break;
           case SSD1306_INVERSE: *pBuf ^=  mask; break;
          }
        }
      }
    } // endif positive height
  } // endif x in bounds
}

// Clear a rectangle, one row at a time (the original went through the
// generic fast-line routine and cost 330 bytes of flash).
template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::clearRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  if (x >= WIDTH) return;
  if (w > WIDTH - x) w = WIDTH - x;
  for (uint8_t yy = y; yy < y + h && yy < HEIGHT; yy++) {
    uint8_t m = ~(1 << (yy & 7));
    uint8_t *p = &buffer[(yy >> 3) * WIDTH + x];
    for (uint8_t i = 0; i < w; i++) p[i] &= m;
  }
}

template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::drawBitmap(int16_t x, int16_t y,
  const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color) {

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++) {
        for(int16_t i=0; i<w; i++) {
            if(i & 7) byte <<= 1;
            else      byte   = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            if(byte & 0x80) drawPixel(x+i, y, color);
        }
    }
}

/*!
    @brief  Return color of a single pixel in display buffer.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @return true if pixel is set (usually SSD1306_WHITE, unless display invert mode
            is enabled), false if clear (SSD1306_BLACK).
    @note   Reads from buffer contents; may not reflect current contents of
            screen if display() has not been called.
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
bool Adafruit_SSD1306<WIDTH, HEIGHT>::getPixel(int16_t x, int16_t y) {
  if((x >= 0) && (x < WIDTH) && (y >= 0) && (y < HEIGHT)) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    return (buffer[x + (y / 8) * WIDTH] & (1 << (y & 7)));
  }
  return false; // Pixel out of bounds
}

/*!
    @brief  Get base address of display buffer for direct reading or writing.
    @return Pointer to an unsigned 8-bit array, column-major, columns padded
            to full byte boundary if needed.
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
uint8_t *Adafruit_SSD1306<WIDTH, HEIGHT>::getBuffer(void) {
  return buffer;
}

// REFRESH DISPLAY ---------------------------------------------------------

/*!
    @brief  Push data currently in RAM to SSD1306 display.
    @return None (void).
    @note   Drawing operations are not visible until this function is
            called. Call after each graphics command, or after a whole set
            of graphics commands, as best needed by one's own application.
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::display(void) {
  // Address the complete 128x64 framebuffer, then stream its 1024 bytes.
  static const uint8_t PROGMEM addr[] = {
    SSD1306_PAGEADDR, 0, (HEIGHT / 8) - 1, SSD1306_COLUMNADDR, 0, WIDTH - 1 };
  ssd1306_commandList(addr, sizeof(addr));
  oledStart(true);
  const uint8_t *p = buffer;
  for (uint16_t n = WIDTH * ((HEIGHT + 7) / 8); n; n--) oledByte(*p++);
  oledEnd();
#ifdef HOST
  host_frame(buffer);
#endif
}

// OTHER HARDWARE SETTINGS -------------------------------------------------

/*!
    @brief  Enable or disable display invert mode (white-on-black vs
            black-on-white).
    @param  i
            If true, switch to invert mode (black-on-white), else normal
            mode (white-on-black).
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed, rather a
            different pixel mode of the display hardware is used. When
            enabled, drawing SSD1306_BLACK (value 0) pixels will actually draw white,
            SSD1306_WHITE (value 1) will draw black.
*/
template <uint8_t WIDTH, uint8_t HEIGHT>
void Adafruit_SSD1306<WIDTH, HEIGHT>::invertDisplay(bool i) {
  ssd1306_command1(i ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
#ifdef HOST
  host_invert(i);
#endif
}

// One explicit instantiation, here where the member definitions are.
template class Adafruit_SSD1306<SCREEN_WIDTH, SCREEN_HEIGHT>;
