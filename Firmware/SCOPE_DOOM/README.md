# SCOPE DOOM

Doom Nano Brutality on the Modulove SCOPE: a raycasting shooter on the
128 × 64 OLED, steered with the encoder, with sound effects and music on the
**CV OUT** jack.

Made for the **SCOPE v2.5 with an LGT8F328P** board. It also runs on v2.5
boards with a Nano and on v2 (green) boards, but without sound (see
*Sound*).

## Controls

| Action | Control |
|---|---|
| Look left / right | turn the encoder (faster turns turn further) |
| Walking speed | hold the encoder and turn it: BACK, STOP, WALK, RUN, SPRINT. The speed stays when you let go, so you can walk and turn at the same time |
| Fire | click (press and release) |
| Fire from the rack | a rising edge on the **TRIG** jack, so a clock or gate fires the shotgun |
| Pause + help | hold the encoder for 0.7 s. Click to resume, or hold again to quit to the title |
| Menus | turn to move, click to select |

The shotgun takes shells. With none left you punch. Kill 20 imps on the
first level, then find the exit of the second.

## Demo (attract mode)

Leave the module alone for **one minute**, on any screen or in the middle of a
game, and it starts playing by itself: `DEMO` in the top left corner, the two
levels in turn, a new round after every death. So an unused SCOPE in the rack
always shows something, and the OLED does not burn in a still picture.

- **Any turn or press** ends the demo and goes back to the title.
- The **TRIG** jack does not end it (a patched clock keeps running into it).
- Sound follows your SETUP choice. The demo changes no stored settings.

## Setup screen

Shown after the skill level:

- **SOUND**: OFF, EFFECTS, or FX AND MUSIC. Sound is **off** until you switch
  it on here.
- **STEER**: NORMAL or REVERSED. Use REVERSED if turning the encoder
  clockwise turns you left.
- **START**: stores both settings and starts the game.

## Sound

Sound comes from the LGT8F328P's own 8-bit DAC on D4. The DAC drives the CV
OUT jack, just like SCOPE's GEN, RND, CLK and REF modes.

- **The DAC solder jumper must be bridged to the LGT8F DAC side.** This is the
  same requirement as the SCOPE firmware has on LGT8F boards (see the main
  README). If the jumper is left on its default side, the module resets at
  random as soon as sound is on.
- While sound is off, the DAC is not switched on at all.
- While sound is on, the jack sits at about +2.5 V and the audio swings
  0 V … VCC around that. Patch it into an AC-coupled input, or into a mixer
  or VCA that can take the offset.
- Nano and v2-encoder boards have no usable DAC on D4, so the SOUND row
  reads NO DAC there.

## Hardware profile

The OLED and encoder wiring differ between hardware revisions. Doom picks the
wiring exactly like the SCOPE firmware does.

If the display stays dark, **hold the encoder while powering up**. Each hold
steps to the next of the four profiles (`P0`–`P3`), and the choice is stored.
It is the same stored byte the SCOPE firmware uses, so both always agree on
the wiring.

## Flashing

```sh
# LGT8F328P (lgt8fx core 2.0.7: https://raw.githubusercontent.com/dbuezas/lgt8fx/master/package_lgt8fx_index.json)
arduino-cli lib install Encoder
arduino-cli compile -b lgt8fx:avr:328 -u -p COM5 Firmware/SCOPE_DOOM
# Nano: -b arduino:avr:nano (or arduino:avr:nano:cpu=atmega328old)
```

The sketch has its own SSD1306 driver, so it needs no Adafruit libraries. It
fills the LGT8F's 29,696 bytes to 99 %.

> **Going back to the scope:** reflash `Firmware/SCOPEv2`. Uploading firmware
> does not erase the LGT8F's settings (measured). DOOM keeps its own settings
> at EEPROM 1000-1002 and only touches SCOPE's byte 20 (hardware profile) if
> you hold the button at power-up to cycle profiles.

## Credits and licence

- *Doom Nano* by Dave Ruiz (github.com/daveruiz/doom-nano): raycaster,
  sprites, font.
- *Doom Nano Brutality*: second level, story, reload animation, boss arena.
- SCOPE adaptation submitted by a user. LGT8F/v2.5 port, sound engine,
  encoder control and fixes by Modulove.
- `SSD1306.*` is a trimmed copy of Adafruit's SSD1306 library (BSD licence,
  notice kept in the file).
- The encoder is read with the PJRC Encoder library (MIT).

The music is an original composition, not a transcription of the Doom
soundtrack.
