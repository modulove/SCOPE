<p align="center">
  <img src="https://dl.modulove.de/module/scope/img/SCOPE_Logo_Gradient.png" alt="SCOPE logo" width="320">
</p>

# SCOPE — Eurorack Oscilloscope & OLED Display

**Know what your patch is actually doing.**

<!-- PLACEHOLDER: product photo. Replace the path below with the real image, e.g. docs/images/scope_front.jpg -->
![SCOPE module front panel — placeholder](docs/images/scope_front.jpg)

SCOPE brings a crisp OLED display to your rack so you can watch your waveforms, tune your oscillators and understand your signals at a glance — all in 6HP. This repository holds the firmware for all SCOPE hardware revisions.

SCOPE is Modulove's adaptation of the [HAGIWO OLED oscilloscope](https://note.com/solder_state/n/n6b4cc8d1c6b9), built on the SMD hardware by Michael Zülch ([CATs-Eurosynth](https://github.com/mzuelch/CATs-Eurosynth/tree/main/Modules/HAGIWO/Display)).

## Features

- 0.96" OLED display (SSD1306, 128 × 64)
- Full 20 Vpp input range — covers the complete Eurorack voltage spectrum, overvoltage protected
- Buffered output for clean signal passthrough and module chaining
- Rotary encoder with push button for mode selection and parameter control
- Trigger input for a stable, locked waveform display and one-shot capture of transients
- Tuner mode
- Function generator output on the newest hardware revision (v2.5) — sine, triangle, saw, square and DC
- Web-based firmware uploader — update from your browser, no Arduino IDE needed
- 6HP, skiff friendly, 40 mA (+12 V) / 10 mA (−12 V)
- Beginner friendly: all SMD parts come pre-soldered, only through-hole assembly left
- Available as panel + PCB kit, full DIY kit, or built module

**Controls:** short press — switch menu slot · rotate — adjust · medium press (1–2 s) — save settings · long press (> 3 s) — global settings (encoder direction, menu timeout, display orientation)

![SCOPE firmware UI](https://dl.modulove.de/module/scope/img/SCOPE_Firmware_UI_Main_887x512.png)

## Hardware revisions

| Revision | PCB | Firmware folder | Modes |
|---|---|---|---|
| SCOPE v1 | black | [`Firmware/SCOPE`](Firmware/SCOPE) | Oscilloscope (LFO / WAVE / SHOT), Spectrum analyzer, Tuner |
| SCOPE v2 | green | [`Firmware/SCOPEv2`](Firmware/SCOPEv2) | LFO, WAVE (triggered), TUNER |
| SCOPE v2.5 | colorful, LGT8F328P or Nano | [`Firmware/SCOPEv2`](Firmware/SCOPEv2) | as v2 + GEN (function generator) |

> Flash the firmware that matches your PCB. The v2 firmware does not run on v1 (black PCB) hardware.

`Firmware/SCOPE_DEBUG` is a debugging build of the v1 firmware.

## Firmware

**Easiest:** flash from the browser (Chrome, Edge or Opera) at **[dl.modulove.io/scope](https://dl.modulove.io/scope/)** — pick *Nano*, *Nano (old bootloader)* or *LGT8F328P* to match your board. Disconnect Eurorack power before plugging in USB.

Pre-built `.hex` files for every board are attached to each [release](https://github.com/modulove/SCOPE/releases). They are produced by the GitHub Actions workflow on every `vX.Y.Z` tag.

**Build it yourself** with [arduino-cli](https://arduino.github.io/arduino-cli/):

```sh
# boards: arduino:avr:nano | arduino:avr:nano:cpu=atmega328old | lgt8fx:avr:328
# lgt8fx core index: https://raw.githubusercontent.com/dbuezas/lgt8fx/master/package_lgt8fx_index.json
arduino-cli lib install "Adafruit GFX Library" "Adafruit SSD1306" Encoder
arduino-cli compile -b lgt8fx:avr:328 -u -p COM5 Firmware/SCOPEv2
```

`Firmware/SCOPE` (v1) additionally needs [fix_fft](https://github.com/kosme/fix_fft) for the spectrum analyzer.

## Documentation & links

- 📄 [Quick Start Guide (PDF)](https://dl.modulove.de/module/scope/SCOPE_QuickStart.pdf)
- 🔩 [Interactive BOM](https://dl.modulove.de/module/scope/ibom.html)
- 🎬 [Build video](https://www.youtube.com/watch?v=FG4vjKHGR1Q&list=PL9-2_fDMIm5cuEoAXl6-avylgxBkOdHC9) (YouTube)
- ⬆️ [Web firmware uploader](https://dl.modulove.io/scope/)
- 🛒 [Product page](https://modulove.io/modules/scope/) · [Shop](https://modulove.io/shop/eurorack/hagiwo/scope-eurorack-oled-display/)
- 🧩 [ModularGrid](https://www.modulargrid.net/e/modulove-hagiwo-oled-display-oscilloscope-spectrum-analyzer)
- 🔧 [Hardware design files](https://github.com/mzuelch/CATs-Eurosynth/tree/main/Modules/HAGIWO/Display) (CATs-Eurosynth, KiCad)
- 📝 [Original HAGIWO article](https://note.com/solder_state/n/n6b4cc8d1c6b9) · [HAGIWO video](https://www.youtube.com/watch?v=yAes5pS3ZTo)

## Credits

- [HAGIWO](https://note.com/solder_state) — original oscilloscope & spectrum analyzer code and concept
- [Michael Zülch / CATs-Eurosynth](https://github.com/mzuelch/CATs-Eurosynth) — SMD hardware
- [Modulove](https://modulove.io) — v2 / v2.5 hardware, firmware, kits and the web uploader
