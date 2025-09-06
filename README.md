# Hydra Delay Taptempo Buddy

This is adaptation of [FV1Buddy tap tempo module](https://github.com/ElectricCanary/FV1Buddy) by [Electric Canary](https://electric-canary.com)
for use with [Hydra multi-head Delay by PedalPCB](https://www.pedalpcb.com/product/pcb238/).
It is also available as a [kit at MusikDing.de](https://www.musikding.de/Hydra-Delay-kit).

`FV1Buddy` was not finished and some parts haven't work at the fork time.

Functional changes:

- Hydra maximum delay time is configured by default (no callibration necessary)
- "Tempo Division Switch" allows to select taping for "Head 2" or "Head 4" of the pedal, read as binary input,
- "48kHz Clock Output" and related functionality is removed
- long Tap bound 'RAMP' feature removed

Non-functional changes:

- "Momentary Tap Tempo Button Input" moved to microcontroller's pin freed by remove
  of "48kHz Clock Output". So UPDI pin is not used and "UPDI High-Voltage
  Activation" capable programmer is not necessary.
- Added hardware debounce circuit for the Tap Tempo button.
- Corrected computiong of the `pwm` value when Tab button is used.
- EEPROM storing code chaged, `avr/eeprom'h` haven't work for me.
- Implemented delayed `tap=0` storing into EEPROM to keep `tap=1` during power-off, as
  pot value change may be detected by uC as power voltage drops.
- Distinct code optimizatios

All changes are marker in source code by `MOD:` comment as accuratelly as possible.

**ToDo** How to connect taptempo module to Hydra PCB

**ToDo** updated schematics image

## Project Content

- **ToDo** `.hex` - firmware binary
- **ToDo** `_geber.zip` - Gerber file for PCB fabrication
- `firmware/` - VSCode/[PlatformIO](https://docs.platformio.org/en/latest/platforms/atmelmegaavr.html) project with firmware
- `FV1BuddyForHydra.dch` - updated schematics
- **ToDo** `.dip` - PCB design file

Schematics and PCB design file can be opened/edited by [DipTrace](https://diptrace.com/).

## License

© 2025 ElvisAlive Tone. This work is openly licensed via [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)
