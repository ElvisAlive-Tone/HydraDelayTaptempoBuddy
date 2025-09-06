# Hydra Delay Taptempo Buddy

This is adaptation of [FV1Buddy tap tempo module](https://github.com/ElectricCanary/FV1Buddy) by [Electric Canary](https://electric-canary.com)
for use with [Hydra multi-head Delay by PedalPCB](https://www.pedalpcb.com/product/pcb238/)
(also available as a [kit at musikding.de](https://www.musikding.de/Hydra-Delay-kit)).

`FV1Buddy` was not finished and some parts haven't work at the fork time.

**NOTE:** this project is not fully finished yet, firmware works, but is under testing still (and some additional feature is planned also).
HW part is under testing also, schematics will be updated and PCB designed then.

Functional changes:

- Hydra maximum delay time is used by default (no callibration necessary)
- "Tempo Division Switch" allows to select taping for "Head 2" or "Head 4" of the pedal.
  It is read as a binary input and is fully optional - defaults to "Head 4" if omitted.
- "48kHz Clock Output" and related functionality is removed
- long Tap initiated 'RAMP' feature removed

Non-functional changes:

- "Momentary Tap Tempo Button Input" moved to microcontroller's pin freed by remove
  of "48kHz Clock Output". So UPDI pin is not used and "UPDI High-Voltage
  Activation" capable programmer is not necessary. You can use
  my [Simple Serial UPDI programmer](https://github.com/ElvisAlive-Tone/updipcb).
- Added hardware debounce circuit for the Tap Tempo button.
- Corrected computiong of the `pwm` value when Tab button is used.
- EEPROM storing code chaged, `avr/eeprom'h` haven't work for me (but it might be due to next problem found later ;-).
- Implemented delayed `tap=0` storing into EEPROM to keep `tap=1` during power-off, as
  `Time Pot` value change may be detected by uC as power voltage drops.
- Distinct code optimizatios.
- Lots of comments added as I learnt the code.

All changes are marker by `MOD:` comment in the source code as accuratelly as possible.

## Features

- Delay time/tempo can be controlled by `Time Pot` or `Tap Button`
- Use `Tap Button` at least two times to switch from `Time Pot` controll to `Tqp Button` conntroll.
  - Second tap must be earlier than 1,7s
  - Subsequent tap times are averaged until tapping finishes
  - Tapping finishes if next tap is not performed for at least 3 times of the currently
    tapped tempo - `LED` blinks in the tapped tempo then
- Move `Time Pot` at least 5% to switch controll back to it - `LED` is on without blinking then
- `Tap to Head switch` (optional)
  - selects if tempo tapped by `Tap Button` targets _Head 2_ or _Head 4_. Tapping
    to _Head 2_ allows you easily set tempo in "eights" for "dotted eigts" played by _Head 3_.
  - change is not used immediatelly, but for the next tapping. Also used after next pedal power-on.
- current `Time Pot` or `Tap Button` controll mode, together with tapped-in temp, is persisted over the pedal power-off.

## Applying module into Hydra

It is really easy:

- Do not solder `Speed` pot to the Hydra PCB, but connect its pads to the module instead:
  - right most square one to `GND` - ground for the module
  - center one to `VOUT` - tempo voltage from the module back to the Hydra
  - left one to `PWR` - 3,3V power for the module
- `Time Pot` - connect Hydra's original `B100k` value `Speed` pot to the module's `TPOT1`, `TPOT2` and `TPOT3`.
  Use `Time` or `Speed` or `Delay` label for the pot as you prefer ;-)
- `Tap Button` - connect Momentary button to the module's `TBTN1` and `TBTN2`.
- `LED` - connect LED to the module's `TLED+` and `TLED-`. Use `TR1` trimmer to set LED's brightness. Used 2k value should
  be OK for the most LED types, if too small for your LED, use higher trimmer value, or connect additional resistor
  in series.
- `Tap to Head switch` - optional, simple on/off switch (on/on can be used also), connect it to the module's `THSW1` and `THSW2`.

**ToDo** Image How to connect taptempo module to Hydra PCB

## Building module

**ToDo** schematics image

**ToDo** BOM

**ToDo** PCB image

## Project Content

- **ToDo** `.hex` - firmware binary
- **ToDo** `_geber.zip` - Gerber file for PCB fabrication
- `firmware/` - VSCode/[PlatformIO](https://docs.platformio.org/en/latest/platforms/atmelmegaavr.html) project with firmware
- `FV1BuddyForHydra.dch` - updated schematics
- **ToDo** `.dip` - PCB design file

Schematics and PCB design file can be opened/edited by [DipTrace](https://diptrace.com/).

## License

© 2025 ElvisAlive Tone. This work is openly licensed via [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)
