# Lighting check (proposal, issue 21)

A short automatic routine to run at a competition before the first search:
it measures how the venue's light changes the IR readings, corrects them
if it can, and says so with the LEDs. The trigger (mode and buttons) is
still to be decided.

Status (10-06): the measurement and the verdict are in every build as the
console's `CHECK` (`src/light_check.c`), reporting only: no correction is
applied and no LED code is shown yet. Run on the PC against a synthetic
start cell (centred and 10 mm off, +100 counts everywhere, +250 on SR, a
lamp shining into the front beams, flicker, a wall missing, saturation),
every case gave the verdict below and the robot ended facing north. Not
yet run on the robot.

## Why

The four IR emitters are always on and the ADC reads each phototransistor
continuously: there is no emitter pulsing, so no ambient subtraction (and
the hardware is fixed). Any extra light on the phototransistors adds to
every raw reading: halogen and incandescent lamps and sunlight carry a lot
of infrared (LED lighting very little), and a shiny floor reflects the
robot's own beams. Raw counts grow as a wall comes closer, so extra light
makes walls read closer and open space read as a wall.

From the calibration cubics (`infrared.c`) and the recordings (open space
reads 20-30 counts at home):

| sensor | open space at raw 0 | raw that reads 140 mm (`WALL_DETECT_MM`) | centre-line reading, and its shift with +50 / +100 counts |
|---|---|---|---|
| FL | 238 mm | 477 | 94 mm: -4.1 / -7.8 mm |
| FR | 318 mm | 668 | 94 mm: -3.8 / -7.3 mm |
| SL | 326 mm | 575 | 89 mm: -5.2 / -9.7 mm |
| SR | 189 mm | 225 | 76 mm: -4.1 / -7.8 mm |

- SR is the weak one: ~200 counts of extra light (5 % of the ADC's range)
  turn an open right side into a phantom wall in the map.
- Centring: with both walls in view, equal shifts on SL and SR cancel; with
  one wall, or a lamp on one side, the PD follows a centre line that is off
  by the shift (~4-5 mm per 50 counts).
- The stops facing a wall (`FRONT_WALL_REF_MM`, `FRONT_TRACK_REF_MM`) end
  early, and if FL and FR get different light the squaring
  (`FRONT_SQUARE_OFFSET_MM`) leaves the robot yawed.

Today the only check is the competition checklist's `IR` at the start, read
by eye (`docs/testing.md`).

## The routine

The robot in the start cell facing north, placed as for a run. The rules'
start cell is closed on three sides (west, south, east). The routine makes
four quarter turns in place, to the right, and samples every sensor for 1 s
stopped at each heading. Every sensor sees either a wall at a known
distance or open space:

| heading | FL, FR | SL | SR |
|---|---|---|---|
| N | open | W wall | E wall |
| E | E wall | open | S wall |
| S | S wall | E wall | W wall |
| W | W wall | S wall | open |

1. The hand placement cancels out. SL reads the west wall facing N and the
   east wall facing S: a sideways placement error adds to one and
   subtracts from the other, so their mean is SL's reading on the centre
   line, which `SIDE_CENTER_L_MM` says it should be. The same holds for SR
   (N and S) and for the front pair facing E and W (`FRONT_WALL_REF_MM`;
   the mean of FL and FR is insensitive to the small yaw the turns leave).
   The fronts facing S depend on the fore-aft placement: a sanity check
   only.
2. Per sensor: the offset in raw counts that, subtracted from both
   readings, makes their mean in mm the reference (a few steps of
   bisection through the calibration cubic). Extra light adds
   photocurrent, so to first order it is the same offset in counts at
   every distance (validation 2 below tests that).
3. The open reading checks the model: corrected by the offset, open space
   must read well beyond `WALL_DETECT_MM` (e.g. > 180 mm).
4. Noise: the standard deviation of each 1 s of samples. Mains lighting
   flickers at 100 Hz, and the 16x oversampling spans only ~0.5 ms, so a
   flickering lamp shows up as noise, not as an offset.

Verdict (LED codes to be chosen with the trigger):

- **OK**: every offset moves the centre-line reading by less than ~3 mm,
  open space is clear, noise is low. Nothing changes.
- **Corrected**: the offsets (raw counts, one per sensor) go into the
  parameters and `ir_mm()` subtracts them from then on. Saved like `SAVE`:
  at rest, into an erased slot. `STATUS` and the boot banner print them;
  a new check at another venue replaces them, `DEFAULTS` clears them.
- **Fail**, nothing changed: open space still reads as a wall after the
  correction; the noise is above its limit; a raw near saturation; the
  wall and open offsets disagree (the light is not additive there, e.g. a
  lamp shining straight into a beam); or an expected wall is missing (not
  in a three-walled cell, or badly placed). The console says which.
  Options then: shade or move the light if the judges allow it, or race
  the safer modes.

The console runs it with `CHECK` (2 s countdown; START or STOP cancels),
which prints a line per sensor and then the verdict:

```
SR: walls 957/957 shift -16.8mm (250 counts) noise 0.0mm open 130mm (182mm corrected) CORRECT
light: shifted: needs a correction (not applied yet, docs/lighting.md)
```

the mean raw readings of the two opposite walls, the centre-line reading's
shift (negative: walls read closer), the offset that would correct it, the
noise in mm on the centre line, and the open reading as it is and
corrected. A failing sensor adds a line saying why. It needs no
development tools (in-place turns and IR reads), so it is in the
competition build.

## Trigger (to decide)

- A fourth entry in the main menu (SELECT: 1 search, 2 race, 3 erase, 4
  check), START with the usual 5 s countdown. The recommended option: same
  gesture as everything else, no phone needed.
- SELECT held during the boot.
- Only the console's `CHECK` (needs the phone or the PC at the venue).

An option on top of any of them: a 1 s still check of the open front and
the noise at every search START (no turns, no delay worth mentioning),
warning if the light changed since the check (curtains opened, a lamp
switched on).

## Validation before relying on it (robot, `practice` build)

1. At home, under the light the cubics and centres were fitted in: `CHECK`
   gives offsets near 0. Run it three times, placing the robot 10 mm
   left, centred and 10 mm right: the means must not move.
2. The additive model: `CAL IR` (backing away from a front wall) with and
   without a lamp aimed at the maze (halogen or incandescent; a phone
   camera shows whether it emits infrared). The raw difference should be
   about the same at every distance. If it grows with the signal, an
   offset is not enough: a gain too, from the two points (wall and open).
3. Under the lamp: `CHECK`, then `WALLS` at the start, a `CAL STRAIGHT`
   with one wall, and a search on a layout with open sides, with and
   without the correction.
4. Sunlight, if possible: the worst case (saturation).

## Cost

The report: 1.75 KB of flash (the competition build keeps 9.5 KB free,
practice and dev 4.3 KB). The correction: a little more code and 8 bytes in
the saved record. The record's layout changes, so the flash that brings it
ignores the saved map: flash it before the competition day, not on it, and
run a new search.
