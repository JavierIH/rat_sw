# OSHWDEM 2026: the lost map, the flash wedge's cause, and other faults

At OSHWDEM 2026 (2026-10-03) the robot lost its map; the investigation the
next evening (2026-10-04), on the robot with its wheels in the air, found
why and reproduced the fault on demand. This document gathers the faults found, the evidence, how it was
reached, how to reproduce each one, the diagnosis and the fixes, with their
validation status. The earlier history of the flash freezes is in
`docs/freezes.md`; the open work in `docs/ISSUES.md` (issues 15, 17, 18).

Log of the investigation: `tools/logs/2026-10-04_19-29-11.log` (times below are
its seconds, t=0 at 19:29:11). Every number here comes from it or from the
host tests.

## Summary

1. **The flash wedge is caused by motor transients.** Hard motor reversals
   alone (no flash operation during them) leave the HSI, the internal RC
   oscillator that times flash program and erase, crawling ~8000 times slow
   while it still reads ready. A halfword then takes ~0.4 s (normal 56 us),
   an erase ~192 s (normal 22 ms) with the CPU stalled throughout, data can
   land wrong, and a reset does not boot at all (the chip boots on the HSI)
   until a power cycle. Restarting the HSI (off and on) cures it every time.
2. **The competition's lost map** fits it: the search saved only at its end,
   the save did not land (most likely a wedge), the map lived only in RAM
   with nothing on the LEDs to say so, the reset button did not boot, and
   the power cycle lost the map.
3. **A map repair livelock in the search** (found writing a test for issue
   15 a): a real wall seen once could be forgotten and seen again until the
   search gave up.
4. Smaller ones: `print()` truncation, the flash log filling up, Bluetooth
   losing the boot banner, a line-ending accident in development.

Fixes (code, all tested on the host; the robot status per fix below):
saves at the goal (search and races), save results on the LEDs, the HSI
stopped except during flash operations (each starts it fresh), map repairs
only at rest keeping the walls just seen.

## 1. What happened at the competition (OSHWDEM 2026, 2026-10-03)

The user's account (no Bluetooth log was recorded there):
- Mode 1 search went well; the robot saved the map when it got back to the
  start (the firmware then saved only there), which cost minutes.
- Race 2.4 crashed into a wall well into the maze (to be discussed apart).
- All LEDs blinked fast (`IND_FAIL`), then LEDs 1-4 blinked slowly (the race
  menu at race 4).
- No race would start after that; **the reset button did not work**; a
  power cycle was needed. After it, races 2.4-2.6 refused: no map.
- Vaguely remembered, not sure: after the first search the buttons felt
  slow or needed more than one press.

Evidence read from the robot the next evening (firmware built Sep 30
19:46:39, commit 6e19649), before touching anything:
- `STATUS`: 0 cells visited, no verified path, 5 of 6 slots free (one
  record), the build's default goal, parameters at their defaults.
- Boot banner after a console `RESET`: "Mapa en flash: 0 celdas".
- So the newest valid record was an empty map with the default goal: the
  `ERASE` done before the search. Had the search's save landed, its record
  (newer sequence number) would have won. The save never landed.
- Why: no log. The candidates were a wedged flash (the reset that did not
  boot is its known signature, `docs/freezes.md`) or a full log (five saves
  per power-on). The tests below make the wedge by far the likelier one.

What the firmware lacked: the map was saved once, at the end of the search;
a failed save printed "!!" only over Bluetooth, nothing on the LEDs; and
once the store was blocked nothing restarted the HSI, so it stayed wedged.

## 2. Fault 1: motor transients wedge the flash (the HSI)

### 2.1 How it was found

A test firmware was built for the purpose, `env:virtual` (`src/test_virtual.c`,
see AGENTS.md "Environments"): the robot firmware on the real chip and flash,
with `motion.c` replaced by the host simulator in a built-in 16x16 maze.
Step by step:

1. Saves on the chip, motors off (t=3185-3441, 3912-4060): searches, races,
   power cuts and `RESET`s during the way back, the log filled and compacted.
   All saves landed (56-63 us a halfword, 9 ms a record, 22 ms an erase).
   No wedge, but no motor activity either.
2. `TUNE RUEDAS 1` (41679c6): each virtual move turns the wheels open loop
   at its speed, ramped. Three cycles of ERASE + search + races, `RESET`s
   during motion (t=4756-5560): 14 saves right after motor work, all normal
   (worst halfword 69 us). Gentle ramps: no wedge.
3. `TUNE ESTRES n` (d0697d9): halfword writes on two scratch pages (60-61,
   never the store) while the wheels turn, right after a hard stop, through
   a hard reversal (+700 to -700 PWM at once) or a turn in place, and the
   pages erased right after the wheels stop. Result in section 2.2.
4. `TUNE INVERSION n` (53320cb): blocks of 10 hard reversals with **no**
   flash operation, then 1 s with the motors off (what the store waits
   before a save) and one timed halfword. This is the competition's case:
   the store never writes while the motors run.
5. `RESET` with the flash wedged, then the fixes (sections 2.4-2.5).

### 2.2 Evidence

| t (s) | clock | firmware | event |
|---|---|---|---|
| 5653-5755 | 21:03-21:05 | d0697d9 | `TUNE ESTRES 50`, then 250: ~15,400 halfwords and ~31 erases normal (worst 63 us, 22 ms), ~60 reversal rounds among them |
| 5761 | 21:05:12 | d0697d9 | round 190, reversing: a halfword 2,073 us; HSI restarted, the next rounds normal |
| 5771 | 21:05:22 | d0697d9 | round 214, reversing: a halfword 2,059 us, and one halfword of the round **landed wrong** (index 7); stress stopped |
| 5773-5965 | 21:05-21:08 | d0697d9 | the next erase (motors off) took **~192 s** (the cycle counter read 12,752 ms after 3 wraps of 59.65 s); the CPU stalled throughout: robot frozen, wheels still, one LED lit |
| 6000 | 21:09:11 | d0697d9 | `SAVE` (store write): "2 bytes en **435,109 us** (dato bien): HSI reiniciado; despues, peor 63 us": still wedged, cured by the store's HSI restart |
| 6243-6316 | 21:13-21:14 | 53320cb | `TUNE INVERSION 60`: after **28 blocks (280 reversals)**, the halfword at rest took **463,153 us** |
| 6329 | 21:14:40 | 53320cb | `RESET` with it wedged: "reiniciando...", then **no boot for 12.9 min**, until a power cycle (t=7104) |
| 7152-7201 | 21:28-21:29 | 352bce9 | `TUNE INVERSION 120`: wedged after **12 blocks (120 reversals)**: 393,928 us |
| 7209 | 21:29:20 | 352bce9 | `RESET`: **no boot for 4.8 min** although SystemInit now switched to the crystal first: the CPU never ran (power cycle t=7499) |
| 7541-7731 | 21:34-21:38 | 286a4fd | HSI refreshed every 10 ms: `TUNE INVERSION 110`, ~47 blocks (~470 reversals) with **no slow halfword** (worst 57 us); then the hardware reset button pressed mid-test: **booted at once** ("Reinicio: boton de reset"). Stopped there to charge the battery |

Throughout, `RCC_CR` read `030b4d83`: HSI on and ready, crystal and PLL on
and ready, clock security on. `FLASH->SR` 0. Nothing in the registers shows
a wedged HSI; only the timing does.

Ratios: halfword 394-463 ms over 56 us = 7,000-8,300; erase 192 s over 22 ms
= 8,700. Earlier wedges (`docs/freezes.md`): halfwords 0.31-0.45 s, erases
~200 s: the same state.

### 2.3 Diagnosis

- The HSI (8 MHz RC oscillator) times every flash program and erase,
  whatever clocks the CPU. Hard motor current transients, here H-bridge
  reversals at 700/1000 PWM, can leave it running ~8000 times slow while
  HSIRDY stays set. No flash operation needs to be running for it to happen
  (INVERSION), so waiting with the motors off before a save
  (`FLASH_SETTLE_MS`) does not protect: the next save just finds it wedged.
- While wedged, every program or erase stalls the CPU (it runs from that
  flash) for ~8000 times its normal time, and a halfword can land wrong.
- Restarting the HSI (`HSION` off, wait, on, wait ready) cures it: five
  times out of five (09-27 x3, 10-04 `SAVE`, and the store's restart in
  round 190). A power cycle cures it.
- A system reset does not: `HSION` stays set, the HSI is never stopped, and
  the CPU, which boots on the HSI, cannot run at all (352bce9 moved the
  switch to the crystal to the first instructions of SystemInit, and the
  reset still did not boot in 4.8 min: the CPU never got that far).
- Not proven: the electrical path (supply or ground disturbance reaching
  the RC oscillator). The chip is a clone (IDCODE 0x307); the hardware
  cannot change, so the firmware must live with it.
- Rates on the stand: wedged after 120 and 280 reversals in the two
  INVERSION runs; ~60 reversal rounds under ESTRES before the first slow
  write. Whether real runs (braking, corrections, crashes) produce such
  transients and how often is not measured; the competition's lost save and
  dead reset button say they can.

What it explains: the competition (section 1); the earlier freezes and slow
saves (`docs/freezes.md`); why restarting the HSI kept "curing" them; why a
software reset once "did not boot" (09-25). What it does not explain yet:
the slow buttons the user half-remembers (a wedged HSI does not slow the CPU
on the PLL; a wedged save stalls it ~1 s at most).

### 2.4 Fixes

| commit | change | status |
|---|---|---|
| 352bce9 | every flash program, erase and probe starts by restarting the HSI (`hsi_fresh()`); the reactive restart on a slow halfword stays | kept: now it starts a stopped HSI |
| 352bce9 | SystemInit switched to the crystal and restarted the HSI first | **did not work** (reset still dead): reverted in 286a4fd |
| 286a4fd | `health_alive()`, called by every waiting loop, restarted the HSI every 10 ms; `RESET` restarted it right before resetting | partial evidence: ~470 reversals without a wedge (P of that by chance at the stand's rate ~10%), the reset button booted mid-test. Replaced by the next one |
| e3c5b21 | the HSI stopped except during a flash operation: the clock setup stops it (on the crystal), every operation starts it fresh and stops it when done; nothing else uses it, the chip starts it at every reset and on a crystal failure. A reset always finds it stopped, so it cannot be crawling then | **validated 10-05** (virtual firmware, log 2026-10-05_19-26-38): with the HSI kept on (`TUNE HSI 1`, control) wedged after 20 reversals (497 ms); stopped, 600 reversals with no slow halfword (worst 57 us); `RESET` right after booted at once; ERASE + search + race saved normally (56-63 us, 9 ms) and the boot's compaction erased in 22 ms, the HSI off after every operation (RCC_CR ...80) |

Not done, on purpose: limiting the PWM slew (would change the control,
needs its own measurements).

### 2.5 How to reproduce

Robot on a stand, wheels free, battery charged (the stress drains it: keep
an eye on it), ST-Link connected for the flashes, Bluetooth logger running.

1. Flash the virtual firmware at the commit to test: for the wedge without
   fixes 53320cb, in a worktree (`git worktree add /tmp/rat-53320cb 53320cb`,
   then `pio run -e virtual -t upload` there); for the fixes, the current tree.
2. `TUNE RUEDAS 1` (the wheels turn; resets to off at every boot).
3. `TUNE INVERSION 60` (~4 min, in short batches for the battery): with
   the HSI kept on (`TUNE HSI 1`, as every firmware before 10-04; the
   default on 53320cb) expect "!! inversion: escritura lenta tras N bloques
   ... ~400000 us" within 10-30 blocks; with the HSI stopped but for the
   flash (the current default), "inversion: terminado sin fallos".
4. Wedged with the HSI kept on: `TUNE HSI 0` stops it, which cures it;
   on 53320cb, `LOG 1` then `SAVE` shows the store meeting it ("2 bytes en
   ~435000 us ... HSI reiniciado").
5. Wedged, `RESET`: no boot; power cycle. Do not flash while it is wedged
   (every halfword would take ~0.4 s): power cycle first.
6. `TUNE ESTRES 250`: writes during the motor transients (stops at the first
   slow halfword and times another one 1 s later).

Never run these on a firmware that drives the motors with the wheels on the
ground: the virtual firmware moves them blind.

## 3. Fault 2: the map saved only at the end, silently

- Found: the competition (section 1).
- Fix (933aff2, 85d42db, 3975b09): the search saves at the goal and again
  back at the start only if the map changed; a race saves at the goal only
  (if changed), never back at the start, and nothing if it does not reach
  the goal. Each save shows on the LEDs: three slow blinks, in flash; three
  fast, only in RAM (do not reset or power off). START refused for lack of a
  verified path blinks fast too.
- Validated: host tests (a STOP on the way back finds the goal's save in
  flash; a race stopped at its goal too); virtual robot on the chip: a power
  cut 13 s into the way back (t=3405) kept the map, the next race ran at
  cost 106 (the optimum); `RESET`s in the way back of a search and of a race
  kept it (t=4018, 4048; again with the wheels turning, t=5204, 5231).
- Reproduce: host `make -C test/host` (test_run_control); robot: virtual
  firmware, `ERASE`, mode 1, cut the power after the goal's blinks.

## 4. Fault 3: the map repair could loop until the search gave up

- Found: writing the test for issue 15 (a) (no map repair on the way).
- Scenario (test_repair_at_rest): the map believes in walls seen twice
  (+2) around the goal and in an entrance that is really closed. The robot
  reaches the entrance and sees its wall (+1): the goal is unreachable.
- Evidence (old code): "olvido 1 paredes dudosas (reparacion 1/3)" at
  (7,6), then 2/3, 3/3, then "!! destino inalcanzable": RUN_FAILED. With
  the repair on the way it also waited for the UART 9 times inside the leg's
  decisions (~140 ms each on the robot).
- Diagnosis: the repair's first pass forgets walls seen once, which
  included the real wall just seen; the phantoms (+2) survived; the robot
  saw the wall again and repaired again until `MAP_MAX_RECOVERIES`.
- Fix (3eaf8cb): repairs only at rest (on the way the leg stops), and the
  first pass keeps the walls of the robot's cell. Host tests pass; the
  simulator counts UART waits inside the legs' decisions (must stay 0).
- Reproduce: `make -C test/host`; reverting either half makes
  test_repair_at_rest fail.

## 5. Smaller findings

- `print()` cuts lines at 119 characters and drops the newline, so the next
  line glues on: the STATUS "flash:" line when blocked (issue 15 b, fixed in
  d1372fb) and the virtual firmware's first banner (82db881).
- The flash log has five free slots per power-on. With the new save points
  ERASE + search (2) + a race or two (while the walls' evidence still
  changes) fill it; the next save says "!! flash sin hueco" and blinks fast;
  the map stays in RAM, `SAVE` compacts, a power cycle too.
- Bluetooth: after a power-on the HC-05 connects after the boot banner (lost:
  send `STATUS`); lines sent while it is not connected arrive in a burst
  later; a power-off is noticed ~20 s later (supervision timeout).
- Development: `src/system_stm32f1xx.c` has CRLF line endings; rewriting it
  with a tool that normalises them turned the whole file over in 352bce9
  (restored byte for byte in 286a4fd).

## 6. Open

- Delete the flash defences the stopped HSI supersedes (`docs/review.md`,
  plan item 2).
- Flash the real firmware with every fix, `ERASE`, and check the saves
  (blinks) and STATUS's "HSI" count in a maze.
- Whether real runs wedge the HSI, and how often, stays unknown: with it
  stopped there is nothing left to wedge (STATUS "HSI n" counts only the
  slow halfwords a save still meets).
- Hardware, the user's option: the HSI, the PLL and the reset block run on
  VDDA, tied straight to the 3.3 V rail on the Blue Pill; a ferrite and 1 uF
  + 100 nF at VDDA, bulk and ceramic capacitors at the H-bridge and across
  the motors may remove the cause. `TUNE HSI 1` + `TUNE INVERSION` measure
  the wedge rate before and after.
- The race 2.4 crash; the slow buttons.
