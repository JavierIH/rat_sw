# Freezes (2026-09-25..26): the flash wedges

## Summary (2026-09-26 evening)
What happens: every freeze is a flash write (the map save). Sometimes the
flash wedges: from then on every program/erase takes ~9000 times longer
(an erase ~200 s instead of 22 ms, a halfword ~0.45 s instead of 56 us:
the same factor, as if the clock timing the flash crawled), until a power
cycle. The CPU runs from that flash, so it stalls through each operation
(SysTick counted 2-18 ms in 200 s; the cycle counter, with its 59.65 s
wraps, matched the log clock). A software reset then did not boot (the chip
boots on the HSI, which also times the flash: consistent with a crawling
HSI, not proven).

The chip is a clone: DBGMCU_IDCODE reads 0x00000307 without a debugger (a
genuine STM32F103 reads 0; its DEV_ID would be 0x410); CPUID 0x411FC231,
option bytes, write protection and FLASH_ACR are normal.

When: three episodes (09-25 21:1x at the goal after a search with curves;
09-25 22:05 and 09-26 11:53 at the end of a speed run), each in a write
that began within ms of the end of a move. Writes at rest (60) and 1-2 s
after hard moves (32) never wedged. Before the 21:14 build of 09-25
(d6fcf2f and earlier) 58 writes never wedged; since it (9716cfb..1090d04,
the search legs) 3 of ~35 run-end writes did. Those commits touch no flash,
clock or driver code: the link is statistical, the cause unknown.

Ruled out: the HSI off (RCC_CR 030b4d83 before and after: HSI on and
ready), flash registers (SR, CR, ACR, OBR, WRPR normal), wear (~100-400
writes), more motor load (the same speeds before and after the regression).

Fix (452756d): no run erases any more. The map is a log in two pages (six
slots of 340-byte records); a save programs an erased slot a halfword at a
time, each timed, and the first slow or failed one stops it and blocks the
store until a power cycle (a wedged flash then costs ~0.5 s once, not 200 s;
the map stays in RAM). Only the boot erases (compacts), after a power-on or
a good probe. Plus: one save per run, only if the map changed; writes wait
for the motors off 1 s and the UART quiet; RESET refuses once blocked.
After a slow halfword the HSI is restarted and one more halfword timed: if
that one is fast, the HSI was the culprit.

To confirm on the robot: normal writes show "1a ~56 us" in STATUS; a
wedge shows "!! flash: escritura LENTA, cortada" within a second, the
robot carries on, and the "tras reiniciar el HSI" line says which part
failed.

Validated on the robot (09-26 16:15, layout D, right after a power-on, no
"!!" line): `ERASE` and the search wrote once each (1st halfword 56 us,
worst 63 us, 9 ms a record); speed runs saved while the map still changed
(2), then "Mapa ya guardado (sin cambios)" (2); `SAVE` until the log was
full, then one compacted (erase 22 ms) and saved. No wedge in these 7
writes, so the "LENTA, cortada" path is still unseen on the robot: if a
"!! flash" line ever appears, copy it here. IDCODE read 0 after this
power-on (the 0x307 reads followed flashes, with the debug block enabled);
0x307 is still not a genuine DEV_ID.

## History of the investigation
Symptom: the whole robot freezes for ~200 s (no output, no reply, LEDs all
off, motors off), then carries on by itself. Flash writes around then hang
and fail (`SAVE` at rest: 199 s, then "error escribiendo la flash"; map
saves after a freeze: 14-67 s, failed). Once, a software `RESET` did not
boot at all (LEDs off) until a power cycle; after a power cycle everything
works again for a while.

Occurrences:
1. 21:1x, firmware built 21:14 (commit 1090d04, first with search legs,
   `motion_explore()` + `path_grow()`), first run after flashing: a search
   with curves. Froze 204 s at the goal right after "Meta alcanzada" (save
   failed), then 205 s at the start before the final turn (save failed after
   14 s); `SAVE` at rest 199 s, failed; `RESET` -> no boot; power cycle ->
   fine, and a stop-per-cell search and a search with curves ran clean.
2. 22:05, firmware built 21:59 (1028e8d + health checks), after a
   straight-leg search (fine, 21.4 s) and a speed run: arrived at the start
   facing the south wall and froze 196 s before turning north; the save
   then took 67 s and failed; "Fin: OK". No stall report ("el programa
   estuvo parado") and no crystal report.

What this says:
- CORRECTED 2026-09-26: the stall report was only printed in mode 5 (see
  Health checks), so its absence says nothing; the ~50 s per flash
  operation matches the HAL flash timeout, which counts SysTick ticks, so
  the CPU most likely kept running, stuck in the HAL's wait for the flash.
  No crystal report: the clock security system did not see the HSE fail.
- A CPU that stops for minutes and then resumes, plus flash operations that
  take ~50 s each (the HAL flash timeout) and fail, and a reset that cannot
  boot, all fit the CPU stalling on flash reads while the flash controller
  stays busy: something leaves the flash (controller) in a bad state that
  only a power cycle clears. What puts it there is unknown.
- Regression, not age (the user: the robot worked perfectly at 15:30 and
  for 10 years): firmwares built 17:39, 18:21, ~18:50 and 20:45 (d6fcf2f)
  ran many searches, speed runs and CAL tests with no freeze. The first
  freeze came with the first firmware containing the search legs
  (9716cfb path_grow, 53118dc motion_explore, 1090d04 CONT). Both freezes
  happened in a session where a search leg (`motion_explore`) had run
  earlier, though the second froze after a speed run. Not ruled out: the
  battery after a long day, or a latent bug the new code layout exposes.
- Static analysis: stack peak ~1.65 KB of ~2.7 KB (STATUS showed 1740 B
  never used right after boot); nothing in the legs code writes outside RAM
  that I could find (`grow_path()` masks IRQs only around `path_grow()`).

Update 2026-09-26 (evening notes, some superseded by the correction above):
- During freeze 2 the robot was already facing north (the user saw it): it
  had made the final turn and froze in the map save that follows; the log
  lines came late only because the stalled CPU could not send them. So all
  four freezes coincide with flash writes (save at the goal, two saves at
  the end, `SAVE` at rest), each lasting about 4 x 50 s, the HAL flash
  timeout (the "CPU stalled" reading was wrong: see the correction).
- LED 2 blinking afterwards was just the idle heartbeat of mode 2.
- After a power cycle the user ran several searches (straight legs, the
  default) and speed runs, saves included: no freeze.
- Hypothesis: on the STM32F1 the flash program/erase timing runs on the HSI
  RC oscillator, the same clock the chip boots on after any reset. An HSI
  that stopped or crawled would make every flash write take minutes and
  fail, would keep a software reset from booting (LEDs off), and would be
  cleared by a power cycle, while the CPU, on the crystal + PLL, runs
  normally otherwise: every symptom. Nothing in the firmware is meant to
  touch RCC->CR after the clock setup; a stray write through a corrupted
  pointer would be the candidate (the USART3 DMA handle sits at the end of
  .bss, and the DMA1 registers are 4 KB below RCC). Unconfirmed.
- Both freeze sessions began right after a flash; to ask the user: was the
  robot power-cycled between flashing and testing then, and in the earlier
  flash sessions that did not freeze?
- Next flash: before every flash write read RCC->CR (HSION/HSIRDY),
  FLASH->SR and FLASH->CR, and time the write with the DWT cycle counter
  (it keeps counting through bus stalls); report them with the result, and
  in STATUS. If HSIRDY is 0, set HSION, wait for it, and say so: that both
  confirms the hypothesis and works around it.

Plan (the user must disassemble the robot to flash: keep flashes minimal):
1. No flash: power cycle, `CONT OFF` (never runs `motion_explore`), then
   ~10 cycles of search + speed run. Note the time since power-on of any
   freeze.
2. No flash: power cycle, `CONT ON`, same cycles. If freezes only show up
   here, the legs code is the cause: review what it touches (IRQ masking,
   SysTick/path state, anything near the flash controller or 0x00000000,
   the flash alias of address 0).
3. If step 1 also freezes: flash d6fcf2f (20:45, known good) and repeat the
   cycles; then bisect 9716cfb..1028e8d.
4. Next flash, whatever the result: include e0db55a (CLOCK/CONT argument
   fix, not yet on the robot; `CLOCK HSI` still untested) and add to
   STATUS/boot: FLASH->SR, FLASH->CR, RCC->CR, RCC->CFGR, to see the flash
   controller's state after a freeze.

2026-09-26 morning session (same firmware as freeze 2, powered on at
~09:25): ~12 map saves in searches and speed runs over two hours, no
freeze. The diagnostics of step 4 (the HSI check and timing of every flash
write, the RCC watch, stall reports from the main loop) are committed
(0221cd6, 0d75630) for the next flash.

Test battery status: steps 1 (soft reset boots fine) and 3 (search, speed
run) done; step 2 (`CLOCK HSI`) failed on the argument bug; step 4 (layout
with 3-4 cell straights: remove walls (1,2)|(2,2), (2,2)|(3,2),
(0,1)|(1,1); add (1,2)/(1,1), (2,2)/(2,1)) not done. The straight-leg
search on the practice maze: same 22/36 actions and map as stopping in
every cell, 21.4 s against 20.7 s (legs of 1-2 cells at 450 mm/s do not
beat stop-and-go at 600 there).

2026-09-26 ~11:50, with the diagnostics (firmware 10:48, last reset a
power-on): search (2 saves), speed run (1 save) fine; the second speed run's
final save froze 212.8 s and failed; then `SAVE` at rest froze 199.9 s and
failed, so once stuck the flash stays stuck (as on 09-25).
- The cycle counter wraps every 59.65 s at 72 MHz: the reported 33811 and
  20616 ms are 212.8 and 199.6 s with 3 wraps, matching the log clock. The
  diagnostic line was cut before "despues FLASH_SR" (print buffer).
- SysTick counted 18 and 2 ms of those ~200 s: the CPU itself was stalled
  on the flash bus (not looping in the HAL's 50 s timeout, which it never
  reached). The first freeze's queued lines ("giro der", "En la salida")
  came out ~7 s before its error line: a long stall (erase?) then ~7 s of
  shorter ones.
- Before and after both writes: RCC_CR=030b4d83 (HSI on and ready,
  HSE/PLL/CSS on), FLASH_SR=00, FLASH_CR=0080 (locked, no error): the HSI
  hypothesis is refuted; nothing visible outside the flash explains it.
- Candidates: the flash array or its controller wedged (the page is
  rewritten on every run end: wear?), or a clone MCU whose flash behaves
  differently (GD32/CKS32: identify it, SCB->CPUID / DBGMCU->IDCODE).
- Next flash: split the diagnostic line (FLASH_SR after, HAL error code),
  print the chip identity in STATUS, and skip writes whose record equals
  what the page already holds (a speed run's end rarely changes the map).

2026-09-26 ~12:15, after a power cycle (same firmware), robot at rest on
layout D: 60 `SAVE` in a row, all 38 ms; then 32 hard moves (`CAL STRAIGHT
2 900` and `CAL TURN 2`, which do not save), each followed by a `SAVE`
1-2 s later: all fine. With failures as frequent as today's run saves
(~1 in 4-20) 92 clean writes would be very unlikely: writing itself and
the driving itself do not wedge the flash (or rarely). What the failing
writes had in common: each started within ms of the end of a move (the
goal stop, the final turn), after a hard run (two speed runs, one search
with curves), with the Bluetooth link still sending the run's lines.
Mechanism still unknown.
- The first freeze's tail: its queued lines trickled out 0.45 s apart
  over the last ~7 s, so after the ~205 s stall the CPU still spent most
  of its time stalled: the programming of the record's halfwords was slow
  too (~20 ms each instead of ~50 us). A stuck flash is slow to program,
  not only to erase: one halfword programmed and timed before the erase
  can detect it in ~20 ms and skip the erase (the plan for the protection).

Protection (firmware 490c9d5, 5595853, 563ce4f; not yet on the robot):
the map is saved once per run, at the end, and only if the record changed
(a speed run's end writes nothing); a write waits until the motors have
been off 1 s (`FLASH_SETTLE_MS`) and the UART is idle, then programs one
spare halfword after the record and times it before erasing: slow (> 1 ms)
or failed = wedged, no erase (~20 ms stall instead of ~200 s, the record
survives). Any failed or slow write locks the flash until a power cycle and
`RESET` refuses. The report fits the print buffer and adds the HAL error,
each phase's time and FLASH_ACR; `STATUS` shows CPUID, IDCODE, flash size
and the option bytes. It does not remove the cause: if a wedged flash
programmed halfwords fast, the first erase would still stall ~200 s (once).
Reproducer: `CAL FLASH [n] [ms]`, n half turns each followed by a write ms
after the stop (0 = right away, as the stuck writes were).
