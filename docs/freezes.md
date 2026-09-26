# Freezes (open investigation since 2026-09-25)
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
