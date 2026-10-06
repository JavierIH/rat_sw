# Docs

| File | What it holds |
|---|---|
| [ISSUES.md](ISSUES.md) | open issues (state and next step) and the closed ones; start here |
| [design.md](design.md) | how and why the firmware works as it does: speed control, IR, centring, curves, search legs, planner costs, health checks, clock, telemetry format |
| [measurements.md](measurements.md) | the numbers behind the constants in `src/robot_config.h` |
| [testing.md](testing.md) | what was validated on the robot, getting ready for a 16x16 without one, the competition checklist |
| [mazes.md](mazes.md) | the practice maze's test layouts A-I, drawn |
| [lighting.md](lighting.md) | proposal: an automatic check of the venue's light on the IR at a competition (issue 21) |
| [review.md](review.md) | outside review of the project (2026-10-04) and the work plan it suggests |
| [faults/freezes.md](faults/freezes.md) | the flash wedges (2026-09): the investigation |
| [faults/oshwdem2026.md](faults/oshwdem2026.md) | OSHWDEM 2026: the lost map, the flash wedge's cause (motor transients) and fix, other faults |
| [faults/centring.md](faults/centring.md) | the centring far from the centre line (GitHub issue #1): crooked starts, the IR delay, the PD wall follower |

Logs and recordings before 2026-10-06 are in Spanish. The firmware's
messages now read: `fin=` `end=`, `lados=` `sides=`, `PARED` `WALL`,
`BLOQUEADO` `BLOCKED`, `PERDIDO` `LOST`, `ABORTADO` `ABORTED`, `ATASCADO`
`STALLED`, `DESLIZAMIENTO` `SLIPPED`, `FALLO` `FAILED`; phases
`META`/`VUELTA` `GOAL`/`RETURN`, `RAPIDA` `FAST`; actions `AVANZA`, `IZQ`,
`DER`, `MEDIA VUELTA` `FORWARD`, `LEFT`, `RIGHT`, `U-TURN`; route texts with
`D`/`I` (derecha/izquierda) now `R`/`L` (`2D1I3` = `2R1L3`); `TUNE RUEDAS`,
`ESTRES`, `INVERSION` `WHEELS`, `STRESS`, `REVERSAL`; the monitor's `/nota`
`/note`. The calibration CSVs were migrated.
