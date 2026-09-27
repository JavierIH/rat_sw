# Test mazes

The robot's physical maze is 4x3 cells of 180 mm. Its internal walls are
rearranged for tests: layouts A-E below. In every layout the border is
closed, the start cell is closed to the east (as in a competition maze), the
robot starts at (0,0) facing north and the goal is (3,2) (the
`PRACTICE_MAZE` build). A new layout needs `ERASE` (or mode 6) and a new
search before speed runs.

How to read the drawings:
- Columns x = 0-3 run west to east (left to right), rows y = 0-2 south to
  north (bottom to top). S = start, G = goal.
- `---` and `|` are walls, the same symbols as the robot's `MAP` (which also
  shows `...` and `:` for walls it has not seen yet).
- A wall is named by a cell and the side of it that it closes: "north of
  (2,1)" is the wall between (2,1) and (2,2), "east of (0,1)" the one between
  (0,1) and (1,1).
- On the right, the speed run: each cell shows the direction the robot
  leaves it in. The route text is how the log prints it: a number of cells,
  then a curve right (D, derecha) or left (I, izquierda) in the last of them.
- If the robot's route or `MAP` differs from the drawing, a wall is missing
  or out of place.

## A: the original practice maze (until 2026-09-25)

```
          walls                  speed run
      0   1   2   3            0   1   2   3
    +---+---+---+---+        +---+---+---+---+
  2 |       |   | G |      2 | >   v |   | G |
    +   +   +   +   +        +   +   +   +   +
  1 |   |       |   |      1 | ^ | v     | ^ |
    +   +   +---+   +        +   +   +---+   +
  0 | S |           |      0 | S | >   >   ^ |
    +---+---+---+---+        +---+---+---+---+
```

- Walls (6): east of (0,0), (0,1), (1,2), (2,1) and (2,2); north of (2,0).
- Speed run: 9 cells and 4 curves, `2D1D2I2I2`: north 2, a U-turn through
  (0,2) and (1,2), south 2, east 2, north 2. Return: `2D2D2I1I2`. Dead end:
  (2,2).
- Taken from the robot's own `MAP` (every cell visited). Everything
  validated up to 2026-09-25 was measured here (`history.md`).

## B: long straights (2026-09-26, 09:28-09:34)

From A: remove the walls east of (0,1), (1,2) and (2,2); put walls north of
(1,1) and (2,1). One wall piece is left over.

```
          walls                  speed run
      0   1   2   3            0   1   2   3
    +---+---+---+---+        +---+---+---+---+
  2 |             G |      2 | >   >   >   G |
    +   +---+---+   +        +   +---+---+   +
  1 |           |   |      1 | ^         |   |
    +   +   +---+   +        +   +   +---+   +
  0 | S |           |      0 | S |           |
    +---+---+---+---+        +---+---+---+---+
```

- Walls (5): east of (0,0) and (2,1); north of (1,1), (2,1) and (2,0).
- Speed run: 5 cells and 1 curve, `2D3`: north 2, east 3 along the top row.
  Return: `3I2`. Dead end: (2,1).

## C: staircase of curves (2026-09-26, 09:34-12:08)

From B: remove the walls north of (1,1) and (2,0); put walls north of (0,1)
and east of (1,1).

```
          walls                  speed run
      0   1   2   3            0   1   2   3
    +---+---+---+---+        +---+---+---+---+
  2 |             G |      2 |     >   >   G |
    +---+   +---+   +        +---+   +---+   +
  1 |       |   |   |      1 | >   ^ |   |   |
    +   +   +   +   +        +   +   +   +   +
  0 | S |           |      0 | S |           |
    +---+---+---+---+        +---+---+---+---+
```

- Walls (5): east of (0,0), (1,1) and (2,1); north of (0,1) and (2,1).
- Speed run: 5 cells and 3 curves in consecutive cells (right, left,
  right), `1D1I1D2`. Return: `2I1D1I1`. Dead ends: (0,2) and (2,1).
- The scrape on the last straight and the curves turning short
  (`history.md`, `CURVE_SLIP`) were measured here.

## D: one curve, then a long straight (since 2026-09-26 12:08)

From C: move the wall north of (0,1) to the east side of (0,1).

```
          walls                  speed run
      0   1   2   3            0   1   2   3
    +---+---+---+---+        +---+---+---+---+
  2 |             G |      2 | >   >   >   G |
    +   +   +---+   +        +   +   +---+   +
  1 |   |   |   |   |      1 | ^ |   |   |   |
    +   +   +   +   +        +   +   +   +   +
  0 | S |           |      0 | S |           |
    +---+---+---+---+        +---+---+---+---+
```

- Walls (5): east of (0,0), (0,1), (1,1) and (2,1); north of (2,1).
- Expected speed run: 5 cells and 1 curve, `2D3`, the same route as B (the
  rest of the maze differs). Return: `3I2`. Dead end: (2,1).
- For the curve compensation (`CURVE_SLIP`: the heading on the straight
  after one curve) and search legs on long straights.

## E: a staircase of six curves (since 2026-09-26 19:15)

Drawn from the robot's `MAP` after its search (31 actions, no "!!").

```
          walls                  speed run
      0   1   2   3            0   1   2   3
    +---+---+---+---+        +---+---+---+---+
  2 |       |     G |      2 | >   v |     G |
    +   +   +---+   +        +   +   +---+   +
  1 |   |       |   |      1 | ^ | >   v | ^ |
    +   +   +   +   +        +   +   +   +   +
  0 | S |   |       |      0 | S |   | >   ^ |
    +---+---+---+---+        +---+---+---+---+
```

- Walls (6): east of (0,0), (1,0), (0,1), (2,1) and (1,2); north of (2,1).
- Speed run: 9 cells and 6 curves in a row, `2D1D1I1D1I1I2`: north 2, then
  a curve in every cell to (3,0), north 2 to the goal. Return: the same text.
  No straight between the curves: nothing corrects the robot's position
  there, so each curve's error carries into the next (issue 1).

## F: a ring round an island (2026-09-26 20:00, curve calibration)

From E: remove east of (0,0), east of (1,0), east of (1,2) and north of
(2,1); add east of (1,1), north of (1,0) and north of (1,1).

```
      0   1   2   3
    +---+---+---+---+
  2 |               |
    +   +---+   +   +
  1 |   |   |   |   |
    +   +---+   +   +
  0 |               |
    +---+---+---+---+
```

- Walls (5): east of (0,1), (1,1) and (2,1); north of (1,0) and (1,1).
- Only for `CAL` moves (the start is open to the east: no searches or
  speed runs). From (0,1), four `CAL CURVE 1` go round the island
  clockwise, four `CAL CURVE -1` anticlockwise, back to (0,1) with the
  same heading; the robot measures FL-FR on the border at (0,2) facing
  north or (0,0) facing south (`docs/control.md`, curve angle on the ring).

## G: E with two walls moved (2026-09-27, planner timing)

```
          walls
      0   1   2   3
    +---+---+---+---+
  2 |       |     G |
    +---+   +---+   +
  1 |           |   |
    +   +   +   +   +
  0 | S |   |       |
    +---+---+---+---+
```

- From E: the wall east of (0,1) removed, one north of (0,1) added.
- Search: goal after 11 actions, start after 27; at (2,2) on the way back
  the OPTIM phase ends and the return is planned in one decision (3 full
  plans, 3072 pops): 13.0 ms on the robot. Speed run `1D2D1I1I2`, return
  `2D1D1I2I1`. Mode 4 clean; mode 5 (480) crashed on the return (issue 12);
  race 2.5 (900/400) clean both ways, twice.

## H: the goal as a 2x2 block (built 09-27, test 4 of issue 11)

```
      0   1   2   3
    +---+---+---+---+
  2 |               |     goal (2,1)-(3,2): GOAL 2 1 3 2
    +   +   +   +   +     one way in: from (1,2)
  1 |   |   |       |
    +   +   +---+---+
  0 | S |           |
    +---+---+---+---+
```

## I: a closed ring, straights of 4 and 3 cells (built 09-27, centring)

```
      0   1   2   3
    +---+---+---+---+
  2 |               |
    +   +---+---+   +
  1 |   |       |   |
    +   +---+---+   +
  0 |               |
    +---+---+---+---+
```

- Walls (6): east of (0,1) and (2,1); north of (1,0), (2,0), (1,1) and
  (2,1). East of (1,1) makes no difference (inside the island).
- A corridor with walls on both sides everywhere: rows 0 and 2 are
  straights of 4 cells (3 cell lengths, 540 mm between corner centres, the
  simulator's `--control` straight), columns 0 and 3 of 3 cells. For `CAL
  STRAIGHT` (issue 14) and a race's long straight into a curve (issue 11,
  test 3). The start (0,0) is open to the east.

## Every wall at a glance

Only the positions that hold a wall in some layout; the other 6 internal
positions are always open.

| Wall | A | B | C | D | E |
|---|:-:|:-:|:-:|:-:|:-:|
| east of (0,0) | X | X | X | X | X |
| east of (2,1) | X | X | X | X | X |
| north of (2,1) |  | X | X | X | X |
| north of (2,0) | X | X |  |  |  |
| east of (0,1) | X |  |  | X | X |
| east of (1,1) |  |  | X | X |  |
| north of (0,1) |  |  | X |  |  |
| north of (1,1) |  | X |  |  |  |
| east of (1,2) | X |  |  |  | X |
| east of (2,2) | X |  |  |  |  |
| east of (1,0) |  |  |  |  | X |
| walls | 6 | 5 | 5 | 5 | 6 |
