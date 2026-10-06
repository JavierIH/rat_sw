#ifndef LIGHT_CHECK_H
#define LIGHT_CHECK_H

// Lighting check (CHECK, docs/lighting.md): how the venue's light shifts each IR, measured in the start cell.

typedef enum { LIGHT_OK, LIGHT_CORRECT, LIGHT_FAIL, LIGHT_ABORTED } light_verdict_t;

// Four quarter turns right in a cell closed W, S and E, starting and ending facing north; prints a line per sensor.
light_verdict_t light_check(void);

#endif // LIGHT_CHECK_H
