#include "params.h"
#include "crc32.h"
#include "robot_config.h"

// No implicit padding: the struct is checksummed and stored byte for byte.
_Static_assert(sizeof(params_t) == 20, "params_t layout changed");

static const params_t DEFAULTS = {
    .kp = PARAM_KP,
    .kd = PARAM_KD,
    .ke = PARAM_KE,
    .search_speed = PARAM_SEARCH_SPEED,
    .fast_speed = PARAM_FAST_SPEED,
    .turn_speed = PARAM_TURN_SPEED,
    .log_level = PARAM_LOG_LEVEL,
    .reserved = 0,
};

params_t params = {
    .kp = PARAM_KP,
    .kd = PARAM_KD,
    .ke = PARAM_KE,
    .search_speed = PARAM_SEARCH_SPEED,
    .fast_speed = PARAM_FAST_SPEED,
    .turn_speed = PARAM_TURN_SPEED,
    .log_level = PARAM_LOG_LEVEL,
    .reserved = 0,
};

void params_reset(void){
    params = DEFAULTS;
}

uint32_t params_defaults_signature(void){
    static const uint8_t goal[4] = {GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1};
    uint32_t crc = crc32_update(0, &DEFAULTS, sizeof(DEFAULTS));
    return crc32_update(crc, goal, sizeof(goal));
}
