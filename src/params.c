#include "params.h"
#include "crc32.h"
#include "robot_config.h"

// No implicit padding: the struct is checksummed and stored byte for byte.
_Static_assert(sizeof(params_t) == 24, "params_t layout changed");

static const params_t DEFAULTS = {
    .kp = PARAM_KP,
    .ki = PARAM_KI,
    .search_speed = PARAM_SEARCH_SPEED,
    .fast_speed = PARAM_FAST_SPEED,
    .accel = PARAM_ACCEL,
    .turn_speed = PARAM_TURN_SPEED,
    .turn_accel = PARAM_TURN_ACCEL,
    .turn_ticks = PARAM_TURN_TICKS,
    .log_level = PARAM_LOG_LEVEL,
    .telemetry = PARAM_TELEMETRY,
    .curve_speed = PARAM_CURVE_SPEED,
};

params_t params = {
    .kp = PARAM_KP,
    .ki = PARAM_KI,
    .search_speed = PARAM_SEARCH_SPEED,
    .fast_speed = PARAM_FAST_SPEED,
    .accel = PARAM_ACCEL,
    .turn_speed = PARAM_TURN_SPEED,
    .turn_accel = PARAM_TURN_ACCEL,
    .turn_ticks = PARAM_TURN_TICKS,
    .log_level = PARAM_LOG_LEVEL,
    .telemetry = PARAM_TELEMETRY,
    .curve_speed = PARAM_CURVE_SPEED,
};

void params_reset(void){
    params = DEFAULTS;
}

uint32_t params_defaults_signature(void){
    return crc32_update(0, &DEFAULTS, sizeof(DEFAULTS));
}
