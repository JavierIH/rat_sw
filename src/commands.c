#include "commands.h"
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "app.h"
#include "encoder.h"
#include "infrared.h"
#include "maze.h"
#include "motion.h"
#include "params.h"
#include "search.h"
#include "storage.h"
#include "telemetry.h"
#include "uart.h"

typedef struct {
    const char *name;
    void (*run)(const char *args);
    uint8_t idle_only;      // blocks, writes flash or uses the planner: robot stopped only
    const char *help;       // NULL = hidden alias
} command_t;

// ---- Parsing (nano-libc: no %f in scanf/printf) ------------------------------------

// Plain ASCII helpers: <ctype.h> would drag newlib's locale tables into RAM.
static uint8_t is_digit(char c){
    return c >= '0' && c <= '9';
}

static char to_upper(char c){
    return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

static const char *skip_spaces(const char *s){
    while(*s == ' ') s++;
    return s;
}

static uint8_t at_end(const char *s){
    return *skip_spaces(s) == '\0';
}

static uint8_t parse_uint(const char **s, uint32_t *out){
    const char *p = skip_spaces(*s);
    if(!is_digit(*p)) return 0;
    uint32_t v = 0;
    while(is_digit(*p)){
        v = v * 10u + (uint32_t)(*p++ - '0');
        if(v > 100000u) return 0;
    }
    *s = p;
    *out = v;
    return 1;
}

// Non-negative decimal: "2", "2.5", "0.05".
static uint8_t parse_decimal(const char **s, float *out){
    uint32_t whole;
    if(!parse_uint(s, &whole)) return 0;
    float v = (float)whole;
    const char *p = *s;
    if(*p == '.'){
        p++;
        if(!is_digit(*p)) return 0;
        float place = 0.1f;
        while(is_digit(*p)){
            v += (float)(*p++ - '0') * place;
            place *= 0.1f;
        }
    }
    *s = p;
    *out = v;
    return 1;
}

const char *format_fixed(char *buf, unsigned size, float v, uint8_t decimals){
    static const uint32_t SCALE[4] = {1u, 10u, 100u, 1000u};
    if(decimals > 3) decimals = 3;
    const char *sign = v < 0.0f ? "-" : "";
    const uint32_t units = (uint32_t)((v < 0.0f ? -v : v) * (float)SCALE[decimals] + 0.5f);
    const unsigned long whole = units / SCALE[decimals], frac = units % SCALE[decimals];
    switch(decimals){
        case 0:  snprintf(buf, size, "%s%lu", sign, whole); break;
        case 1:  snprintf(buf, size, "%s%lu.%01lu", sign, whole, frac); break;
        case 2:  snprintf(buf, size, "%s%lu.%02lu", sign, whole, frac); break;
        default: snprintf(buf, size, "%s%lu.%03lu", sign, whole, frac); break;
    }
    return buf;
}

const char *format_fixed2(char *buf, unsigned size, float v){
    return format_fixed(buf, size, v, 2);
}

static uint8_t parse_int(const char **s, int32_t *out){
    const char *p = skip_spaces(*s);
    uint8_t negative = *p == '-';
    uint32_t v;
    if(negative) p++;
    if(!is_digit(*p) || !parse_uint(&p, &v)) return 0;
    *s = p;
    *out = negative ? -(int32_t)v : (int32_t)v;
    return 1;
}

static uint8_t parse_on_off(const char *args, uint8_t *on){
    const char *p = skip_spaces(args);
    if(strcmp(p, "ON") == 0) *on = 1;
    else if(strcmp(p, "OFF") == 0) *on = 0;
    else return 0;
    return 1;
}

// ---- Commands ------------------------------------------------------------------------------

static void cmd_help(const char *args);

static void cmd_status(const char *args){
    (void)args;
    char kp[12], ki[12], kd[12], ke[12];
    uint8_t x, y;
    heading_t h;
    search_pose(&x, &y, &h);
    print("modo %u %s | %s | robot (%u,%u)%c %s\n", app_mode(), app_mode_name(app_mode()),
          app_run_active() ? "EN MARCHA" : "parado", x, y, "NESW"[h],
          search_ready() ? "en la salida" : "fuera de la salida");
    print("SPD %d FAST %d TURN %d TURNTICKS %d KP %s KI %s KD %s KE %s LOG %u%s\n", params.search_speed,
          params.fast_speed, params.turn_speed, params.turn_ticks, format_fixed2(kp, sizeof(kp), params.kp),
          format_fixed2(ki, sizeof(ki), params.ki), format_fixed2(kd, sizeof(kd), params.kd),
          format_fixed2(ke, sizeof(ke), params.ke), params.log_level, motion_step_mode() ? " | PASO A PASO" : "");
    if(app_run_active()) return;    // the planner buffers belong to the run
    uint8_t g[4];
    maze_get_goal(g);
    uint16_t cost = search_fast_path_cost();
    if(cost == PLAN_INF){
        print("meta (%u,%u)-(%u,%u) | %u celdas visitadas | sin camino rapido verificado\n",
              g[0], g[1], g[2], g[3], maze_visited_count());
    }
    else{
        print("meta (%u,%u)-(%u,%u) | %u celdas visitadas | camino rapido coste %u\n",
              g[0], g[1], g[2], g[3], maze_visited_count(), cost);
    }
}

static void cmd_mode(const char *args){
    uint32_t m;
    if(!parse_uint(&args, &m) || !at_end(args) || !app_set_mode((uint8_t)(m > 255u ? 0u : m))){
        print("MODE 1-%u\n", MODE_COUNT);
        return;
    }
    print("modo %u: %s\n", app_mode(), app_mode_name(app_mode()));
}

static void cmd_start(const char *args){
    (void)args;
    uint8_t m = app_mode();
    if(app_run_active()){
        print("ya hay un run en marcha\n");
    }
    else if(m != MODE_SENSORS && m != MODE_ERASE && !search_ready()){
        print("START rechazado: el robot no esta en la salida. Colocalo mirando al norte y manda HOME\n");
    }
    else{
        app_request_start();
        print("START modo %u: %s\n", m, app_mode_name(m));
    }
}

static void cmd_stop(const char *args){
    (void)args;
    if(!app_run_active()){
        print("nada que parar\n");
        return;
    }
    motion_request_abort();
    print("STOP\n");
}

static void cmd_pause(const char *args){
    (void)args;
    motion_set_paused(1);
    print("pausa (RESUME para seguir)\n");
}

static void cmd_resume(const char *args){
    (void)args;
    motion_set_paused(0);
    print("sigue\n");
}

static void cmd_step(const char *args){
    uint8_t on;
    if(!parse_on_off(args, &on)){
        print("STEP ON|OFF\n");
        return;
    }
    motion_set_step_mode(on);
    print(on ? "paso a paso ON: pausa tras cada accion, RESUME para seguir\n" : "paso a paso OFF\n");
}

static void set_speed(const char *args, int16_t *dst, const char *name){
    uint32_t v;
    if(!parse_uint(&args, &v) || !at_end(args) || v > 1000u){
        print("%s 0-1000\n", name);
        return;
    }
    *dst = (int16_t)v;
    print("%s=%d\n", name, *dst);
}

static void set_gain(const char *args, float *dst, float max, const char *name){
    float v;
    char buf[12];
    if(!parse_decimal(&args, &v) || !at_end(args) || v > max){
        print("%s 0-%s (decimales con punto)\n", name, format_fixed2(buf, sizeof(buf), max));
        return;
    }
    *dst = v;   // single aligned 32-bit store: the SysTick controller never sees half of it
    print("%s=%s\n", name, format_fixed2(buf, sizeof(buf), v));
}

static void cmd_spd(const char *args){ set_speed(args, &params.search_speed, "SPD"); }
static void cmd_fast(const char *args){ set_speed(args, &params.fast_speed, "FAST"); }
static void cmd_turn(const char *args){ set_speed(args, &params.turn_speed, "TURN"); }
static void cmd_turnticks(const char *args){
    uint32_t v;
    if(!parse_uint(&args, &v) || !at_end(args) || v < 300u || v > 600u){
        print("TURNTICKS 300-600 (ticks de un giro de 90; ahora %d)\n", params.turn_ticks);
        return;
    }
    params.turn_ticks = (int16_t)v;
    print("TURNTICKS=%d (SAVE para guardarlo)\n", params.turn_ticks);
}

static void cmd_kp(const char *args){ set_gain(args, &params.kp, 100.0f, "KP"); }
static void cmd_ki(const char *args){ set_gain(args, &params.ki, 100.0f, "KI"); }
static void cmd_kd(const char *args){ set_gain(args, &params.kd, 1000.0f, "KD"); }
static void cmd_ke(const char *args){ set_gain(args, &params.ke, 100.0f, "KE"); }

static void cmd_log(const char *args){
    uint32_t v;
    if(!parse_uint(&args, &v) || !at_end(args) || v > 2u){
        print("LOG 0-2 (0 eventos, 1 decisiones, 2 telemetria)\n");
        return;
    }
    params.log_level = (uint8_t)v;
    print("LOG=%u\n", params.log_level);
}

static void cmd_defaults(const char *args){
    (void)args;
    params_reset();
    print("parametros por defecto (SAVE para guardarlos)\n");
}

static void cmd_ir(const char *args){
    (void)args;
    print("IR mm FL=%d FR=%d SL=%d SR=%d | raw %u %u %u %u | enc L=%ld R=%ld\n",
          (int)ir_mm(IR_FL), (int)ir_mm(IR_FR), (int)ir_mm(IR_SL), (int)ir_mm(IR_SR),
          ir_raw(IR_FL), ir_raw(IR_FR), ir_raw(IR_SL), ir_raw(IR_SR),
          (long)encoder_total(ENCODER_L), (long)encoder_total(ENCODER_R));
}

static void cmd_walls(const char *args){
    (void)args;
    wall_sense_t w;
    if(motion_sense_walls(&w) != MOVE_OK){
        print("cancelado\n");
        return;
    }
    static const char SIGHTING[3] = {'0', '1', '?'};
    print("paredes: frente=%c izq=%c der=%c (? = lateral dudoso, no se apunta)\n",
          SIGHTING[w.front], SIGHTING[w.left], SIGHTING[w.right]);
}

static void cmd_map(const char *args){
    (void)args;
    search_print_map();
}

static void cmd_goal(const char *args){
    uint32_t v[4];
    uint8_t n = 0;
    while(n < 4 && parse_uint(&args, &v[n])) n++;
    if(n == 2){
        v[2] = v[0];
        v[3] = v[1];
    }
    if((n != 2 && n != 4) || !at_end(args) || v[0] > 255u || v[1] > 255u || v[2] > 255u || v[3] > 255u
       || !maze_set_goal((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3])){
        print("GOAL x y | GOAL x0 y0 x1 y1 (0-%u)\n", MAZE_SIZE - 1);
        return;
    }
    print("meta (%lu,%lu)-(%lu,%lu) (SAVE para guardarla)\n", (unsigned long)v[0], (unsigned long)v[1],
          (unsigned long)v[2], (unsigned long)v[3]);
    app_telemetry_sync();
}

static void cmd_save(const char *args){
    (void)args;
    print(storage_save() ? "guardado: mapa, meta y parametros\n" : "!! error escribiendo la flash\n");
}

static void cmd_erase(const char *args){
    (void)args;
    maze_init();
    print(storage_save() ? "mapa borrado (RAM y flash)\n" : "mapa borrado en RAM; !! error escribiendo la flash\n");
    app_telemetry_sync();
}

static void cmd_home(const char *args){
    (void)args;
    search_set_home();
    print("robot en la salida mirando al norte: listo\n");
}

static void cmd_sync(const char *args){
    (void)args;
    app_telemetry_sync();
}

static void cmd_telem(const char *args){
    uint8_t on;
    if(!parse_on_off(args, &on)){
        print("TELEM ON|OFF\n");
        return;
    }
    params.telemetry = on;
    print(on ? "telemetria ON\n" : "telemetria OFF\n");
    if(on && !app_run_active()) app_telemetry_sync();
}

// CAL <test> [args]: validated here, run by the main loop (so STOP keeps
// working during the test), recorded and dumped by calib.c.
static void cmd_cal(const char *args){
    static const struct { const char *name; cal_test_t test; } TESTS[] = {
        {"NOISE", CAL_NOISE}, {"STRAIGHT", CAL_STRAIGHT}, {"TURN", CAL_TURN},
        {"STEP", CAL_STEP}, {"IR", CAL_IR}, {"DUMP", CAL_DUMP},
    };
    const char *p = skip_spaces(args);
    size_t len = strcspn(p, " ");
    int found = -1;
    for(int i = 0; i < (int)(sizeof(TESTS) / sizeof(TESTS[0])); i++){
        if(strlen(TESTS[i].name) == len && strncmp(TESTS[i].name, p, len) == 0) found = i;
    }
    p += len;
    int32_t a = 0, b = 0;
    uint8_t ok = found >= 0;
    if(ok){
        switch(TESTS[found].test){
            case CAL_NOISE:     // [ms]
                a = 2000;
                if(parse_int(&p, &a)) ok = a >= 100 && a <= 3000;
                break;
            case CAL_STRAIGHT:  // [cells] [pwm]
                a = 1;
                b = params.search_speed;
                if(parse_int(&p, &a) && parse_int(&p, &b)) ok = b >= 0 && b <= 1000;
                ok = ok && a >= 1 && a <= MAZE_SIZE - 1;
                break;
            case CAL_TURN:      // [quarter turns, negative = left]
                a = 4;
                if(parse_int(&p, &a)) ok = a != 0 && a >= -8 && a <= 8;
                break;
            case CAL_STEP:      // [pwm] [ms]
                a = params.search_speed;
                b = 500;
                if(parse_int(&p, &a) && parse_int(&p, &b)) ok = b >= 50 && b <= 2000;
                ok = ok && a >= 0 && a <= 1000;
                break;
            case CAL_IR:        // [mm]
                a = 200;
                if(parse_int(&p, &a)) ok = a >= 20 && a <= 300;
                break;
            case CAL_DUMP:
                break;
        }
    }
    if(!ok || !at_end(p)){
        print("CAL NOISE [ms] | STRAIGHT [celdas] [pwm] | TURN [+-cuartos] | STEP [pwm] [ms] | IR [mm] | DUMP\n");
        return;
    }
    app_request_cal(TESTS[found].test, a, b);
}

static void cmd_reset(const char *args){
    (void)args;
    motion_stop();
    print("reiniciando...\n");
    uart_flush(1500);
    NVIC_SystemReset();
}

static const command_t COMMANDS[] = {
    {"HELP",     cmd_help,     1, "esta ayuda"},
    {"STATUS",   cmd_status,   0, "estado, parametros y mapa"},
    {"MODE",     cmd_mode,     1, "n: 1 busqueda 2 rapida 3 seg.izq 4 seg.der 5 sensores 6 borrar"},
    {"START",    cmd_start,    0, "lanza el modo seleccionado (como el boton)"},
    {"STOP",     cmd_stop,     0, "detiene el run (como START durante el run)"},
    {"PAUSE",    cmd_pause,    0, "frena y espera"},
    {"RESUME",   cmd_resume,   0, "continua tras PAUSE o un paso"},
    {"STEP",     cmd_step,     0, "ON|OFF: pausa tras cada accion"},
    {"DEBUG",    cmd_step,     0, NULL},
    {"SPD",      cmd_spd,      0, "n: PWM de busqueda y de final de recta"},
    {"FAST",     cmd_fast,     0, "n: PWM de crucero en carrera rapida"},
    {"TURN",     cmd_turn,     0, "n: PWM de giro"},
    {"TURNTICKS", cmd_turnticks, 0, "n: ticks de un giro de 90 (menos = gira menos)"},
    {"KP",       cmd_kp,       0, "f: ganancia P de centrado"},
    {"KI",       cmd_ki,       0, "f: ganancia I de centrado (corrige el desvio; 0 = off)"},
    {"KD",       cmd_kd,       0, "f: ganancia D de centrado"},
    {"KE",       cmd_ke,       0, "f: mantener rumbo sin paredes (0 = off)"},
    {"LOG",      cmd_log,      0, "0-2: detalle del log"},
    {"TELEM",    cmd_telem,    0, "ON|OFF: lineas @ para el mapa en vivo del monitor"},
    {"SYNC",     cmd_sync,     1, "reenvia mapa y estado al monitor"},
    {"CAL",      cmd_cal,      1, "NOISE|STRAIGHT|TURN|STEP|IR|DUMP: datos de calibracion"},
    {"DEFAULTS", cmd_defaults, 0, "parametros por defecto"},
    {"IR",       cmd_ir,       0, "lectura de sensores y encoders"},
    {"WALLS",    cmd_walls,    1, "detecta las paredes ahora"},
    {"MAP",      cmd_map,      1, "dibuja el mapa y el camino rapido"},
    {"GOAL",     cmd_goal,     1, "x y | x0 y0 x1 y1: celdas meta"},
    {"SAVE",     cmd_save,     1, "guarda mapa, meta y parametros"},
    {"ERASE",    cmd_erase,    1, "borra el mapa (RAM y flash)"},
    {"HOME",     cmd_home,     1, "el robot esta en la salida mirando al norte"},
    {"RESET",    cmd_reset,    0, "reinicia el micro"},
};

#define COMMAND_COUNT (sizeof(COMMANDS) / sizeof(COMMANDS[0]))

static void cmd_help(const char *args){
    (void)args;
    for(size_t i = 0; i < COMMAND_COUNT; i++){
        if(!COMMANDS[i].help) continue;
        uart_wait_space(500);
        print("%-8s %s\n", COMMANDS[i].name, COMMANDS[i].help);
    }
}

void commands_poll(void){
    // Handlers that wait (WALLS) poll inputs themselves: queue nested lines
    // instead of recursing into another handler.
    static uint8_t busy;
    if(busy) return;
    char line[48];
    if(!uart_read_line(line, sizeof(line))) return;
    busy = 1;

    for(char *p = line; *p; p++) *p = to_upper(*p);
    const char *name = skip_spaces(line);
    size_t len = strcspn(name, " ");
    const command_t *cmd = NULL;
    for(size_t i = 0; i < COMMAND_COUNT; i++){
        if(strlen(COMMANDS[i].name) == len && strncmp(COMMANDS[i].name, name, len) == 0){
            cmd = &COMMANDS[i];
            break;
        }
    }
    if(!cmd) print("? %s (HELP: lista de comandos)\n", name);
    else if(cmd->idle_only && app_run_active()) print("%s: solo con el robot parado\n", cmd->name);
    else cmd->run(name + len);
    busy = 0;
}
