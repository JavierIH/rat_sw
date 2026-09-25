#include "search.h"
#include "motion.h"
#include "params.h"
#include "robot_config.h"
#include "storage.h"
#include "telemetry.h"
#include "uart.h"

typedef struct { uint8_t x, y; heading_t h; } pose_t;

typedef enum { PH_TO_GOAL, PH_OPTIMIZE, PH_TO_START } phase_t;

typedef enum { REPLAN_OK, REPLAN_UNREACHABLE } replan_t;

static const plan_costs_t SEARCH_COSTS = {SEARCH_COST_CELL, SEARCH_COST_TURN};
static const plan_costs_t FAST_COSTS = {FAST_COST_CELL, FAST_COST_TURN};
static const char HEADING_CHAR[4] = {'N', 'E', 'S', 'W'};
static const char *const PHASE_TAG[3] = {"META", "OPTIM", "VUELTA"};
static const char *const ACTION_NAME[5] = {"-", "AVANZA", "IZQ", "DER", "MEDIA VUELTA"};

static pose_t pose = {START_X, START_Y, NORTH};
static uint8_t ready = 1;               // pose is the start facing north, for real
static uint8_t turned;                  // turned in place since the last straight
static uint16_t cost_a[MAZE_STATES];    // planner buffers
static uint16_t cost_b[MAZE_STATES];
static int8_t route_turn[PATH_MAX_CELLS];   // speed-run route: the curve in each cell (path.h)
static run_path_t route = {route_turn, 0};

static void pose_reset(void){
    pose.x = START_X;
    pose.y = START_Y;
    pose.h = NORTH;
    turned = 0;
}

static uint16_t pose_state(void){
    return maze_state(pose.x, pose.y, pose.h);
}

static void start_cell(cellset_t *s){
    cellset_clear(s);
    cellset_add(s, START_X, START_Y);
}

uint8_t search_ready(void){
    return ready;
}

void search_set_home(void){
    pose_reset();
    ready = 1;
    telemetry_pose(pose.x, pose.y, pose.h);
}

void search_set_lost(void){
    ready = 0;
}

void search_pose(uint8_t *x, uint8_t *y, heading_t *h){
    *x = pose.x;
    *y = pose.y;
    *h = pose.h;
}

// ---- Actions ---------------------------------------------------------------------

static const char SIGHTING_CHAR[3] = {'0', '1', '?'};

// Senses the three visible walls and feeds them to the map. Doubtful side
// readings (possible phantom walls) are left out: the wall keeps whatever the
// map knew, and is confirmed later from a better pose.
static move_result_t sense_here(wall_sense_t *w){
    move_result_t r = motion_sense_walls(w);
    if(r != MOVE_OK) return r;
    // Sides read from the stop after a turn caught posts and the passage just
    // driven through: 5 phantom walls in 14 such readings on the practice
    // maze. After a turn only a "no wall" is recorded; the walls there were
    // seen before the turn, by the front sensors or on the way in. (At the
    // start the robot was placed centred by hand: those readings are kept.)
    if(!w->moving && turned){
        if(w->left == SEEN_PRESENT) w->left = SEEN_DOUBTFUL;
        if(w->right == SEEN_PRESENT) w->right = SEEN_DOUBTFUL;
    }
    maze_observe(pose.x, pose.y, pose.h, w->front == SEEN_PRESENT);
    if(w->left != SEEN_DOUBTFUL) maze_observe(pose.x, pose.y, heading_left(pose.h), w->left == SEEN_PRESENT);
    if(w->right != SEEN_DOUBTFUL) maze_observe(pose.x, pose.y, heading_right(pose.h), w->right == SEEN_PRESENT);
    maze_mark_visited(pose.x, pose.y);
    telemetry_cell(pose.x, pose.y, pose.h);
    telemetry_background_row();
    return MOVE_OK;
}

static move_result_t turn_by(int8_t quarter_turns){
    if(!quarter_turns) return MOVE_OK;
    move_result_t r = motion_turn(quarter_turns);
    if(r == MOVE_OK){
        turned = 1;
        pose.h = (heading_t)((pose.h + quarter_turns + 4) & 3);
        telemetry_pose(pose.x, pose.y, pose.h);
    }
    return r;
}

// MOVE_OK: moved and pose updated. MOVE_BLOCKED: still in place, wall noted.
// Anything else: the real position is unknown.
static move_result_t forward(uint8_t cells, int16_t speed){
    int16_t end_x = (int16_t)(pose.x + cells * heading_dx(pose.h));
    int16_t end_y = (int16_t)(pose.y + cells * heading_dy(pose.h));
    if(end_x < 0 || end_x >= MAZE_SIZE || end_y < 0 || end_y >= MAZE_SIZE) return MOVE_LOST;

    move_result_t r = motion_forward(cells, speed);
    if(r == MOVE_OK){
        turned = 0;
        for(uint8_t i = 0; i < cells; i++){
            maze_mark_crossed(pose.x, pose.y, pose.h);
            pose.x = (uint8_t)(pose.x + heading_dx(pose.h));
            pose.y = (uint8_t)(pose.y + heading_dy(pose.h));
        }
        telemetry_pose(pose.x, pose.y, pose.h);
        motion_align_front();
    }
    else if(r == MOVE_BLOCKED){
        maze_mark_blocked(pose.x, pose.y, pose.h);
        telemetry_cell(pose.x, pose.y, pose.h);
    }
    return r;
}

// Drives `route` without stopping (smooth curves) and moves the pose along
// the cells actually covered. MOVE_OK: at its end. MOVE_BLOCKED: stopped
// short at a cell centre, facing a wall the map had as open (noted now).
static move_result_t run_route(int16_t speed){
    uint8_t entered = 0;
    move_result_t r = motion_run_path(&route, speed, params.curve_speed, &entered);
    for(uint8_t i = 0; i < entered; i++){
        maze_mark_crossed(pose.x, pose.y, pose.h);
        pose.x = (uint8_t)(pose.x + heading_dx(pose.h));
        pose.y = (uint8_t)(pose.y + heading_dy(pose.h));
        pose.h = (heading_t)((pose.h + route.turn[i] + 4) & 3);
    }
    if(entered) turned = 0;
    if(r == MOVE_OK || r == MOVE_BLOCKED) telemetry_pose(pose.x, pose.y, pose.h);
    if(r == MOVE_OK){
        motion_align_front();
    }
    else if(r == MOVE_BLOCKED){
        maze_mark_blocked(pose.x, pose.y, pose.h);
        telemetry_cell(pose.x, pose.y, pose.h);
    }
    return r;
}

static move_result_t do_action(action_t a){
    switch(a){
        case ACT_FORWARD:     return forward(1, params.search_speed);
        case ACT_TURN_LEFT:   return turn_by(-1);
        case ACT_TURN_RIGHT:  return turn_by(1);
        case ACT_TURN_AROUND: return turn_by(2);
        case ACT_NONE:        break;
    }
    return MOVE_OK;
}

static move_result_t face(heading_t h){
    int8_t q = (int8_t)((h - pose.h + 4) & 3);
    return turn_by(q == 3 ? -1 : q);
}

// ---- Run endings -------------------------------------------------------------------

static run_result_t fail_move(move_result_t r, const char *what){
    ready = 0;
    if(r == MOVE_ABORTED){
        print("Run detenido en (%u,%u)%c\n", pose.x, pose.y, HEADING_CHAR[pose.h]);
        return RUN_ABORTED;
    }
    print("!! %s: %s en (%u,%u)%c. Posicion no fiable: lleva el robot a la salida\n",
          what, move_result_name(r), pose.x, pose.y, HEADING_CHAR[pose.h]);
    motion_indicate(IND_FAIL);
    return RUN_FAILED;
}

static run_result_t fail_plan(const char *why){
    ready = 0;
    print("!! %s en (%u,%u)%c\n", why, pose.x, pose.y, HEADING_CHAR[pose.h]);
    motion_indicate(IND_FAIL);
    return RUN_FAILED;
}

static void save_map(void){
    if(storage_save()) print("Mapa guardado (%u celdas visitadas)\n", maze_visited_count());
    else print("!! no se pudo guardar el mapa en flash\n");
}

static run_result_t finish_at_start(uint16_t steps){
    move_result_t r = face(NORTH);
    if(r != MOVE_OK) return fail_move(r, "orientacion");
    ready = 1;
    print("En la salida tras %u acciones\n", steps);
    save_map();
    uint16_t cost = search_fast_path_cost();
    if(cost != PLAN_INF) print("Camino rapido verificado: coste %u\n", cost);
    motion_indicate(IND_DONE);
    return RUN_OK;
}

// ---- Planning helpers ----------------------------------------------------------------

// Plans towards `targets` over the optimistic map into cost_a. If a phantom
// wall sealed the targets off, forgets doubtful walls (first those seen only
// once, then all) instead of giving up: the position is still trusted, so
// the robot simply re-learns them.
static replan_t plan_explore(const cellset_t *targets, uint8_t *repairs){
    maze_plan_to(targets, PLAN_OPTIMISTIC, SEARCH_COSTS, cost_a);
    if(cost_a[pose_state()] != PLAN_INF) return REPLAN_OK;
    if(*repairs >= MAP_MAX_RECOVERIES) return REPLAN_UNREACHABLE;
    (*repairs)++;
    uint16_t forgotten = maze_forget_walls(1);
    maze_plan_to(targets, PLAN_OPTIMISTIC, SEARCH_COSTS, cost_a);
    if(cost_a[pose_state()] == PLAN_INF){
        forgotten = (uint16_t)(forgotten + maze_forget_walls(INT8_MAX));
        maze_plan_to(targets, PLAN_OPTIMISTIC, SEARCH_COSTS, cost_a);
    }
    print("!! destino inalcanzable segun el mapa: olvido %u paredes dudosas (reparacion %u/%u)\n",
          forgotten, *repairs, MAP_MAX_RECOVERIES);
    telemetry_map();    // the forgotten walls can be anywhere: resend the whole map
    return cost_a[pose_state()] != PLAN_INF ? REPLAN_OK : REPLAN_UNREACHABLE;
}

// Unvisited cells on some optimistic optimal speed-run path from the start to
// the goal. Once there are none, the best speed-run path runs through visited
// cells only, whose walls are all known: it is verified optimal.
static uint16_t optimize_candidates(cellset_t *out){
    cellset_t goal;
    maze_goal_cells(&goal);
    maze_plan_from(START_X, START_Y, NORTH, PLAN_OPTIMISTIC, FAST_COSTS, cost_a);
    maze_plan_to(&goal, PLAN_OPTIMISTIC, FAST_COSTS, cost_b);
    uint32_t best = cost_b[maze_state(START_X, START_Y, NORTH)];
    cellset_clear(out);
    if(best == PLAN_INF) return 0;

    uint16_t count = 0;
    for(uint8_t y = 0; y < MAZE_SIZE; y++){
        for(uint8_t x = 0; x < MAZE_SIZE; x++){
            if(maze_is_visited(x, y)) continue;
            for(uint8_t h = 0; h < 4; h++){
                uint16_t s = maze_state(x, y, (heading_t)h);
                if(cost_a[s] != PLAN_INF && cost_b[s] != PLAN_INF && (uint32_t)cost_a[s] + cost_b[s] == best){
                    cellset_add(out, x, y);
                    count++;
                    break;
                }
            }
        }
    }
    return count;
}

// ---- Strategies ------------------------------------------------------------------------

run_result_t search_explore(void){
    phase_t phase = PH_TO_GOAL;
    uint16_t steps = 0, optimize_steps = 0;
    uint8_t repairs = 0;
    cellset_t targets;

    pose_reset();
    ready = 0;
    telemetry_activity(TM_TO_GOAL);
    print("== BUSQUEDA ==\n");
    for(;;){
        if(!motion_checkpoint()) return fail_move(MOVE_ABORTED, "busqueda");
        wall_sense_t w;
        move_result_t r = sense_here(&w);
        if(r != MOVE_OK) return fail_move(r, "sensado");

        if(phase == PH_TO_GOAL && maze_is_goal(pose.x, pose.y)){
            print("Meta alcanzada en (%u,%u) tras %u acciones\n", pose.x, pose.y, steps);
            motion_indicate(IND_GOAL);
            save_map();
            phase = PH_OPTIMIZE;
            telemetry_activity(TM_OPTIMIZE);
        }
        if(phase == PH_OPTIMIZE){
            uint8_t budget_left = optimize_steps < OPTIMIZE_MAX_STEPS;
            if(!budget_left || !optimize_candidates(&targets)){
                print(budget_left ? "Camino rapido optimo verificado: vuelta a la salida\n"
                                  : "Presupuesto de optimizacion agotado: vuelta a la salida\n");
                phase = PH_TO_START;
                telemetry_activity(TM_TO_START);
            }
        }
        if(phase == PH_TO_START && pose.x == START_X && pose.y == START_Y){
            return finish_at_start(steps);
        }
        if(phase == PH_TO_GOAL) maze_goal_cells(&targets);
        else if(phase == PH_TO_START) start_cell(&targets);

        if(plan_explore(&targets, &repairs) == REPLAN_UNREACHABLE){
            return fail_plan("destino inalcanzable");
        }
        action_t a = maze_best_action(cost_a, pose.x, pose.y, pose.h, PLAN_OPTIMISTIC, SEARCH_COSTS);
        if(params.log_level >= 1){
            print("%s (%u,%u)%c F%c I%c D%c coste=%u -> %s\n", PHASE_TAG[phase], pose.x, pose.y,
                  HEADING_CHAR[pose.h], SIGHTING_CHAR[w.front], SIGHTING_CHAR[w.left], SIGHTING_CHAR[w.right],
                  cost_a[pose_state()], ACTION_NAME[a]);
        }
        if(++steps > SEARCH_MAX_STEPS) return fail_plan("presupuesto de acciones agotado");
        if(phase == PH_OPTIMIZE) optimize_steps++;
        // The map may still believe in a passage the sensors now see closed:
        // never drive into it. The sighting already raised its evidence, so
        // sensing again converges to the truth.
        if(a == ACT_FORWARD && w.front == SEEN_PRESENT) continue;
        r = do_action(a);
        if(r != MOVE_OK && r != MOVE_BLOCKED) return fail_move(r, "movimiento");
    }
}

// The route as the log shows it: cells straight ahead, then D/I for a curve
// right/left in the last of them ("2D1I3": 2 cells curving right in the
// second, 1 cell curving left, 3 cells). Cut with '+' if too long.
#define ROUTE_TEXT_MAX 40
static const char *route_text(char *text){
    uint8_t n = 0, run = 0;
    for(uint8_t i = 0; i < route.cells; i++){
        run++;
        if(!route_turn[i] && i + 1u < route.cells) continue;
        if(n + 5u > ROUTE_TEXT_MAX){
            text[n++] = '+';
            break;
        }
        if(run >= 100) text[n++] = (char)('0' + run / 100);
        if(run >= 10) text[n++] = (char)('0' + run / 10 % 10);
        text[n++] = (char)('0' + run % 10);
        if(route_turn[i]) text[n++] = route_turn[i] > 0 ? 'D' : 'I';
        run = 0;
    }
    text[n] = '\0';
    return text;
}

// Drives to `targets` over verified passages in one go (straights and
// smooth curves), sensing at every stop. If the verified map has no route (a
// wall appeared where it was believed open), explores step by step instead.
static run_result_t drive_to(const cellset_t *targets, int16_t speed, const char *tag, uint16_t *steps){
    uint8_t repairs = 0;
    while(!cellset_has(targets, pose.x, pose.y)){
        if(!motion_checkpoint()) return fail_move(MOVE_ABORTED, tag);
        if(++*steps > SEARCH_MAX_STEPS) return fail_plan("presupuesto de acciones agotado");
        wall_sense_t w;
        move_result_t r = sense_here(&w);
        if(r != MOVE_OK) return fail_move(r, "sensado");

        maze_plan_to(targets, PLAN_VERIFIED, FAST_COSTS, cost_a);
        int8_t turn;
        if(maze_route(cost_a, pose.x, pose.y, pose.h, PLAN_VERIFIED, FAST_COSTS, &turn, route_turn, PATH_MAX_CELLS,
                      &route.cells)){
            if(turn == 0 && w.front == SEEN_PRESENT) continue;
            if(params.log_level >= 1){
                char text[ROUTE_TEXT_MAX + 2];
                print("%s (%u,%u)%c giro %d + ruta %s (%u celdas)\n", tag, pose.x, pose.y, HEADING_CHAR[pose.h], turn,
                      route_text(text), route.cells);
            }
            r = turn_by(turn);
            if(r == MOVE_OK && route.cells) r = run_route(speed);
        }
        else{
            if(plan_explore(targets, &repairs) == REPLAN_UNREACHABLE) return fail_plan("destino inalcanzable");
            action_t a = maze_best_action(cost_a, pose.x, pose.y, pose.h, PLAN_OPTIMISTIC, SEARCH_COSTS);
            if(params.log_level >= 1){
                print("%s (%u,%u)%c sin camino verificado -> %s\n", tag, pose.x, pose.y,
                      HEADING_CHAR[pose.h], ACTION_NAME[a]);
            }
            if(a == ACT_FORWARD && w.front == SEEN_PRESENT) continue;
            r = do_action(a);
        }
        if(r != MOVE_OK && r != MOVE_BLOCKED) return fail_move(r, tag);
    }
    return RUN_OK;
}

run_result_t search_fast_run(void){
    cellset_t goal, home;
    maze_goal_cells(&goal);
    start_cell(&home);
    pose_reset();

    uint16_t cost = search_fast_path_cost();
    if(cost == PLAN_INF){
        print("Sin camino verificado salida->meta: haz antes una busqueda (modo 1)\n");
        return RUN_FAILED;  // nothing moved: still ready
    }
    ready = 0;
    telemetry_activity(TM_FAST);
    print("== CARRERA RAPIDA (coste %u) ==\n", cost);
    uint16_t steps = 0;
    run_result_t res = drive_to(&goal, params.fast_speed, "RAPIDA", &steps);
    if(res != RUN_OK) return res;
    print("Meta alcanzada en carrera rapida tras %u tramos\n", steps);
    motion_indicate(IND_GOAL);
    telemetry_activity(TM_RETURN);
    res = drive_to(&home, params.search_speed, "VUELTA", &steps);
    if(res != RUN_OK) return res;
    return finish_at_start(steps);
}

run_result_t search_wall_follow(uint8_t left_hand){
    const int8_t near_turn = left_hand ? -1 : 1;
    uint16_t steps = 0;

    pose_reset();
    ready = 0;
    telemetry_activity(TM_FOLLOW);
    print("== SEGUIDOR DE PARED %s ==\n", left_hand ? "IZQUIERDA" : "DERECHA");
    while(!maze_is_goal(pose.x, pose.y)){
        if(!motion_checkpoint()) return fail_move(MOVE_ABORTED, "seguidor");
        if(++steps > SEARCH_MAX_STEPS) return fail_plan("presupuesto de acciones agotado");
        wall_sense_t w;
        move_result_t r = sense_here(&w);
        if(r != MOVE_OK) return fail_move(r, "sensado");

        // Decide on the map, which now holds this sighting plus the border.
        heading_t near_side = left_hand ? heading_left(pose.h) : heading_right(pose.h);
        heading_t far_side = heading_back(near_side);
        int8_t q;
        if(maze_wall(pose.x, pose.y, near_side) != WALL_PRESENT) q = near_turn;
        else if(maze_wall(pose.x, pose.y, pose.h) != WALL_PRESENT) q = 0;
        else if(maze_wall(pose.x, pose.y, far_side) != WALL_PRESENT) q = (int8_t)-near_turn;
        else q = 2;

        r = turn_by(q);
        if(r == MOVE_OK) r = forward(1, params.search_speed);
        if(r != MOVE_OK && r != MOVE_BLOCKED) return fail_move(r, "movimiento");
    }
    print("Meta alcanzada (seguidor) en (%u,%u) tras %u acciones\n", pose.x, pose.y, steps);
    motion_indicate(IND_GOAL);
    return RUN_OK;
}

// ---- Reports ---------------------------------------------------------------------------

uint16_t search_fast_path_cost(void){
    cellset_t goal;
    maze_goal_cells(&goal);
    maze_plan_to(&goal, PLAN_VERIFIED, FAST_COSTS, cost_a);
    return cost_a[maze_state(START_X, START_Y, NORTH)];
}

// Cells of the verified speed-run path from the start (empty if none).
static void trace_fast_path(cellset_t *path){
    cellset_clear(path);
    if(search_fast_path_cost() == PLAN_INF) return;
    uint8_t x = START_X, y = START_Y;
    heading_t h = NORTH;
    cellset_add(path, x, y);
    for(uint16_t i = 0; i < MAZE_STATES; i++){
        action_t a = maze_best_action(cost_a, x, y, h, PLAN_VERIFIED, FAST_COSTS);
        if(a == ACT_NONE) break;
        if(a == ACT_FORWARD){
            x = (uint8_t)(x + heading_dx(h));
            y = (uint8_t)(y + heading_dy(h));
            cellset_add(path, x, y);
        }
        else if(a == ACT_TURN_LEFT) h = heading_left(h);
        else if(a == ACT_TURN_RIGHT) h = heading_right(h);
        else h = heading_back(h);
    }
}

static const char *hwall(uint8_t x, uint8_t y, heading_t side){
    switch(maze_wall(x, y, side)){
        case WALL_PRESENT: return "---";
        case WALL_UNKNOWN: return "...";
        case WALL_ABSENT:  break;
    }
    return "   ";
}

static char vwall(uint8_t x, uint8_t y, heading_t side){
    switch(maze_wall(x, y, side)){
        case WALL_PRESENT: return '|';
        case WALL_UNKNOWN: return ':';
        case WALL_ABSENT:  break;
    }
    return ' ';
}

static char cell_mark(const cellset_t *path, uint8_t x, uint8_t y){
    if(x == pose.x && y == pose.y) return "^>v<"[pose.h];
    if(x == START_X && y == START_Y) return 'S';
    if(maze_is_goal(x, y)) return 'G';
    if(cellset_has(path, x, y)) return '*';
    if(!maze_is_visited(x, y)) return '?';
    return ' ';
}

void search_print_map(void){
    // Draw only what matters: visited cells, the goal, the start and the robot.
    uint8_t goal[4];
    maze_get_goal(goal);
    uint8_t max_x = goal[2] > pose.x ? goal[2] : pose.x;
    uint8_t max_y = goal[3] > pose.y ? goal[3] : pose.y;
    for(uint8_t y = 0; y < MAZE_SIZE; y++){
        for(uint8_t x = 0; x < MAZE_SIZE; x++){
            if(!maze_is_visited(x, y)) continue;
            if(x > max_x) max_x = x;
            if(y > max_y) max_y = y;
        }
    }

    cellset_t path;
    trace_fast_path(&path);
    char line[4 * MAZE_SIZE + 2];
    for(int8_t y = (int8_t)max_y; y >= 0; y--){
        uint8_t n = 0;
        for(uint8_t x = 0; x <= max_x; x++){
            const char *wall = hwall(x, (uint8_t)y, NORTH);
            line[n++] = '+';
            line[n++] = wall[0];
            line[n++] = wall[1];
            line[n++] = wall[2];
        }
        line[n++] = '+';
        line[n] = '\0';
        uart_wait_space(500);
        print("%s\n", line);

        n = 0;
        for(uint8_t x = 0; x <= max_x; x++){
            line[n++] = vwall(x, (uint8_t)y, WEST);
            line[n++] = ' ';
            line[n++] = cell_mark(&path, x, (uint8_t)y);
            line[n++] = ' ';
        }
        line[n++] = vwall(max_x, (uint8_t)y, EAST);
        line[n] = '\0';
        uart_wait_space(500);
        print("%s\n", line);
    }
    uint8_t n = 0;
    for(uint8_t x = 0; x <= max_x; x++){
        const char *wall = hwall(x, 0, SOUTH);
        line[n++] = '+';
        line[n++] = wall[0];
        line[n++] = wall[1];
        line[n++] = wall[2];
    }
    line[n++] = '+';
    line[n] = '\0';
    uart_wait_space(500);
    print("%s\n", line);
    uart_wait_space(500);
    print("S salida G meta * camino rapido ? sin visitar ^>v< robot  ...: pared dudosa\n");
}
