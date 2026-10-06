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
static const char *const PHASE_TAG[3] = {"GOAL", "OPTIM", "RETURN"};
static const char *const ACTION_NAME[5] = {"-", "FORWARD", "LEFT", "RIGHT", "U-TURN"};

static pose_t pose = {START_X, START_Y, NORTH};
static uint8_t ready = 1;               // pose is the start facing north, for real
static uint8_t turned;                  // turned in place since the last straight
static uint16_t cost_a[MAZE_STATES];    // planner buffers
static uint16_t cost_b[MAZE_STATES];
static int8_t route_turn[PATH_MAX_CELLS];   // speed-run route: the curve in each cell (path.h)
static run_path_t route = {route_turn, 0};
static search_mode_t mode = SEARCH_STRAIGHTS;

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

// Senses the three visible walls into the map; doubtful sides are left out (confirmed later from a better pose).
static move_result_t sense_here(wall_sense_t *w, uint8_t sides_recorded){
    move_result_t r = motion_sense_walls(w);
    if(r != MOVE_OK) return r;
    // Stopped at the end of a leg: the sides were recorded on the way in; counting them again doubles a sighting.
    if(sides_recorded){
        w->left = w->left == SEEN_PRESENT ? SEEN_DOUBTFUL : w->left;
        w->right = w->right == SEEN_PRESENT ? SEEN_DOUBTFUL : w->right;
        maze_observe(pose.x, pose.y, pose.h, w->front == SEEN_PRESENT);
        maze_mark_visited(pose.x, pose.y);
        telemetry_cell(pose.x, pose.y, pose.h);
        telemetry_background_row();
        return MOVE_OK;
    }
    // After a turn only "no wall" sides are recorded (stopped, they caught posts: 5 phantoms in 14); not at the start.
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

// MOVE_OK: moved, pose updated. MOVE_BLOCKED: in place, wall noted. Else: position unknown.
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

// Drives `route` without stopping and moves the pose along the cells covered (MOVE_BLOCKED: stopped short facing a wall).
static move_result_t run_route(int16_t speed, int16_t curve_speed){
    uint8_t entered = 0;
    move_result_t r = motion_run_path(&route, speed, curve_speed, &entered);
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
        print("Run stopped at (%u,%u)%c\n", pose.x, pose.y, HEADING_CHAR[pose.h]);
        return RUN_ABORTED;
    }
    print("!! %s: %s at (%u,%u)%c. Position lost: take the robot to the start\n",
          what, move_result_name(r), pose.x, pose.y, HEADING_CHAR[pose.h]);
    motion_indicate(IND_FAIL);
    return RUN_FAILED;
}

static run_result_t fail_plan(const char *why){
    ready = 0;
    print("!! %s at (%u,%u)%c\n", why, pose.x, pose.y, HEADING_CHAR[pose.h]);
    motion_indicate(IND_FAIL);
    return RUN_FAILED;
}

// At rest (a flash write stalls the CPU); LEDs: 3 slow blinks = in flash, 3 fast = only in RAM.
static void save_map(void){
    const storage_save_t r = storage_save(1);
    switch(r){
        case STORAGE_WRITTEN:   print("Map saved (%u cells visited)\n", maze_visited_count()); break;
        case STORAGE_UNCHANGED: print("Map already saved (unchanged)\n"); break;
        case STORAGE_FULL:      print("!! flash full: the map is only in RAM (SAVE compacts and saves it)\n"); break;
        case STORAGE_FAILED:    print("!! could not save the map to flash: it is only in RAM\n"); break;
    }
    motion_indicate(r == STORAGE_WRITTEN || r == STORAGE_UNCHANGED ? IND_DONE : IND_NOT_SAVED);
}

static run_result_t finish_at_start(uint16_t steps){
    move_result_t r = face(NORTH);
    if(r != MOVE_OK) return fail_move(r, "facing north");
    ready = 1;
    print("At the start after %u actions\n", steps);
    uint16_t cost = search_fast_path_cost();
    if(cost != PLAN_INF) print("Fast path verified: cost %u\n", cost);
    return RUN_OK;
}

// ---- Planning helpers ----------------------------------------------------------------

// Plans to `targets`; if phantom walls sealed them off, forgets doubtful walls (not this cell's) instead of giving up, at rest only.
static replan_t plan_explore(const cellset_t *targets, uint8_t *repairs, uint8_t at_rest){
    maze_plan_to(targets, PLAN_OPTIMISTIC, SEARCH_COSTS, cost_a);
    if(cost_a[pose_state()] != PLAN_INF) return REPLAN_OK;
    if(!at_rest || *repairs >= MAP_MAX_RECOVERIES) return REPLAN_UNREACHABLE;
    (*repairs)++;
    uint16_t forgotten = maze_forget_walls(1, pose.x, pose.y);
    maze_plan_to(targets, PLAN_OPTIMISTIC, SEARCH_COSTS, cost_a);
    if(cost_a[pose_state()] == PLAN_INF){
        forgotten = (uint16_t)(forgotten + maze_forget_walls(INT8_MAX, MAZE_SIZE, MAZE_SIZE));
        maze_plan_to(targets, PLAN_OPTIMISTIC, SEARCH_COSTS, cost_a);
    }
    print("!! target unreachable on the map: forgetting %u doubtful walls (repair %u/%u)\n",
          forgotten, *repairs, MAP_MAX_RECOVERIES);
    telemetry_map();    // the forgotten walls can be anywhere: resend the whole map
    return cost_a[pose_state()] != PLAN_INF ? REPLAN_OK : REPLAN_UNREACHABLE;
}

// Unvisited cells on an optimistic optimal path start->goal; none left = the best path is verified optimal.
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

void search_set_mode(search_mode_t m){
    mode = m;
}

search_mode_t search_mode(void){
    return mode;
}

typedef struct {
    phase_t phase;
    uint16_t steps, optimize_steps;
    uint8_t repairs;
    cellset_t targets;
    uint8_t reached;        // cells of the last leg the robot got to (it stopped in the last one)
    uint8_t failed;         // planning failed on the way: handled at rest
} explore_t;

typedef enum { PHASE_GO, PHASE_STOP, PHASE_DONE } phase_step_t;

// Phase changes at the robot's cell and the phase's targets; the goal is saved at rest, so on the way it only asks for a stop.
static phase_step_t explore_phase(explore_t *e, uint8_t stopped){
    if(e->phase == PH_TO_GOAL && maze_is_goal(pose.x, pose.y)){
        if(!stopped) return PHASE_STOP;
        print("Goal reached at (%u,%u) after %u actions\n", pose.x, pose.y, e->steps);
        // Saved here too: a failed way back loses nothing.
        save_map();
        e->phase = PH_OPTIMIZE;
        telemetry_activity(TM_OPTIMIZE);
    }
    if(e->phase == PH_OPTIMIZE){
        uint8_t budget_left = e->optimize_steps < OPTIMIZE_MAX_STEPS;
        if(!budget_left || !optimize_candidates(&e->targets)){
            print(budget_left ? "Optimal fast path verified: returning to the start\n"
                              : "Optimization budget spent: returning to the start\n");
            e->phase = PH_TO_START;
            telemetry_activity(TM_TO_START);
        }
    }
    if(e->phase == PH_TO_START && pose.x == START_X && pose.y == START_Y) return stopped ? PHASE_DONE : PHASE_STOP;
    if(e->phase == PH_TO_GOAL) maze_goal_cells(&e->targets);
    else if(e->phase == PH_TO_START) start_cell(&e->targets);
    return PHASE_GO;
}

// Best action from the pose, logged with the walls seen; 0 if unreachable or out of budget (e->failed says which).
static uint8_t explore_plan(explore_t *e, const wall_sense_t *w, action_t *a, uint8_t at_rest){
    if(plan_explore(&e->targets, &e->repairs, at_rest) == REPLAN_UNREACHABLE){
        e->failed = 1;
        return 0;
    }
    *a = maze_best_action(cost_a, pose.x, pose.y, pose.h, PLAN_OPTIMISTIC, SEARCH_COSTS);
    if(params.log_level >= 1){
        print("%s (%u,%u)%c F%c L%c R%c cost=%u -> %s\n", PHASE_TAG[e->phase], pose.x, pose.y,
              HEADING_CHAR[pose.h], SIGHTING_CHAR[w->front], SIGHTING_CHAR[w->left], SIGHTING_CHAR[w->right],
              cost_a[pose_state()], ACTION_NAME[*a]);
    }
    if(++e->steps > SEARCH_MAX_STEPS){
        e->failed = 2;
        return 0;
    }
    if(e->phase == PH_OPTIMIZE) e->optimize_steps++;
    return 1;
}

// ---- Legs: straight on through the cells, each decided on the way -----------------

static uint8_t leg_decided;             // cells decided in the current leg

// Getting to the next cell of a leg, walls in view: move the pose and record them (every leg's `decide` starts here).
static void leg_enter_cell(const wall_sense_t *w){
    maze_mark_crossed(pose.x, pose.y, pose.h);
    pose.x = (uint8_t)(pose.x + heading_dx(pose.h));
    pose.y = (uint8_t)(pose.y + heading_dy(pose.h));
    if(w->front != SEEN_DOUBTFUL) maze_observe(pose.x, pose.y, pose.h, w->front == SEEN_PRESENT);
    if(w->left != SEEN_DOUBTFUL) maze_observe(pose.x, pose.y, heading_left(pose.h), w->left == SEEN_PRESENT);
    if(w->right != SEEN_DOUBTFUL) maze_observe(pose.x, pose.y, heading_right(pose.h), w->right == SEEN_PRESENT);
    maze_mark_visited(pose.x, pose.y);
    leg_decided++;
    telemetry_cell(pose.x, pose.y, pose.h);
    telemetry_pose(pose.x, pose.y, pose.h);
    telemetry_background_row();
}

// Forward from rest, deciding every cell on the way until `decide` stops; the pose follows the cells reached.
static move_result_t drive_leg(next_cell_fn decide, void *ctx, uint8_t *reached){
    const pose_t start = pose;
    const int16_t speed = params.search_speed < SEARCH_LEG_SPEED_MAX ? params.search_speed : SEARCH_LEG_SPEED_MAX;
    uint8_t entered = 0;
    leg_decided = 0;
    move_result_t r = motion_explore(speed, decide, ctx, &entered);
    turned = 0;
    *reached = entered;
    if(entered < leg_decided){
        // Stopped short (an obstacle: backed up to a cell centre).
        pose = start;
        pose.x = (uint8_t)(pose.x + entered * heading_dx(pose.h));
        pose.y = (uint8_t)(pose.y + entered * heading_dy(pose.h));
    }
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

// The search's leg: record each cell, then decide; anything done at rest is a stop there.
static next_move_t explore_next(const wall_sense_t *w, void *ctx){
    explore_t *e = ctx;
    leg_enter_cell(w);
    if(explore_phase(e, 0) != PHASE_GO) return NEXT_STOP;
    action_t a;
    if(!explore_plan(e, w, &a, 0)) return NEXT_STOP;
    if(a == ACT_FORWARD && w->front == SEEN_ABSENT) return NEXT_STRAIGHT;
    // A stop here decides again at rest: one action, not two (the budget counts them).
    e->steps--;
    if(e->phase == PH_OPTIMIZE) e->optimize_steps--;
    return NEXT_STOP;
}

run_result_t search_explore(void){
    explore_t e = {.phase = PH_TO_GOAL};
    uint8_t sides_recorded = 0;     // this cell's sides were read on the way in, at the end of a leg

    pose_reset();
    ready = 0;
    telemetry_activity(TM_TO_GOAL);
    print("== SEARCH ==\n");
    for(;;){
        if(!motion_checkpoint()) return fail_move(MOVE_ABORTED, "search");
        wall_sense_t w;
        move_result_t r = sense_here(&w, sides_recorded);
        sides_recorded = 0;
        if(r != MOVE_OK) return fail_move(r, "sensing");
        if(explore_phase(&e, 1) == PHASE_DONE){
            const run_result_t res = finish_at_start(e.steps);
            if(res == RUN_OK) save_map();   // if the return learned something
            return res;
        }
        action_t a;
        if(!explore_plan(&e, &w, &a, 1)){
            return fail_plan(e.failed == 2 ? "action budget spent" : "target unreachable");
        }
        // Never drive into a passage the sensors see closed: the sighting raised its evidence, sensing converges.
        if(a == ACT_FORWARD && w.front == SEEN_PRESENT) continue;
        if(a == ACT_FORWARD && mode != SEARCH_STOP_EACH){
            r = drive_leg(explore_next, &e, &e.reached);
            if(r != MOVE_OK && r != MOVE_BLOCKED) return fail_move(r, "move");
            sides_recorded = e.reached > 0;
            continue;
        }
        r = do_action(a);
        if(r != MOVE_OK && r != MOVE_BLOCKED) return fail_move(r, "move");
    }
}

// The route as the log shows it: cells, then R/L for a curve right/left in the last one ("2R1L3"), '+' if cut.
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
        if(route_turn[i]) text[n++] = route_turn[i] > 0 ? 'R' : 'L';
        run = 0;
    }
    text[n] = '\0';
    return text;
}

// Drives the verified route to `targets` in one go, replanned at every stop (explores if none); curve_speed 0: turns in place.
static run_result_t drive_to(const cellset_t *targets, int16_t speed, int16_t curve_speed, const char *tag,
                             uint16_t *steps){
    uint8_t repairs = 0;
    while(!cellset_has(targets, pose.x, pose.y)){
        if(!motion_checkpoint()) return fail_move(MOVE_ABORTED, tag);
        if(++*steps > SEARCH_MAX_STEPS) return fail_plan("action budget spent");
        wall_sense_t w;
        move_result_t r = sense_here(&w, 0);
        if(r != MOVE_OK) return fail_move(r, "sensing");

        maze_plan_to(targets, PLAN_VERIFIED, FAST_COSTS, cost_a);
        int8_t turn;
        if(maze_route(cost_a, pose.x, pose.y, pose.h, PLAN_VERIFIED, FAST_COSTS, &turn, route_turn, PATH_MAX_CELLS,
                      &route.cells)){
            if(turn == 0 && w.front == SEEN_PRESENT) continue;
            for(uint8_t i = 0; !curve_speed && i < route.cells; i++){
                if(!route_turn[i]) continue;
                route_turn[i] = 0;
                route.cells = (uint8_t)(i + 1u);
            }
            if(params.log_level >= 1){
                char text[ROUTE_TEXT_MAX + 2];
                print("%s (%u,%u)%c turn %d + route %s (%u cells)\n", tag, pose.x, pose.y, HEADING_CHAR[pose.h], turn,
                      route_text(text), route.cells);
            }
            r = turn_by(turn);
            if(r == MOVE_OK && route.cells) r = run_route(speed, curve_speed);
        }
        else{
            if(plan_explore(targets, &repairs, 1) == REPLAN_UNREACHABLE) return fail_plan("target unreachable");
            action_t a = maze_best_action(cost_a, pose.x, pose.y, pose.h, PLAN_OPTIMISTIC, SEARCH_COSTS);
            if(params.log_level >= 1){
                print("%s (%u,%u)%c no verified path -> %s\n", tag, pose.x, pose.y,
                      HEADING_CHAR[pose.h], ACTION_NAME[a]);
            }
            if(a == ACT_FORWARD && w.front == SEEN_PRESENT) continue;
            r = do_action(a);
        }
        if(r != MOVE_OK && r != MOVE_BLOCKED) return fail_move(r, tag);
    }
    return RUN_OK;
}

run_result_t search_fast_run(uint8_t curves){
    cellset_t goal, home;
    maze_goal_cells(&goal);
    start_cell(&home);
    pose_reset();

    uint16_t cost = search_fast_path_cost();
    if(cost == PLAN_INF){
        print("No verified start-goal path: run a search first (mode 1)\n");
        return RUN_FAILED;  // nothing moved: still ready
    }
    ready = 0;
    telemetry_activity(TM_FAST);
    print("== SPEED RUN (cost %u) ==\n", cost);
    uint16_t steps = 0;
    run_result_t res = drive_to(&goal, params.fast_speed, curves ? params.curve_speed : 0, "FAST", &steps);
    if(res != RUN_OK) return res;
    print("Goal reached in the speed run after %u legs\n", steps);
    // Saved here, never back at the start: a failed return loses nothing.
    save_map();
    telemetry_activity(TM_RETURN);
    // Not timed, and the map is saved: no faster curves than the safe race's.
    const int16_t back = params.curve_speed < FAST_SAFE_CURVE ? params.curve_speed : FAST_SAFE_CURVE;
    res = drive_to(&home, params.search_speed, curves ? back : 0, "RETURN", &steps);
    if(res != RUN_OK) return res;
    return finish_at_start(steps);
}

typedef struct {
    uint8_t left_hand;
    uint8_t goal;       // 1 once it passed through the goal, 2 once that was reported
    uint32_t steps;
} follow_t;

// The side of the hand the robot follows, from its heading.
static heading_t follow_near_side(const follow_t *f){
    return f->left_hand ? heading_left(pose.h) : heading_right(pose.h);
}

// The follower's leg: straight on while its wall goes on and the way is open; anything else is a stop there.
static next_move_t follow_next(const wall_sense_t *w, void *ctx){
    follow_t *f = ctx;
    leg_enter_cell(w);
    if(!f->goal && maze_is_goal(pose.x, pose.y)) f->goal = 1;
    f->steps++;
    if(maze_wall(pose.x, pose.y, follow_near_side(f)) == WALL_PRESENT
       && maze_wall(pose.x, pose.y, pose.h) != WALL_PRESENT && w->front == SEEN_ABSENT) return NEXT_STRAIGHT;
    f->steps--;     // decided again at rest: one action, not two
    return NEXT_STOP;
}

run_result_t search_wall_follow(uint8_t left_hand){
    const int8_t near_turn = left_hand ? -1 : 1;
    follow_t f = {.left_hand = left_hand};
    uint8_t sides_recorded = 0;     // this cell's sides were read on the way in, at the end of a leg

    pose_reset();
    ready = 0;
    telemetry_activity(TM_FOLLOW);
    print("== WALL FOLLOWER %s ==\n", left_hand ? "LEFT" : "RIGHT");
    // It never ends on its own: past the goal it follows the wall until STOP.
    for(;;){
        if(!f.goal && maze_is_goal(pose.x, pose.y)) f.goal = 1;
        if(f.goal == 1){
            print("Goal reached (follower) at (%u,%u) after %lu actions: going on until STOP\n", pose.x, pose.y,
                  (unsigned long)f.steps);
            motion_indicate(IND_GOAL);
            f.goal = 2;
        }
        if(!motion_checkpoint()) return fail_move(MOVE_ABORTED, "follower");
        f.steps++;
        wall_sense_t w;
        move_result_t r = sense_here(&w, sides_recorded);
        sides_recorded = 0;
        if(r != MOVE_OK) return fail_move(r, "sensing");

        // Decide on the map, which now holds this sighting plus the border.
        heading_t near_side = follow_near_side(&f);
        heading_t far_side = heading_back(near_side);
        int8_t q;
        if(maze_wall(pose.x, pose.y, near_side) != WALL_PRESENT) q = near_turn;
        else if(maze_wall(pose.x, pose.y, pose.h) != WALL_PRESENT) q = 0;
        else if(maze_wall(pose.x, pose.y, far_side) != WALL_PRESENT) q = (int8_t)-near_turn;
        else q = 2;

        r = turn_by(q);
        if(r == MOVE_OK){
            if(mode != SEARCH_STOP_EACH){
                uint8_t reached = 0;
                r = drive_leg(follow_next, &f, &reached);
                sides_recorded = reached > 0;
            }
            else{
                r = forward(1, params.search_speed);
            }
        }
        if(r != MOVE_OK && r != MOVE_BLOCKED) return fail_move(r, "move");
    }
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
    print("S start G goal * fast path ? unvisited ^>v< robot  ...: doubtful wall\n");
}
