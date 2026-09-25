#include "maze.h"
#include <string.h>
#include "robot_config.h"

#define EV_MAX          3   // evidence saturates here, both signs
#define EV_VERIFIED     2   // evidence <= -EV_VERIFIED: open for speed runs
#define EV_BLOCKED      2   // evidence given to a passage the robot bumped into

// One slot per interior wall, shared by the two cells it separates, so both
// sides can never disagree.
static int8_t ev_north[MAZE_SIZE][MAZE_SIZE];   // wall between (x, y) and (x, y + 1)
static int8_t ev_east[MAZE_SIZE][MAZE_SIZE];    // wall between (x, y) and (x + 1, y)
static uint16_t visited_rows[MAZE_SIZE];
static uint8_t goal[4] = {GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1};

// Evidence slot of the wall on side `dir` of (x, y); NULL for the border.
static int8_t *evidence_slot(uint8_t x, uint8_t y, heading_t dir){
    if(x >= MAZE_SIZE || y >= MAZE_SIZE) return NULL;
    switch(dir){
        case NORTH: return (y + 1 < MAZE_SIZE) ? &ev_north[x][y] : NULL;
        case EAST:  return (x + 1 < MAZE_SIZE) ? &ev_east[x][y] : NULL;
        case SOUTH: return (y > 0) ? &ev_north[x][y - 1] : NULL;
        case WEST:  return (x > 0) ? &ev_east[x - 1][y] : NULL;
    }
    return NULL;
}

void maze_init(void){
    memset(ev_north, 0, sizeof(ev_north));
    memset(ev_east, 0, sizeof(ev_east));
    memset(visited_rows, 0, sizeof(visited_rows));
}

void maze_observe(uint8_t x, uint8_t y, heading_t dir, uint8_t present){
    int8_t *e = evidence_slot(x, y, dir);
    if(!e) return;
    if(present){
        if(*e < EV_MAX) (*e)++;
    }
    else if(*e > -EV_MAX){
        (*e)--;
    }
}

void maze_mark_crossed(uint8_t x, uint8_t y, heading_t dir){
    int8_t *e = evidence_slot(x, y, dir);
    if(e) *e = -EV_MAX;
}

void maze_mark_blocked(uint8_t x, uint8_t y, heading_t dir){
    int8_t *e = evidence_slot(x, y, dir);
    if(!e) return;
    int8_t v = (int8_t)((*e > 0 ? *e : 0) + EV_BLOCKED);
    *e = v > EV_MAX ? EV_MAX : v;
}

int8_t maze_evidence(uint8_t x, uint8_t y, heading_t dir){
    const int8_t *e = evidence_slot(x, y, dir);
    return e ? *e : EV_MAX;
}

wall_state_t maze_wall(uint8_t x, uint8_t y, heading_t dir){
    int8_t e = maze_evidence(x, y, dir);
    if(e > 0) return WALL_PRESENT;
    if(e < 0) return WALL_ABSENT;
    return WALL_UNKNOWN;
}

uint16_t maze_forget_walls(int8_t max_evidence){
    uint16_t count = 0;
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            if(ev_north[x][y] > 0 && ev_north[x][y] <= max_evidence){ ev_north[x][y] = 0; count++; }
            if(ev_east[x][y] > 0 && ev_east[x][y] <= max_evidence){ ev_east[x][y] = 0; count++; }
        }
    }
    return count;
}

void maze_mark_visited(uint8_t x, uint8_t y){
    if(x < MAZE_SIZE && y < MAZE_SIZE) visited_rows[y] = (uint16_t)(visited_rows[y] | (1u << x));
}

uint8_t maze_is_visited(uint8_t x, uint8_t y){
    return x < MAZE_SIZE && y < MAZE_SIZE && ((visited_rows[y] >> x) & 1u);
}

uint16_t maze_visited_count(void){
    uint16_t count = 0;
    for(uint8_t y = 0; y < MAZE_SIZE; y++){
        for(uint16_t row = visited_rows[y]; row; row &= (uint16_t)(row - 1)) count++;
    }
    return count;
}

uint8_t maze_set_goal(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1){
    if(x0 > x1 || y0 > y1 || x1 >= MAZE_SIZE || y1 >= MAZE_SIZE) return 0;
    goal[0] = x0; goal[1] = y0; goal[2] = x1; goal[3] = y1;
    return 1;
}

void maze_get_goal(uint8_t out[4]){
    memcpy(out, goal, sizeof(goal));
}

uint8_t maze_is_goal(uint8_t x, uint8_t y){
    return x >= goal[0] && x <= goal[2] && y >= goal[1] && y <= goal[3];
}

void maze_goal_cells(cellset_t *out){
    cellset_clear(out);
    for(uint8_t y = goal[1]; y <= goal[3]; y++){
        for(uint8_t x = goal[0]; x <= goal[2]; x++) cellset_add(out, x, y);
    }
}

// ---- Planner ------------------------------------------------------------------------
// Label-correcting shortest paths (SPFA): with the small positive integer
// costs used here each state is relaxed only a few times, so a full 16x16
// plan takes on the order of a millisecond. Each state is queued at most
// once at a time, so the ring buffer never overflows.

static uint16_t queue[MAZE_STATES];
static uint32_t queued[MAZE_STATES / 32];
static uint16_t q_head, q_count;

static void queue_reset(void){
    q_head = 0;
    q_count = 0;
    memset(queued, 0, sizeof(queued));
}

static void queue_push(uint16_t s){
    uint32_t bit = 1u << (s & 31u);
    if(queued[s >> 5] & bit) return;
    queued[s >> 5] |= bit;
    queue[(q_head + q_count) % MAZE_STATES] = s;
    q_count++;
}

static uint16_t queue_pop(void){
    uint16_t s = queue[q_head];
    q_head = (uint16_t)((q_head + 1) % MAZE_STATES);
    q_count--;
    queued[s >> 5] &= ~(1u << (s & 31u));
    return s;
}

static void relax(uint16_t *cost, uint16_t s, uint32_t c){
    if(c < cost[s]){
        cost[s] = (uint16_t)c;
        queue_push(s);
    }
}

static uint8_t passable(uint8_t x, uint8_t y, heading_t dir, plan_mode_t mode){
    const int8_t *e = evidence_slot(x, y, dir);
    if(!e) return 0;
    return mode == PLAN_VERIFIED ? (*e <= -EV_VERIFIED) : (*e <= 0);
}

static void cost_fill(uint16_t *cost){
    for(uint16_t s = 0; s < MAZE_STATES; s++) cost[s] = PLAN_INF;
}

void maze_plan_to(const cellset_t *targets, plan_mode_t mode, plan_costs_t costs, uint16_t *cost){
    cost_fill(cost);
    queue_reset();
    for(uint8_t y = 0; y < MAZE_SIZE; y++){
        for(uint8_t x = 0; x < MAZE_SIZE; x++){
            if(!cellset_has(targets, x, y)) continue;
            for(uint8_t h = 0; h < 4; h++){
                uint16_t s = maze_state(x, y, (heading_t)h);
                cost[s] = 0;
                queue_push(s);
            }
        }
    }
    // Walk the edges backwards: which states lead into the popped one?
    while(q_count){
        uint16_t s = queue_pop();
        uint32_t c = cost[s];
        heading_t h = (heading_t)(s & 3u);
        uint8_t x = (uint8_t)((s >> 2) % MAZE_SIZE);
        uint8_t y = (uint8_t)((s >> 2) / MAZE_SIZE);
        // Driving forward into (x, y) with heading h, from the cell behind.
        if(passable(x, y, heading_back(h), mode)){
            uint8_t px = (uint8_t)(x - heading_dx(h));
            uint8_t py = (uint8_t)(y - heading_dy(h));
            relax(cost, maze_state(px, py, h), c + costs.cell);
        }
        // Turning in place into heading h.
        relax(cost, maze_state(x, y, heading_left(h)), c + costs.turn);
        relax(cost, maze_state(x, y, heading_right(h)), c + costs.turn);
    }
}

void maze_plan_from(uint8_t x0, uint8_t y0, heading_t h0, plan_mode_t mode, plan_costs_t costs, uint16_t *cost){
    cost_fill(cost);
    queue_reset();
    uint16_t start = maze_state(x0, y0, h0);
    cost[start] = 0;
    queue_push(start);
    while(q_count){
        uint16_t s = queue_pop();
        uint32_t c = cost[s];
        heading_t h = (heading_t)(s & 3u);
        uint8_t x = (uint8_t)((s >> 2) % MAZE_SIZE);
        uint8_t y = (uint8_t)((s >> 2) / MAZE_SIZE);
        if(passable(x, y, h, mode)){
            uint8_t nx = (uint8_t)(x + heading_dx(h));
            uint8_t ny = (uint8_t)(y + heading_dy(h));
            relax(cost, maze_state(nx, ny, h), c + costs.cell);
        }
        relax(cost, maze_state(x, y, heading_left(h)), c + costs.turn);
        relax(cost, maze_state(x, y, heading_right(h)), c + costs.turn);
    }
}

action_t maze_best_action(const uint16_t *cost, uint8_t x, uint8_t y, heading_t h,
                          plan_mode_t mode, plan_costs_t costs){
    uint16_t here = cost[maze_state(x, y, h)];
    if(here == 0 || here == PLAN_INF) return ACT_NONE;

    // Candidates in tie-break order: keep going straight, then turn around in
    // one go (only ties when a single turn would be followed by another one),
    // then right, then left.
    uint32_t best = PLAN_INF;
    action_t action = ACT_NONE;
    if(passable(x, y, h, mode)){
        uint8_t nx = (uint8_t)(x + heading_dx(h));
        uint8_t ny = (uint8_t)(y + heading_dy(h));
        uint32_t c = (uint32_t)costs.cell + cost[maze_state(nx, ny, h)];
        if(c < best){ best = c; action = ACT_FORWARD; }
    }
    uint32_t around = 2u * costs.turn + cost[maze_state(x, y, heading_back(h))];
    if(around < best){ best = around; action = ACT_TURN_AROUND; }
    uint32_t right = (uint32_t)costs.turn + cost[maze_state(x, y, heading_right(h))];
    if(right < best){ best = right; action = ACT_TURN_RIGHT; }
    uint32_t left = (uint32_t)costs.turn + cost[maze_state(x, y, heading_left(h))];
    if(left < best){ best = left; action = ACT_TURN_LEFT; }
    return best < PLAN_INF ? action : ACT_NONE;
}

uint8_t maze_route(const uint16_t *cost, uint8_t x, uint8_t y, heading_t h, plan_mode_t mode, plan_costs_t costs,
                   int8_t *turn, int8_t *turns, uint8_t max, uint8_t *cells){
    *turn = 0;
    *cells = 0;
    switch(maze_best_action(cost, x, y, h, mode, costs)){
        case ACT_NONE:        return 0;
        case ACT_FORWARD:     break;
        case ACT_TURN_LEFT:   *turn = -1; h = heading_left(h); break;
        case ACT_TURN_RIGHT:  *turn = 1;  h = heading_right(h); break;
        case ACT_TURN_AROUND: *turn = 2;  h = heading_back(h); break;
    }
    // Past the first cell every turn of an optimal path falls between two
    // cells (turning twice in one cell would be a detour): a turn inside the
    // cell just entered.
    while(*cells < max){
        action_t a = maze_best_action(cost, x, y, h, mode, costs);
        if(a == ACT_FORWARD){
            x = (uint8_t)(x + heading_dx(h));
            y = (uint8_t)(y + heading_dy(h));
            turns[(*cells)++] = 0;
        }
        else if((a == ACT_TURN_LEFT || a == ACT_TURN_RIGHT) && *cells && !turns[*cells - 1]){
            turns[*cells - 1] = a == ACT_TURN_RIGHT ? 1 : -1;
            h = a == ACT_TURN_RIGHT ? heading_right(h) : heading_left(h);
        }
        else{
            break;
        }
    }
    if(*cells) turns[*cells - 1] = 0;   // cut short by `max`: stop at that cell's centre
    return 1;
}

// ---- Persistence -----------------------------------------------------------------------

void maze_export(maze_snapshot_t *out){
    memcpy(out->north, ev_north, sizeof(ev_north));
    memcpy(out->east, ev_east, sizeof(ev_east));
    memcpy(out->visited, visited_rows, sizeof(visited_rows));
    memcpy(out->goal, goal, sizeof(goal));
}

uint8_t maze_import(const maze_snapshot_t *in){
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            if(in->north[x][y] < -EV_MAX || in->north[x][y] > EV_MAX) return 0;
            if(in->east[x][y] < -EV_MAX || in->east[x][y] > EV_MAX) return 0;
        }
    }
    const uint8_t *g = in->goal;
    if(g[0] > g[2] || g[1] > g[3] || g[2] >= MAZE_SIZE || g[3] >= MAZE_SIZE) return 0;

    memcpy(ev_north, in->north, sizeof(ev_north));
    memcpy(ev_east, in->east, sizeof(ev_east));
    memcpy(visited_rows, in->visited, sizeof(visited_rows));
    memcpy(goal, g, sizeof(goal));
    return 1;
}
