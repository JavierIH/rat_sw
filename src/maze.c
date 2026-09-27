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

void maze_erase(void){
    maze_init();
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
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
// costs used here each state is popped about once (1024 pops for the
// empty 16x16). The loop is the search legs' decision time (~5000 pops in
// the worst decision on the way), so it runs on locals with nothing called.
// Each state is queued at most once at a time: the ring never overflows.

_Static_assert((MAZE_STATES & (MAZE_STATES - 1)) == 0, "the queue ring wraps with a mask");

static uint16_t queue[MAZE_STATES];
static uint32_t queued[MAZE_STATES / 32];
static uint16_t q_count;    // seeded by the callers, then the loop's
static uint32_t pops;

// State index step to the neighbour cell on each side.
static const int16_t STEP[4] = {MAZE_SIZE * 4, 4, -MAZE_SIZE * 4, -4};

static void queue_reset(void){
    q_count = 0;
    memset(queued, 0, sizeof(queued));
}

static void queue_push(uint16_t s){
    queued[s >> 5] |= 1u << (s & 31u);
    queue[q_count++] = s;
}

static uint8_t passable(uint8_t x, uint8_t y, heading_t dir, plan_mode_t mode){
    const int8_t *e = evidence_slot(x, y, dir);
    if(!e) return 0;
    return mode == PLAN_VERIFIED ? (*e <= -EV_VERIFIED) : (*e <= 0);
}

static void cost_fill(uint16_t *cost){
    for(uint16_t s = 0; s < MAZE_STATES; s++) cost[s] = PLAN_INF;
}

// Side `dir` of (x, y) open for the mode: evidence <= limit, never the border.
static inline uint8_t open_side(uint8_t x, uint8_t y, heading_t dir, int8_t limit){
    switch(dir){
        case NORTH: return y + 1 < MAZE_SIZE && ev_north[x][y] <= limit;
        case EAST:  return x + 1 < MAZE_SIZE && ev_east[x][y] <= limit;
        case SOUTH: return y > 0 && ev_north[x][y - 1] <= limit;
        default:    return x > 0 && ev_east[x - 1][y] <= limit;
    }
}

// One edge into state t at cost c: queue it if that is cheaper (and it is
// not queued already). Inlined: the loop's queue stays in registers.
static inline __attribute__((always_inline)) void relax(uint16_t *cost, uint32_t t, uint32_t c, uint32_t head,
                                                         uint32_t *count){
    if(c >= cost[t]) return;
    cost[t] = (uint16_t)c;
    const uint32_t bit = 1u << (t & 31u);
    if(queued[t >> 5] & bit) return;
    queued[t >> 5] |= bit;
    queue[(head + *count) & (MAZE_STATES - 1u)] = (uint16_t)t;
    (*count)++;
}

// Pops the queue the callers seeded until it is empty. Each state (x, y, h)
// has three edges: a cell forward (forward: out of its side h; backward: in
// from the cell on its side opposite h, with the same heading) and the two
// quarter turns.
static void spfa(uint16_t *cost, plan_mode_t mode, plan_costs_t costs, uint8_t backward){
    const int8_t limit = mode == PLAN_VERIFIED ? -EV_VERIFIED : 0;
    const uint32_t back = backward ? 2u : 0u, cell = costs.cell, turn = costs.turn;
    uint32_t head = 0, count = q_count, popped = 0;
    while(count){
        const uint32_t s = queue[head];
        head = (head + 1u) & (MAZE_STATES - 1u);
        count--;
        popped++;
        queued[s >> 5] &= ~(1u << (s & 31u));
        const uint32_t c = cost[s];
        const uint32_t side = (s + back) & 3u;
        if(open_side((uint8_t)((s >> 2) % MAZE_SIZE), (uint8_t)((s >> 2) / MAZE_SIZE), (heading_t)side, limit))
            relax(cost, (uint32_t)((int32_t)s + STEP[side]), c + cell, head, &count);
        relax(cost, (s & ~3u) | ((s + 3u) & 3u), c + turn, head, &count);
        relax(cost, (s & ~3u) | ((s + 1u) & 3u), c + turn, head, &count);
    }
    pops += popped;
}

uint32_t maze_plan_pops(void){
    return pops;
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
    spfa(cost, mode, costs, 1);
}

void maze_plan_from(uint8_t x0, uint8_t y0, heading_t h0, plan_mode_t mode, plan_costs_t costs, uint16_t *cost){
    cost_fill(cost);
    queue_reset();
    uint16_t start = maze_state(x0, y0, h0);
    cost[start] = 0;
    queue_push(start);
    spfa(cost, mode, costs, 0);
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
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            out->walls[x][y] = (uint8_t)((ev_north[x][y] + EV_MAX) | ((ev_east[x][y] + EV_MAX) << 4));
        }
    }
    memcpy(out->visited, visited_rows, sizeof(visited_rows));
    memcpy(out->goal, goal, sizeof(goal));
}

uint8_t maze_import(const maze_snapshot_t *in){
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            if((in->walls[x][y] & 0x0Fu) > 2 * EV_MAX || (in->walls[x][y] >> 4) > 2 * EV_MAX) return 0;
        }
    }
    const uint8_t *g = in->goal;
    if(g[0] > g[2] || g[1] > g[3] || g[2] >= MAZE_SIZE || g[3] >= MAZE_SIZE) return 0;

    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            ev_north[x][y] = (int8_t)((in->walls[x][y] & 0x0Fu) - EV_MAX);
            ev_east[x][y] = (int8_t)((in->walls[x][y] >> 4) - EV_MAX);
        }
    }
    memcpy(visited_rows, in->visited, sizeof(visited_rows));
    memcpy(goal, g, sizeof(goal));
    return 1;
}
