#include "maze.h"

uint8_t maze_walls[MAZE_SIZE][MAZE_SIZE];
uint8_t maze_visited[MAZE_SIZE][MAZE_SIZE];
uint16_t maze_flood[MAZE_SIZE][MAZE_SIZE];
static int8_t wall_confidence[MAZE_SIZE][MAZE_SIZE][4];

static int8_t goal_x_min, goal_y_min, goal_x_max, goal_y_max;

robot_heading_t maze_left_of(robot_heading_t heading){
    return (robot_heading_t)((heading + 3) % 4);
}

robot_heading_t maze_right_of(robot_heading_t heading){
    return (robot_heading_t)((heading + 1) % 4);
}

static robot_heading_t opposite_of(robot_heading_t dir){
    return (robot_heading_t)((dir + 2) % 4);
}

static uint8_t neighbor_of(int8_t x, int8_t y, robot_heading_t dir, int8_t *nx, int8_t *ny);

// A wall is shared: setting it on a cell also sets it on the neighbor it faces.
void maze_set_wall(int8_t x, int8_t y, robot_heading_t dir){
    if(x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;
    maze_walls[x][y] |= WALL_BIT(dir);

    int8_t nx = x, ny = y;
    switch(dir){
        case UP_DIR:    ny++; break;
        case RIGHT_DIR: nx++; break;
        case DOWN_DIR:  ny--; break;
        case LEFT_DIR:  nx--; break;
    }
    if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
        maze_walls[nx][ny] |= WALL_BIT(opposite_of(dir));
    }
}

static void maze_clear_wall(int8_t x, int8_t y, robot_heading_t dir){
    int8_t nx, ny;
    if(!neighbor_of(x, y, dir, &nx, &ny)) return;
    maze_walls[x][y] &= (uint8_t)~WALL_BIT(dir);
    maze_walls[nx][ny] &= (uint8_t)~WALL_BIT(opposite_of(dir));
}

void maze_observe_wall(int8_t x, int8_t y, robot_heading_t dir, uint8_t present){
    if(x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;

    int8_t confidence = wall_confidence[x][y][dir];
    if(present){
        if(confidence < 3) confidence++;
        // Symmetric with the clear branch below: a single noisy reading must
        // not be able to weld a wall shut permanently (it previously did,
        // while clearing already required 2 consecutive contradicting reads -
        // that asymmetry let one false-positive IR blip poison the map with a
        // wall that could then never be undone by a later contradicting read).
        if(confidence >= 2) maze_set_wall(x, y, dir);
    }
    else{
        if(confidence > -3) confidence--;
        if(confidence <= -2) maze_clear_wall(x, y, dir);
    }
    wall_confidence[x][y][dir] = confidence;

    int8_t nx, ny;
    if(neighbor_of(x, y, dir, &nx, &ny)){
        wall_confidence[nx][ny][opposite_of(dir)] = confidence;
    }
}

uint8_t maze_has_wall(int8_t x, int8_t y, robot_heading_t dir){
    if(x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return 1; // out of bounds counts as a wall
    return (maze_walls[x][y] & WALL_BIT(dir)) != 0;
}

uint8_t maze_is_goal(int8_t x, int8_t y){
    return x >= goal_x_min && x <= goal_x_max && y >= goal_y_min && y <= goal_y_max;
}

void maze_set_goal(int8_t x_min, int8_t y_min, int8_t x_max, int8_t y_max){
    if(x_min < 0 || y_min < 0 || x_max >= MAZE_SIZE || y_max >= MAZE_SIZE) return;
    if(x_min > x_max || y_min > y_max) return;
    goal_x_min = x_min;
    goal_y_min = y_min;
    goal_x_max = x_max;
    goal_y_max = y_max;
}

static uint8_t neighbor_of(int8_t x, int8_t y, robot_heading_t dir, int8_t *nx, int8_t *ny){
    *nx = x;
    *ny = y;
    switch(dir){
        case UP_DIR:    (*ny)++; break;
        case RIGHT_DIR: (*nx)++; break;
        case DOWN_DIR:  (*ny)--; break;
        case LEFT_DIR:  (*nx)--; break;
    }
    return *nx >= 0 && *nx < MAZE_SIZE && *ny >= 0 && *ny < MAZE_SIZE;
}

// Classic BFS flood fill from every goal cell outward, following only
// passages without a known wall. Cells never reached keep distance 0xFFFF.
void maze_compute_flood(void){
    static int16_t qx[MAZE_SIZE * MAZE_SIZE];
    static int16_t qy[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    for(int x = 0; x < MAZE_SIZE; x++){
        for(int y = 0; y < MAZE_SIZE; y++){
            maze_flood[x][y] = 0xFFFF;
        }
    }

    for(int x = 0; x < MAZE_SIZE; x++){
        for(int y = 0; y < MAZE_SIZE; y++){
            if(maze_is_goal(x, y)){
                maze_flood[x][y] = 0;
                qx[tail] = x;
                qy[tail] = y;
                tail++;
            }
        }
    }

    while(head < tail){
        int8_t x = (int8_t)qx[head];
        int8_t y = (int8_t)qy[head];
        head++;
        uint16_t dist = maze_flood[x][y];

        for(int d = 0; d < 4; d++){
            robot_heading_t dir = (robot_heading_t)d;
            if(maze_has_wall(x, y, dir)) continue;
            int8_t nx, ny;
            if(!neighbor_of(x, y, dir, &nx, &ny)) continue;
            if(maze_flood[nx][ny] > dist + 1){
                maze_flood[nx][ny] = dist + 1;
                qx[tail] = nx;
                qy[tail] = ny;
                tail++;
            }
        }
    }
}

// Among open neighbors, pick the lowest flood value; ties prefer to keep
// going straight (fewer turns).
robot_heading_t maze_next_move(int8_t x, int8_t y, robot_heading_t current_heading){
    robot_heading_t best = current_heading;
    uint16_t best_dist = 0xFFFF;
    uint8_t found = 0;

    for(int d = 0; d < 4; d++){
        robot_heading_t dir = (robot_heading_t)d;
        if(maze_has_wall(x, y, dir)) continue;
        int8_t nx, ny;
        if(!neighbor_of(x, y, dir, &nx, &ny)) continue;

        uint16_t d_val = maze_flood[nx][ny];
        if(!found || d_val < best_dist || (d_val == best_dist && dir == current_heading)){
            best_dist = d_val;
            best = dir;
            found = 1;
        }
    }
    return best;
}

void maze_init(void){
    for(int x = 0; x < MAZE_SIZE; x++){
        for(int y = 0; y < MAZE_SIZE; y++){
            maze_walls[x][y] = 0;
            maze_visited[x][y] = 0;
            for(int d = 0; d < 4; d++) wall_confidence[x][y][d] = 0;
        }
    }
    // Outer border is always a wall, known up front.
    for(int i = 0; i < MAZE_SIZE; i++){
        maze_set_wall(i, 0, DOWN_DIR);
        maze_set_wall(i, MAZE_SIZE - 1, UP_DIR);
        maze_set_wall(0, i, LEFT_DIR);
        maze_set_wall(MAZE_SIZE - 1, i, RIGHT_DIR);
    }
    // Real competition default: center 2x2 block of the 16x16 maze.
    // Override with maze_set_goal() for smaller practice mazes.
    maze_set_goal(MAZE_SIZE / 2 - 1, MAZE_SIZE / 2 - 1, MAZE_SIZE / 2, MAZE_SIZE / 2);
}
