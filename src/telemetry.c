#include "telemetry.h"
#include "params.h"
#include "uart.h"

static const char HEX[] = "0123456789ABCDEF";
static const char HEADING[] = "NESW";
static const char WALL_CHAR[4] = {'?', '#', '.', 'o'};
static const char CELL_CODE[] = "0123456789ABCDEFGHIJKLMNOPQRSTUV";

static uint8_t next_row;

// 0 unknown, 1 wall, 2 open (seen once), 3 open (verified).
static uint8_t wall_code(uint8_t x, uint8_t y, heading_t dir){
    int8_t e = maze_evidence(x, y, dir);
    if(e > 0) return 1;
    if(e == 0) return 0;
    return e == -1 ? 2 : 3;
}

// Rows the monitor needs: up to the goal or the highest visited cell.
static uint8_t rows_in_use(void){
    uint8_t goal[4];
    maze_get_goal(goal);
    uint8_t top = goal[3];
    for(uint8_t y = (uint8_t)(top + 1); y < MAZE_SIZE; y++){
        for(uint8_t x = 0; x < MAZE_SIZE; x++){
            if(maze_is_visited(x, y)){
                top = y;
                break;
            }
        }
    }
    return (uint8_t)(top + 1);
}

static void send_row(uint8_t y){
    char row[MAZE_SIZE + 1];
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        uint8_t code = (uint8_t)(wall_code(x, y, NORTH) | (wall_code(x, y, EAST) << 2)
                                 | (maze_is_visited(x, y) << 4));
        row[x] = CELL_CODE[code];
    }
    row[MAZE_SIZE] = '\0';
    print("@R%c%s\n", HEX[y & 15u], row);
}

static void send_goal(void){
    uint8_t g[4];
    maze_get_goal(g);
    print("@G%c%c%c%c\n", HEX[g[0] & 15u], HEX[g[1] & 15u], HEX[g[2] & 15u], HEX[g[3] & 15u]);
}

void telemetry_mode(uint8_t mode){
    if(params.telemetry) print("@M%u\n", mode);
}

void telemetry_activity(telemetry_activity_t activity){
    if(params.telemetry) print("@A%c\n", (char)activity);
}

void telemetry_pose(uint8_t x, uint8_t y, heading_t h){
    if(params.telemetry) print("@P%c%c%c\n", HEX[x & 15u], HEX[y & 15u], HEADING[h & 3u]);
}

void telemetry_cell(uint8_t x, uint8_t y, heading_t h){
    if(!params.telemetry) return;
    print("@C%c%c%c%c%c%c%c\n", HEX[x & 15u], HEX[y & 15u], HEADING[h & 3u],
          WALL_CHAR[wall_code(x, y, NORTH)], WALL_CHAR[wall_code(x, y, EAST)],
          WALL_CHAR[wall_code(x, y, SOUTH)], WALL_CHAR[wall_code(x, y, WEST)]);
}

void telemetry_background_row(void){
    if(!params.telemetry) return;
    if(next_row >= rows_in_use()) next_row = 0;
    send_row(next_row++);
}

void telemetry_map(void){
    if(!params.telemetry) return;
    uint8_t rows = rows_in_use();
    for(uint8_t row = 0; row < rows; row++){
        uart_wait_space(500);
        send_row(row);
    }
}

void telemetry_sync(uint8_t mode, telemetry_activity_t activity, uint8_t x, uint8_t y, heading_t h){
    if(!params.telemetry) return;
    uart_wait_space(500);
    print("@Y%c\n", HEX[(rows_in_use() - 1u) & 15u]);
    uart_wait_space(500);
    send_goal();
    uart_wait_space(500);
    telemetry_mode(mode);
    uart_wait_space(500);
    telemetry_activity(activity);
    uart_wait_space(500);
    telemetry_pose(x, y, h);
    telemetry_map();
}
