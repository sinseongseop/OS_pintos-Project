#include "projects/automated_warehouse/aw_manager.h"

// #####################################################################
// DO NOT TOUCH THIS CODE
// #####################################################################

unsigned int step = 0;
const char thread_status[4][10] = {
    "RUNNING",
    "READY",
    "BLOCKED",
    "DYING"
};

const char map_draw_default[6][7] = {
    {'X', 'X', 'A', 'X', 'X', 'X', 'X' },
    {'X', '1', ' ', '2', '3', '4', 'X' },
    {'B', ' ', ' ', ' ', ' ', ' ', 'X' },
    {'X', ' ', ' ', ' ', ' ', ' ', 'X' },
    {'X', '5', ' ', '6', '7', 'S', 'X' },
    {'X', 'X', 'C', 'X', 'X', 'W', 'X' }
};

/**
 * Before starting the simulator, initialize essential component
 * It must be called top of the main code
 */
void init_automated_warehouse(char** __argv){
    printf("arguments list:%s, %s, %s\n", __argv[0], __argv[1], __argv[2]);
    list_init(&blocked_threads);
}

/**
 * Private code for printing
 */
void _print_place(struct robot* __robots, int __number_of_robots, int __row, int __col){
    for (int robotIdx = 0; robotIdx < __number_of_robots; robotIdx++){
        struct robot* __robot = &__robots[robotIdx];
        if (__robot->row == __row && __robot->col == __col) printf("%sM%d,",__robot->name, __robot->current_payload);
    }
}
/**
 * Print a map of warehouse and robots
 * It requires array of robots and length of array
 * It must be called before unblocking robot threads
 */
void print_map(struct robot* __robots, int __number_of_robots){    
    printf("STEP_INFO_START::%d\n", step);
    printf("MAP_INFO::\n");
    for (int row = 0; row < 6; row++){
        for (int col = 0; col < 6; col++){
            if (map_draw_default[row][col] == 'A' ||
                map_draw_default[row][col] == 'B' ||
                map_draw_default[row][col] == 'C' ||
                map_draw_default[row][col] == 'W'){
                printf("%c    ", map_draw_default[row][col]);
            }
            else {
                int isFound = 0;
                for (int robotIdx = 0; robotIdx < __number_of_robots; robotIdx++){
                    struct robot* __robot = &__robots[robotIdx];
                    if (__robot->col == col && __robot->row == row){
                        if (__robot->current_payload > 0) printf("%sM%d ", __robot->name, __robot->current_payload);
                        else printf("%s   ", __robot->name);
                        isFound = 1;
                    }
                }
                if (!isFound) printf("%c    ", map_draw_default[row][col]);
            }
        }
        printf("\n");
    }

    printf("\n");
    printf("PLACE_INFO::\n");
    printf("W:");
    _print_place(__robots, __number_of_robots, ROW_W, COL_W);
    printf("\n");

    printf("A:");
    _print_place(__robots, __number_of_robots, ROW_A, COL_A);
    printf("\n");

    printf("B:");
    _print_place(__robots, __number_of_robots, ROW_B, COL_B);
    printf("\n");

    printf("C:");
    _print_place(__robots, __number_of_robots, ROW_C, COL_C);
    printf("\n");
    printf("STEP_INFO_DONE::%d\n", step);
}

/**
 * A function increasing 1 step
 * It must be called before unblocking and after print_map
 */
void increase_step(){
    step++;
}