#ifndef _PROJECTS_PROJECT1_AW_MANAGER_H__
#define _PROJECTS_PROJECT1_AW_MANAGER_H__

#include <stdio.h>

#include "threads/thread.h"
#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_thread.h"

#define ROW_A 0
#define COL_A 2
#define ROW_B 2
#define COL_B 0
#define ROW_C 5
#define COL_C 2
#define ROW_S 4
#define COL_S 5
#define ROW_W 5
#define COL_W 5

extern unsigned int step;
extern const char thread_status[4][10];
extern const char map_draw_default[6][7];

void init_automated_warehouse(char** argv);

void print_map(struct robot* __robots, int __number_of_robots);

void increase_step();

#endif