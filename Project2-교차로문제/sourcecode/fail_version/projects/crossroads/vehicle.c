#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"
#include "projects/crossroads/crossroads.h"

// 세마포어 선언
struct semaphore move_decision_sema;
struct semaphore waiting_thread_sema;
struct semaphore all_threads_done;

// 전역 변수 
int waiting_threads; // 대기 중인 쓰레드 개수
int position[4]; // {2,2},{2,4},{4,2},{4,4}를 지날 예정인 차가 있나?
int total_threads; // 전체 쓰레드 개수

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
    /* from A */
    {
        /* to A */
        {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from B */
    {
        /* to A */
        {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from C */
    {
        /* to A */
        {{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from D */
    {
        /* to A */
        {{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    }
};

static int is_position_outside(struct position pos)
{
    return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info* vi)
{
    struct position pos_cur, pos_next;

    pos_next = vehicle_path[start][dest][step];
    pos_cur = vi->position;

    if (vi->state == VEHICLE_STATUS_RUNNING) {
        /* check termination */
        if (is_position_outside(pos_next)) {
            /* actual move */
            vi->position.row = vi->position.col = -1;
            /* release previous */
            lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
            return 0;
        }
    }

    /* lock next position */
    lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
    if (vi->state == VEHICLE_STATUS_READY) {
        /* start this vehicle */
        vi->state = VEHICLE_STATUS_RUNNING;
    }
    else {
        /* release current position */
        lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
    }

    if (vi->state == VEHICLE_STATUS_STOP) {
        return -1;
    }


    /* update position */
    vi->position = pos_next;

    return 1;
}

void init_on_mainthread(int thread_cnt) {
    /* Called once before spawning threads */

    sema_init(&move_decision_sema, 1); // 움직일지 말지를 결정하는 세마포어 초기화
    sema_init(&waiting_thread_sema, 1); // waiting_thread 수를 관리하는 세마포어 초기화
    sema_init(&all_threads_done, 0); // 모든 스레드 완료 세마포어 초기화
    waiting_threads = 0; // 대기 중인 스레드 수 초기화
    crossroads_step = 0; // 단위 스텝 초기화
    total_threads = thread_cnt; // 전체 쓰레드 수 초기화

    for (int i = 0; i < 4; i++) { // 위치 초기화
        position[i] = 0;
    }
}

// move_point[i][j][k] 는 i가 시작점이고 j가 도착점인 경우 k번째 모서리를 지나가면 1 아니면 0 
int move_point[4][4][4] = {
    {
        {1,1,1,1},
        {0,0,1,0},
        {0,0,1,1},
        {0,1,1,1}
    },
    {
        {1,1,0,1},
        {1,1,1,1},
        {0,0,0,1},
        {0,1,0,1}
    },
    {
        {1,1,0,0},
        {1,0,1,0},
        {1,1,1,1},
        {0,1,0,0}
    },
    {
        {1,0,0,0},
        {1,0,1,0},
        {1,0,1,1},
        {1,1,1,1}
    }
};

// 진입 결정을 내리는 위치
int decision_point[4][2] = { {1,2},{2,5},{4,1},{5,4} };

// 교차로 자원 반환을 하는 위치
int release_point[4][2] = { {1,4},{2,1},{4,5},{5,2} };

// 교차로 진입 결정을 내리는 함수
void move_decision(struct vehicle_info* vi) {

    // 1이면 교차로 진입 가능 그 외는 불가능.
    int possible_move = 1;

    int start = vi->start - 'A';
    int dest = vi->dest - 'A';

    for (int i = 0; i < 4; i++) {
        if (move_point[start][dest][i] == 1) {
            if (position[i] != 0) {
                possible_move = 0;
                break;
            }
        }
    }

    // 교차로에 진입 가능 여부 업데이트
    if (possible_move != 1) {
        vi->state = VEHICLE_STATUS_STOP;
    }
    else {
        vi->state = VEHICLE_STATUS_RUNNING;
        for (int i = 0; i < 4; i++) { // 교차로 모서리를 점유 
            if (move_point[start][dest][i] == 1) {
                position[i] = 1;
            }
        }
    }
}

// 가지고 있는 교차로 자원을 반환하는 함수
void release_resource(struct vehicle_info* vi) {

    int start = vi->start - 'A';
    int dest = vi->dest - 'A';

    for (int i = 0; i < 4; i++) { // 교차로 모서리를 반환 
        if (move_point[start][dest][i] == 1) {
            position[i] = 0;
        }
    }
}

void vehicle_loop(void* _vi)
{
    int res;
    int start, dest, step;

    struct vehicle_info* vi = _vi;

    start = vi->start - 'A';
    dest = vi->dest - 'A';

    vi->position.row = vi->position.col = -1;
    vi->state = VEHICLE_STATUS_READY;

    step = 0;
    while (1) {
        /* vehicle main code */

        // 교차로 진입 결정을 내려야 하는 위치면 판단을 내림
        for (int i = 0; i < 4; i++) {
            if (vi->position.row == decision_point[i][0] && vi->position.col == decision_point[i][1]) {
                sema_down(&move_decision_sema);
                move_decision(vi);
                sema_up(&move_decision_sema);
            }
        }

        res = try_move(start, dest, step, vi);
        if (res == 1) {
            step++;
        }

        /* termination condition */
        if (res == 0) {
            sema_down(&waiting_thread_sema);
            total_threads -= 1;
            sema_up(&waiting_thread_sema);
            break;
        }

        // 단위 스텝 증가를 위한 로직
        sema_down(&waiting_thread_sema);
        waiting_threads++;
        if (waiting_threads == total_threads) {
            waiting_threads = 0;
            sema_up(&waiting_thread_sema); // waiting_thread_sema를 다시 업
            crossroads_step = crossroads_step + 1;
            unitstep_changed();
            for (int i = 0; i < total_threads - 1; i++) {
                sema_up(&all_threads_done); // 모든 스레드 완료 세마포어 업
            }
        }
        else {
            sema_up(&waiting_thread_sema);
            sema_down(&all_threads_done); // 모든 스레드 완료 대기
        }

        for (int i = 0; i < 4; i++) {
            if (vi->position.row == release_point[i][0] && vi->position.col == release_point[i][1]) {
                sema_down(&move_decision_sema);
                release_resource(vi);
                sema_up(&move_decision_sema);
            }
        }

    }

    /* status transition must happen before sema_up */
    vi->state = VEHICLE_STATUS_FINISHED;
}