#include <stdio.h>
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"


// Lock & condition
struct lock lock_step; // step의 일관성을 보장하기 위한 Lock
struct lock lock_critical; // 임계영역에 들어가는 자동차의 수의 일관성을 위한 Lock
struct condition waiting_thread; //wating 중인 thread들의 집합

// 전역 변수
int total_threads;  // 총 쓰레드(총 자동차)의 개수
int move_car_cnt; // try_move가 끝난 자동차의 총개수
int cnt_critical; // 임계영역에 들어가있는 자동차의 개수

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


/* 0 : termination, 1 : success, -1 : fail */
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

    // 다음 위치의 Lock을 획득하려고 시도
    if (lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) {
        if (vi->state == VEHICLE_STATUS_READY) {
            vi->state = VEHICLE_STATUS_RUNNING;
        }
        else {
            
            // 맵 밖으로 나가지 않은 경우
            if (!is_position_outside(pos_cur)) {
                // 현재 쓰레드가 Lock을 가지고 있는 상황이면 Lock 풀기
                if (lock_held_by_current_thread(&vi->map_locks[pos_cur.row][pos_cur.col])) {
                    lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
                }

                // 임계영역 Lock을 풀어준 후 임계영역에 들어 있는 자동차 수를 감소시킨다.
                if (vi->is_critical) {
                    lock_acquire(&lock_critical);
                    cnt_critical--;
                    vi->is_critical = false; // 임계영역 탈출
                    lock_release(&lock_critical);
                }
            }
        }
        
        // 다음위치가 임계영역 안 인 경우
        if ((pos_next.row >= 2 && pos_next.row <= 4) && (pos_next.col >= 2 && pos_next.col <= 4)) {
            lock_acquire(&lock_critical);
            
            // 임계영역에 이미 4대의 자동차가 있는 경우 진입 불가능
            if (cnt_critical >= 4) {
                lock_release(&lock_critical);
               
                // 정지이므로 다음 위치의 map_Lock을 release 해야함.
                lock_release(&vi->map_locks[pos_next.row][pos_next.col]);
                
                //정지이므로 현재 위치의 Lock을 획득해야 함.
                lock_try_acquire(&vi->map_locks[pos_cur.row][pos_cur.col]);
                return -1; // 정지이므로 -1
            }

            cnt_critical++;
            vi->is_critical = true; // 자동차가 임계영역에 있음
           
            lock_release(&lock_critical);
        }

        // 자동차 위치 업데이트
        vi->position = pos_next;

        return 1;
    }
    return -1;
}

void init_on_mainthread(int thread_cnt) {
    total_threads = thread_cnt; // 자동차 쓰레드 개수( 총 자동차 개수)
    lock_init(&lock_step);
    lock_init(&lock_critical);
    cond_init(&waiting_thread);
    move_car_cnt = 0; // 움직이는 자동차 0대
    cnt_critical = 0; // 임계영역에 들어가있는 자동차 수 0대
 
}

void vehicle_loop(void* _vi) {
    int res;
    int start, dest, step;

    struct vehicle_info* vi = _vi;

    start = vi->start - 'A';
    dest = vi->dest - 'A';

    vi->state = VEHICLE_STATUS_READY;
    vi->position.row = vi->position.col = -1;
    vi->is_critical = false; 

    step = 0;
    while (1) {
        /* vehicle main code */

        res = try_move(start, dest, step, vi);
        
        if (res == 1) {
            step++;
        }

        //단위 스텝에 lock을 걸어서 step값에 일관성을 보장한다.
        lock_acquire(&lock_step);
        
        if (res == 0) {
            total_threads--;  // 자동차가 목적지에 도착했으므로 총쓰레드 개수(자동차의 총개수) 가 1 감소
        }
        else {
            move_car_cnt++; // 움직임이 끝난 자동차 1 증가
        }

        if (move_car_cnt == total_threads) {
            //모든 자동차가 움직였으므로 단위 스텝 1증가
            crossroads_step++;
            unitstep_changed();
            move_car_cnt = 0;  // 다시 0으로
            cond_broadcast(&waiting_thread, &lock_step); // waiting된 thread들을 모두 활동 상태로 변경
        }
        else {
            //모든 자동차가 수행할때까지 정지
            cond_wait(&waiting_thread, &lock_step);
        }

        //단위 스텝 변경이 끝났으므로 Lock 풀기
        lock_release(&lock_step);

        if (res == 0) {
            break;
        }

    }

    /* status transition must happen before sema_up */
    vi->state = VEHICLE_STATUS_FINISHED;
}