#include <stdio.h>
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"


// Lock & condition
struct lock lock_step; // step�� �ϰ����� �����ϱ� ���� Lock
struct lock lock_critical; // �Ӱ迵���� ���� �ڵ����� ���� �ϰ����� ���� Lock
struct condition waiting_thread; //wating ���� thread���� ����

// ���� ����
int total_threads;  // �� ������(�� �ڵ���)�� ����
int move_car_cnt; // try_move�� ���� �ڵ����� �Ѱ���
int cnt_critical; // �Ӱ迵���� ���ִ� �ڵ����� ����

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

    // ���� ��ġ�� Lock�� ȹ���Ϸ��� �õ�
    if (lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) {
        if (vi->state == VEHICLE_STATUS_READY) {
            vi->state = VEHICLE_STATUS_RUNNING;
        }
        else {
            
            // �� ������ ������ ���� ���
            if (!is_position_outside(pos_cur)) {
                // ���� �����尡 Lock�� ������ �ִ� ��Ȳ�̸� Lock Ǯ��
                if (lock_held_by_current_thread(&vi->map_locks[pos_cur.row][pos_cur.col])) {
                    lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
                }

                // �Ӱ迵�� Lock�� Ǯ���� �� �Ӱ迵���� ��� �ִ� �ڵ��� ���� ���ҽ�Ų��.
                if (vi->is_critical) {
                    lock_acquire(&lock_critical);
                    cnt_critical--;
                    vi->is_critical = false; // �Ӱ迵�� Ż��
                    lock_release(&lock_critical);
                }
            }
        }
        
        // ������ġ�� �Ӱ迵�� �� �� ���
        if ((pos_next.row >= 2 && pos_next.row <= 4) && (pos_next.col >= 2 && pos_next.col <= 4)) {
            lock_acquire(&lock_critical);
            
            // �Ӱ迵���� �̹� 4���� �ڵ����� �ִ� ��� ���� �Ұ���
            if (cnt_critical >= 4) {
                lock_release(&lock_critical);
               
                // �����̹Ƿ� ���� ��ġ�� map_Lock�� release �ؾ���.
                lock_release(&vi->map_locks[pos_next.row][pos_next.col]);
                
                //�����̹Ƿ� ���� ��ġ�� Lock�� ȹ���ؾ� ��.
                lock_try_acquire(&vi->map_locks[pos_cur.row][pos_cur.col]);
                return -1; // �����̹Ƿ� -1
            }

            cnt_critical++;
            vi->is_critical = true; // �ڵ����� �Ӱ迵���� ����
           
            lock_release(&lock_critical);
        }

        // �ڵ��� ��ġ ������Ʈ
        vi->position = pos_next;

        return 1;
    }
    return -1;
}

void init_on_mainthread(int thread_cnt) {
    total_threads = thread_cnt; // �ڵ��� ������ ����( �� �ڵ��� ����)
    lock_init(&lock_step);
    lock_init(&lock_critical);
    cond_init(&waiting_thread);
    move_car_cnt = 0; // �����̴� �ڵ��� 0��
    cnt_critical = 0; // �Ӱ迵���� ���ִ� �ڵ��� �� 0��
 
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

        //���� ���ܿ� lock�� �ɾ step���� �ϰ����� �����Ѵ�.
        lock_acquire(&lock_step);
        
        if (res == 0) {
            total_threads--;  // �ڵ����� �������� ���������Ƿ� �Ѿ����� ����(�ڵ����� �Ѱ���) �� 1 ����
        }
        else {
            move_car_cnt++; // �������� ���� �ڵ��� 1 ����
        }

        if (move_car_cnt == total_threads) {
            //��� �ڵ����� ���������Ƿ� ���� ���� 1����
            crossroads_step++;
            unitstep_changed();
            move_car_cnt = 0;  // �ٽ� 0����
            cond_broadcast(&waiting_thread, &lock_step); // waiting�� thread���� ��� Ȱ�� ���·� ����
        }
        else {
            //��� �ڵ����� �����Ҷ����� ����
            cond_wait(&waiting_thread, &lock_step);
        }

        //���� ���� ������ �������Ƿ� Lock Ǯ��
        lock_release(&lock_step);

        if (res == 0) {
            break;
        }

    }

    /* status transition must happen before sema_up */
    vi->state = VEHICLE_STATUS_FINISHED;
}