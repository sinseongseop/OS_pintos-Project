#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"
#include "projects/crossroads/crossroads.h"

// �������� ����
struct semaphore move_decision_sema;
struct semaphore waiting_thread_sema;
struct semaphore all_threads_done;

// ���� ���� 
int waiting_threads; // ��� ���� ������ ����
int position[4]; // {2,2},{2,4},{4,2},{4,4}�� ���� ������ ���� �ֳ�?
int total_threads; // ��ü ������ ����

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

    sema_init(&move_decision_sema, 1); // �������� ������ �����ϴ� �������� �ʱ�ȭ
    sema_init(&waiting_thread_sema, 1); // waiting_thread ���� �����ϴ� �������� �ʱ�ȭ
    sema_init(&all_threads_done, 0); // ��� ������ �Ϸ� �������� �ʱ�ȭ
    waiting_threads = 0; // ��� ���� ������ �� �ʱ�ȭ
    crossroads_step = 0; // ���� ���� �ʱ�ȭ
    total_threads = thread_cnt; // ��ü ������ �� �ʱ�ȭ

    for (int i = 0; i < 4; i++) { // ��ġ �ʱ�ȭ
        position[i] = 0;
    }
}

// move_point[i][j][k] �� i�� �������̰� j�� �������� ��� k��° �𼭸��� �������� 1 �ƴϸ� 0 
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

// ���� ������ ������ ��ġ
int decision_point[4][2] = { {1,2},{2,5},{4,1},{5,4} };

// ������ �ڿ� ��ȯ�� �ϴ� ��ġ
int release_point[4][2] = { {1,4},{2,1},{4,5},{5,2} };

// ������ ���� ������ ������ �Լ�
void move_decision(struct vehicle_info* vi) {

    // 1�̸� ������ ���� ���� �� �ܴ� �Ұ���.
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

    // �����ο� ���� ���� ���� ������Ʈ
    if (possible_move != 1) {
        vi->state = VEHICLE_STATUS_STOP;
    }
    else {
        vi->state = VEHICLE_STATUS_RUNNING;
        for (int i = 0; i < 4; i++) { // ������ �𼭸��� ���� 
            if (move_point[start][dest][i] == 1) {
                position[i] = 1;
            }
        }
    }
}

// ������ �ִ� ������ �ڿ��� ��ȯ�ϴ� �Լ�
void release_resource(struct vehicle_info* vi) {

    int start = vi->start - 'A';
    int dest = vi->dest - 'A';

    for (int i = 0; i < 4; i++) { // ������ �𼭸��� ��ȯ 
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

        // ������ ���� ������ ������ �ϴ� ��ġ�� �Ǵ��� ����
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

        // ���� ���� ������ ���� ����
        sema_down(&waiting_thread_sema);
        waiting_threads++;
        if (waiting_threads == total_threads) {
            waiting_threads = 0;
            sema_up(&waiting_thread_sema); // waiting_thread_sema�� �ٽ� ��
            crossroads_step = crossroads_step + 1;
            unitstep_changed();
            for (int i = 0; i < total_threads - 1; i++) {
                sema_up(&all_threads_done); // ��� ������ �Ϸ� �������� ��
            }
        }
        else {
            sema_up(&waiting_thread_sema);
            sema_down(&all_threads_done); // ��� ������ �Ϸ� ���
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