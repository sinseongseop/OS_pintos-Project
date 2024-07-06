#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/aw_thread.h"

struct robot* robots;
char* robotName[] = { "R1", "R2","R3", "R4", "R5", "R6", "R7", "R8", "R9"}; // �� �κ��� �̸�
int idxs[11] = { 0, 0, 1, 2, 3, 4, 5, 6, 7 , 8, 9 }; // 0�� �ε��� : �κ� �Ѱ���  1�� �ε��� ���� : �κ����� �ο��Ǵ� ���� ID

// test code for central control node thread
void test_cnt(){
        while(1){
                print_map(robots, 5);
                thread_sleep(1000);
                block_thread();
        }
}

// test code for robot thread
void test_thread(void* aux){
        int idx = *((int *)aux);
        int test = 0;
        while(1){
                printf("thread %d : %d\n", idx, test++);
                thread_sleep(idx * 1000);
        }
}

// �߾� ���� ��带 ���۽�Ű�� �����尡 �����ϴ� �Լ�
void centralNode(void* aux) {
    int robotCnt = *((int*)aux);
    //printf("%d \n", robotCnt); // ����� �ڵ�
    
    int total = 0;
    while (total != robotCnt) { // ��� �κ��� ���� �Ͽ� �Ҷ����� �ݺ�
        thread_sleep(1000);
        total+=MesaageByCentral(robotCnt);
        thread_sleep(1000);
        print_map(robots, robotCnt);
        increase_step();
        unblock_threads();
    }

    thread_sleep(1000);
    total += MesaageByCentral(robotCnt);
    print_map(robots, robotCnt);
    increase_step();

    printf("simulation success! congratulation! \n");
}

// �κ��� ���۽�Ű�� �����尡 �����ϴ� �Լ�
void robotNode(void* aux) {
    int idx = *((int*)aux);
    while (1) {
        writeMessageByRobot(idx, &robots[idx]);
        block_thread();
        readMessageByRobot(idx, &robots[idx]);
    }
}


// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this

        printf("implement automated warehouse!\n");
        
        //printf("%s %s %s", argv[1], argv[2]); // ����� �ڵ�

        int robotCnt = atoi(argv[1]); // �κ��� �� ����
        char* inputString = argv[2]; // �κ����� �� ��

        // requireLoad �ǹ� : (A:10, B : 20, C; 30 + ���� ��ȣ(1~7))
        int requireLoad[10] = { 0 }; // �� �κ��� ���ϴ� ��

        // strtok_r�� ����Ͽ� ���ڿ��� :�� �������� �Ľ�
        int robotNum = 0;
        char* token;
        char* savePtr; // strtok_r �Լ����� ����� �ӽ� ����

        token = strtok_r(inputString, ":", &savePtr);

        while (token != NULL) {
            //printf("%s\n", token); // ����� �ڵ�

            // ���ڿ��κ��� ������ ��� ���� atoi �Լ� ���
            requireLoad[robotNum] += atoi(&token[0]);

            if (token[1] == 'A') {
                requireLoad[robotNum] += 10;
            }
            else if (token[1] == 'B') {
                requireLoad[robotNum] += 20;
            }
            else if (token[1] == 'C') {
                requireLoad[robotNum] += 30;
            }

            // ���� ��ū�� ��������.
            token = strtok_r(NULL, ":", &savePtr);
            //printf("%d : %d \n", robotNum, requireLoad[robotNum]); //������ �ڵ�
            robotNum += 1;
        }
        

        initialize_messageBoxs(robotCnt); // �����尣 �޼��� ����� ���� messageBox �ʱ�ȭ
        robots = malloc(sizeof(struct robot) * robotCnt); //�κ� ����

        for (int i = 0; i < robotCnt; i++) {
            setRobot(&robots[i], robotName[i], 5, 5, requireLoad[i], 0);
        }

        tid_t* threads = malloc(sizeof(tid_t) * (robotCnt + 1));
       
        idxs[0] = robotCnt; //0�� �ε����� �κ��� �� ����
        threads[0] = thread_create("CNT", 0, &centralNode, &idxs[0]);
       
        for (int i = 1; i <= robotCnt; i++) {
            threads[i] = thread_create(robotName[i-1], 0, &robotNode, &idxs[i]);
        }

        // test case robots
        //robots = malloc(sizeof(struct robot) * 4);
        //setRobot(&robots[0], "R1", 5, 5, 0, 0);
        //setRobot(&robots[1], "R2", 0, 2, 0, 0);
        //setRobot(&robots[2], "R3", 1, 1, 1, 1);
        //setRobot(&robots[3], "R4", 5, 5, 0, 0);

        // example of create thread
        //tid_t* threads = malloc(sizeof(tid_t) * 4);
        //int idxs[4] = {1, 2, 3, 4};
        //threads[0] = thread_create("CNT", 0, &test_cnt, NULL);
        //threads[1] = thread_create("R1", 0, &test_thread, &idxs[1]);
        //threads[2] = thread_create("R2", 0, &test_thread, &idxs[2]);
        //threads[3] = thread_create("R3", 0, &test_thread, &idxs[3]);
        
}