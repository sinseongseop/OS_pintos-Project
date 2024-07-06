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
char* robotName[] = { "R1", "R2","R3", "R4", "R5", "R6", "R7", "R8", "R9"}; // 각 로봇별 이름
int idxs[11] = { 0, 0, 1, 2, 3, 4, 5, 6, 7 , 8, 9 }; // 0번 인덱스 : 로봇 총개수  1번 인덱스 이후 : 로봇에게 부여되는 고유 ID

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

// 중앙 관제 노드를 동작시키는 쓰레드가 수행하는 함수
void centralNode(void* aux) {
    int robotCnt = *((int*)aux);
    //printf("%d \n", robotCnt); // 디버깅 코드
    
    int total = 0;
    while (total != robotCnt) { // 모든 로봇이 짐을 하역 할때까지 반복
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

// 로봇을 동작시키는 쓰레드가 수행하는 함수
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
        
        //printf("%s %s %s", argv[1], argv[2]); // 디버깅 코드

        int robotCnt = atoi(argv[1]); // 로봇의 총 개수
        char* inputString = argv[2]; // 로봇들의 할 일

        // requireLoad 의미 : (A:10, B : 20, C; 30 + 물건 번호(1~7))
        int requireLoad[10] = { 0 }; // 각 로봇별 원하는 짐

        // strtok_r를 사용하여 문자열을 :을 기준으로 파싱
        int robotNum = 0;
        char* token;
        char* savePtr; // strtok_r 함수에서 사용할 임시 변수

        token = strtok_r(inputString, ":", &savePtr);

        while (token != NULL) {
            //printf("%s\n", token); // 디버깅 코드

            // 문자열로부터 정수를 얻기 위해 atoi 함수 사용
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

            // 다음 토큰을 가져오기.
            token = strtok_r(NULL, ":", &savePtr);
            //printf("%d : %d \n", robotNum, requireLoad[robotNum]); //디버깅용 코드
            robotNum += 1;
        }
        

        initialize_messageBoxs(robotCnt); // 쓰레드간 메세지 통신을 위한 messageBox 초기화
        robots = malloc(sizeof(struct robot) * robotCnt); //로봇 생성

        for (int i = 0; i < robotCnt; i++) {
            setRobot(&robots[i], robotName[i], 5, 5, requireLoad[i], 0);
        }

        tid_t* threads = malloc(sizeof(tid_t) * (robotCnt + 1));
       
        idxs[0] = robotCnt; //0번 인덱스는 로봇의 총 개수
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