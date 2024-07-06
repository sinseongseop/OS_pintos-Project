#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/robot.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>

/** message boxes from central control node to each robot */
struct messsage_box* boxes_from_central_control_node;
/** message boxes from robots to central control node */
struct messsage_box* boxes_from_robots;

// message의 cmd 정의 : 0:(0,0) wait , 1:(0,-1) 서  , 2:(1,0) 남 , 3:(0,1) 동 , 4:(-1,0) 북

int requirePosition[8][2] = { {0,0},{1,1},{1,3},{1,4},{1,5},{4,1},{4,3},{4,4} }; // 물건이 있는 위치
int destination[4][2] = { {0,0},{0,2},{2,0},{5,2} }; // A,B,C 하역 장소의 위치

void initialize_messageBoxs(int robotCnt) {
	boxes_from_central_control_node = malloc(sizeof(struct messsage_box) * robotCnt);
	boxes_from_robots = malloc(sizeof(struct messsage_box) * robotCnt);
    for (int i = 0; i < robotCnt; i++) {
        boxes_from_robots[i].dirtyBit = 0;
        boxes_from_central_control_node[i].dirtyBit = 0;
    }
}

void writeMessageByRobot(int robotIndex, struct robot* _robot) {
	//printf("%d : writeMessageByRobt 수행\n", robotIndex);
    if (boxes_from_robots[robotIndex].dirtyBit == 0) { //dirtybit가 1인경우 아직 사용하지 않은 메세지가 든 경우, 0인 경우 메세지가 처리된 경우(or 할당 X)
        boxes_from_robots[robotIndex].dirtyBit = 1;
        boxes_from_robots[robotIndex].msg.row = _robot->row;
        boxes_from_robots[robotIndex].msg.col = _robot->col;
        boxes_from_robots[robotIndex].msg.required_payload = _robot->required_payload;
        boxes_from_robots[robotIndex].msg.current_payload = _robot->current_payload;
        boxes_from_robots[robotIndex].msg.cmd = 0;
    }
    //printf("%d: WriteMessageByRobt 종료 \n\n", robotIndex);
}



void readMessageByRobot(int robotIndex, struct robot* _robot) {
    //printf("%d : readMessageByRobot 수행\n", robotIndex);
    //printf("%d \n", boxes_from_central_control_node[robotIndex].msg.cmd);
    
    if (boxes_from_central_control_node[robotIndex].dirtyBit == 1) {
        if (boxes_from_central_control_node[robotIndex].msg.cmd == 1) { // 서
            _robot->col = _robot->col - 1;
        }
        else if (boxes_from_central_control_node[robotIndex].msg.cmd == 2) { // 남
            _robot->row = _robot->row + 1;
        }
        else if (boxes_from_central_control_node[robotIndex].msg.cmd == 3) { // 동
            _robot->col = _robot->col + 1;
        }
        else if (boxes_from_central_control_node[robotIndex].msg.cmd == 4) { // 북
            _robot->row = _robot->row - 1;
        }

        int gotoPosition = _robot->required_payload % 10;
        if (_robot->col == requirePosition[gotoPosition][1] && _robot->row == requirePosition[gotoPosition][0]) { //로봇이 짐을 적재하는 곳에 도착시 짐을 적재한다.
            _robot->current_payload = gotoPosition;
        }

        boxes_from_central_control_node[robotIndex].dirtyBit = 0; // 메세지를 다 사용했으므로 새로운 메세지가 쓰이는 걸 허용.
    }
}

void writeMessageByCentral(int robotIndex, int nextCommand) {

    if (boxes_from_central_control_node[robotIndex].dirtyBit == 0) {
        boxes_from_central_control_node[robotIndex].msg.cmd = nextCommand;
        boxes_from_central_control_node[robotIndex].dirtyBit = 1;
    }

}

int MesaageByCentral(int robotCnt) {
    for (int i = 0; i < robotCnt; i++) {
        while (boxes_from_robots[i].dirtyBit == 0) {}; //중앙 노드는 모든 로봇들이 메세지를 보낼 때까지 대기
    }

    int total = 0; // 이번턴에 하역장소에 도착하는 로봇의 총 수

    // 로봇들이 이동가능한 위치를 표기 하기 위한 맵
    char world[6][7] = {
        {'X', 'X', 'D', 'X', 'X', 'X', 'X' },
        { 'X', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'D', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'X', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'X', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'X', 'X', 'D', 'X', 'X', 'W', 'X' }
    };

    // S위치를 제외한 나머지 위치의 로봇의 위치를 world에 X로 표시
    for (int i = 0; i < robotCnt; i++) {
        int row = boxes_from_robots[i].msg.row;
        int column = boxes_from_robots[i].msg.col;
        if (!(row == 4 && column == 5)) {
            world[row][column] = 'X';
        }
        
        
    }

    for (int i = 0; i < robotCnt; i++) {
        int row = boxes_from_robots[i].msg.row;
        int column = boxes_from_robots[i].msg.col;
        int require = boxes_from_robots[i].msg.required_payload % 10;
        int position = boxes_from_robots[i].msg.required_payload / 10;
        int currentLoad = boxes_from_robots[i].msg.current_payload;
        //printf("%d %d %d %d %d \n", row, column, require, position, currentLoad);

        if (column == 5 && row > 2) {
            if (row == 5 && world[row - 1][column] != 'X') {
                world[row - 1][column] = 'X';
                writeMessageByCentral(i, 4);
            }
            else if (row != 5) {
                writeMessageByCentral(i, 4);
            }
            else {
                writeMessageByCentral(i, 0);
            }
        }
        else if (currentLoad == 0) { // 짐이 없는 상황
 
            if (row == 2 && requirePosition[require][0] == row - 1 && requirePosition[require][1] == column && world[row - 1][column] != 'X') {
                writeMessageByCentral(i, 4);
            }
            else if (row == 3 && requirePosition[require][0] == row + 1 && requirePosition[require][1] == column && world[row + 1][column] != 'X') {
                writeMessageByCentral(i, 2);
            }
            else if (row == 2 && column == 1) {
                writeMessageByCentral(i, 2);
            }
            else if (row == 3) {
                writeMessageByCentral(i, 3);
            }
            else if (row == 2) {
                writeMessageByCentral(i, 1);
            }
            else {
                writeMessageByCentral(i, 0);
            }

        }
        else { // 원하는 짐을 가진 상태
            if (row == 1 && column == 2) {
                writeMessageByCentral(i, 4);
                total += 1; // A 구역에 로봇 한개 추가 도착
            }
            else if (row == 4 && column == 2) {
                writeMessageByCentral(i, 2); 
                total += 1;//C 구역에 로봇 한개 추가 도착
            }
            else if (row == 1 && column !=5) {
                if (world[row+1][column+1]!='X') {
                    writeMessageByCentral(i, 2);
                }
                else {
                    writeMessageByCentral(i, 0);
                }
            }
            else if (row == 1 && column == 5) {
                if (world[row + 2][column ] != 'X') {
                    writeMessageByCentral(i, 2);
                }
                else {
                    writeMessageByCentral(i, 0);
                }
            
            }
            else if (row == 4 && column == 5) {
                if (world[row + 2][column] != 'X') {
                    writeMessageByCentral(i, 2);
                }
                else {
                    writeMessageByCentral(i, 0);
                }
            }
            else if (row == 4 && column == 1) {
                if (world[row - 2][column] != 'X') {
                    writeMessageByCentral(i, 4);
                }
                else {
                    writeMessageByCentral(i, 0);
                }
            }
            else if (row == 4) {
                if (world[row - 1][column-1] != 'X') {
                    writeMessageByCentral(i, 4);
                }
                else {
                    writeMessageByCentral(i, 0);
                }

            }
            else if (row == 2 && column == 1 && position == 2) {
                writeMessageByCentral(i, 1);
                total += 1; //B 구역에 로봇 한개 추가 도착
            }
            else if (row == 2 && column == 2 && position == 1) {
                writeMessageByCentral(i, 4);
            }
            else if (row == 3 && column == 2 && position == 3) {
                writeMessageByCentral(i, 2);
            }
            else if (row == 2 && column == 1) {
                writeMessageByCentral(i, 2);
            }
            else if (row== 2 && column > 1) {
                writeMessageByCentral(i, 1);
            }
            else if (row == 3 ) {
                writeMessageByCentral(i, 3);
            }
            else {
                writeMessageByCentral(i, 0);
            }
        
        }
    }

    for (int i = 0; i < robotCnt; i++) {
        boxes_from_robots[i].dirtyBit = 0; // 메세지를 다 사용했으므로 새로운 메세지가 쓰이는 걸 허용
    }

    return total; // 이번 턴에 하역장소에 도착한 로봇의 수.
}
