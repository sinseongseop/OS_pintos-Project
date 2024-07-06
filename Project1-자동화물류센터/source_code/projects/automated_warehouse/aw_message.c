#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/robot.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>

/** message boxes from central control node to each robot */
struct messsage_box* boxes_from_central_control_node;
/** message boxes from robots to central control node */
struct messsage_box* boxes_from_robots;

// message�� cmd ���� : 0:(0,0) wait , 1:(0,-1) ��  , 2:(1,0) �� , 3:(0,1) �� , 4:(-1,0) ��

int requirePosition[8][2] = { {0,0},{1,1},{1,3},{1,4},{1,5},{4,1},{4,3},{4,4} }; // ������ �ִ� ��ġ
int destination[4][2] = { {0,0},{0,2},{2,0},{5,2} }; // A,B,C �Ͽ� ����� ��ġ

void initialize_messageBoxs(int robotCnt) {
	boxes_from_central_control_node = malloc(sizeof(struct messsage_box) * robotCnt);
	boxes_from_robots = malloc(sizeof(struct messsage_box) * robotCnt);
    for (int i = 0; i < robotCnt; i++) {
        boxes_from_robots[i].dirtyBit = 0;
        boxes_from_central_control_node[i].dirtyBit = 0;
    }
}

void writeMessageByRobot(int robotIndex, struct robot* _robot) {
	//printf("%d : writeMessageByRobt ����\n", robotIndex);
    if (boxes_from_robots[robotIndex].dirtyBit == 0) { //dirtybit�� 1�ΰ�� ���� ������� ���� �޼����� �� ���, 0�� ��� �޼����� ó���� ���(or �Ҵ� X)
        boxes_from_robots[robotIndex].dirtyBit = 1;
        boxes_from_robots[robotIndex].msg.row = _robot->row;
        boxes_from_robots[robotIndex].msg.col = _robot->col;
        boxes_from_robots[robotIndex].msg.required_payload = _robot->required_payload;
        boxes_from_robots[robotIndex].msg.current_payload = _robot->current_payload;
        boxes_from_robots[robotIndex].msg.cmd = 0;
    }
    //printf("%d: WriteMessageByRobt ���� \n\n", robotIndex);
}



void readMessageByRobot(int robotIndex, struct robot* _robot) {
    //printf("%d : readMessageByRobot ����\n", robotIndex);
    //printf("%d \n", boxes_from_central_control_node[robotIndex].msg.cmd);
    
    if (boxes_from_central_control_node[robotIndex].dirtyBit == 1) {
        if (boxes_from_central_control_node[robotIndex].msg.cmd == 1) { // ��
            _robot->col = _robot->col - 1;
        }
        else if (boxes_from_central_control_node[robotIndex].msg.cmd == 2) { // ��
            _robot->row = _robot->row + 1;
        }
        else if (boxes_from_central_control_node[robotIndex].msg.cmd == 3) { // ��
            _robot->col = _robot->col + 1;
        }
        else if (boxes_from_central_control_node[robotIndex].msg.cmd == 4) { // ��
            _robot->row = _robot->row - 1;
        }

        int gotoPosition = _robot->required_payload % 10;
        if (_robot->col == requirePosition[gotoPosition][1] && _robot->row == requirePosition[gotoPosition][0]) { //�κ��� ���� �����ϴ� ���� ������ ���� �����Ѵ�.
            _robot->current_payload = gotoPosition;
        }

        boxes_from_central_control_node[robotIndex].dirtyBit = 0; // �޼����� �� ��������Ƿ� ���ο� �޼����� ���̴� �� ���.
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
        while (boxes_from_robots[i].dirtyBit == 0) {}; //�߾� ���� ��� �κ����� �޼����� ���� ������ ���
    }

    int total = 0; // �̹��Ͽ� �Ͽ���ҿ� �����ϴ� �κ��� �� ��

    // �κ����� �̵������� ��ġ�� ǥ�� �ϱ� ���� ��
    char world[6][7] = {
        {'X', 'X', 'D', 'X', 'X', 'X', 'X' },
        { 'X', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'D', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'X', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'X', ' ', ' ', ' ', ' ', ' ', 'X' },
        { 'X', 'X', 'D', 'X', 'X', 'W', 'X' }
    };

    // S��ġ�� ������ ������ ��ġ�� �κ��� ��ġ�� world�� X�� ǥ��
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
        else if (currentLoad == 0) { // ���� ���� ��Ȳ
 
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
        else { // ���ϴ� ���� ���� ����
            if (row == 1 && column == 2) {
                writeMessageByCentral(i, 4);
                total += 1; // A ������ �κ� �Ѱ� �߰� ����
            }
            else if (row == 4 && column == 2) {
                writeMessageByCentral(i, 2); 
                total += 1;//C ������ �κ� �Ѱ� �߰� ����
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
                total += 1; //B ������ �κ� �Ѱ� �߰� ����
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
        boxes_from_robots[i].dirtyBit = 0; // �޼����� �� ��������Ƿ� ���ο� �޼����� ���̴� �� ���
    }

    return total; // �̹� �Ͽ� �Ͽ���ҿ� ������ �κ��� ��.
}
