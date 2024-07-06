#ifndef _PROJECTS_PROJECT1_AW_MESSAGE_H__
#define _PROJECTS_PROJECT1_AW_MESSAGE_H__
#include "projects/automated_warehouse/robot.h"

/**
 * For easy to implement, combine robot and central control node message
 * If you want to modify message structure, don't split it
 */
struct message {
    //
    // To central control node
    //
    /** current row of robot */
    int row;
    /** current column of robot */
    int col;
    /** current payload of robot */
    int current_payload;
    /** required paylod of robot */
    int required_payload;

    //
    // To robots
    //
    /** next command for robot */
    int cmd;
};

/** 
 * Simple message box which can receive only one message from sender
*/
struct messsage_box {
    /** check if the message was written by others */
    int dirtyBit;
    /** stored message */
    struct message msg;
};

/** message boxes from central control node to each robot */
extern struct messsage_box* boxes_from_central_control_node;
/** message boxes from robots to central control node */
extern struct messsage_box* boxes_from_robots;

void initialize_messageBoxs(int robotCnt);

void writeMessageByRobot(int robotIndex, struct robot* _robot);

void readMessageByRobot(int robotIndex, struct robot* _robot);

void writeMessageByCentral(int robotIndex, int nextCommand);

int MesaageByCentral(int robotCnt);



#endif