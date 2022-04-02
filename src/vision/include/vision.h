#ifndef _VISION_H_
#define _VISION_H_

#include "rclcpp/rclcpp.hpp"
#include "main_interface/msg/vision_ball.hpp"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>

#define PORT_VISION 2000
#define BUFLEN_VISION 4096 //Max length of buffer

struct sockaddr_in si_meV, si_otherV, addrTerima, addrKirim;
socklen_t slenV = sizeof(addrKirim);
int v, recv_lenV, socTrim;

class robotVision : public rclcpp::Node
{
public:
    robotVision(); //: Node("robot_vision")
    void BallCoor();
    void processing_vision();


private:
    char * parseVision;
    char recvVision[BUFLEN_VISION];
    int index_vision;

    int cntData;
    int dataAkhir[10];

    int Ball_X, Ball_Y, Ball_W, Ball_H, Ball_D;
    int Goal_X, Goal_Y, Goal_D, Goal_W, Goal_H, Goal_LH, Goal_RH, Goal_C, Goal_LD, Goal_RD;
    int Pinalty_D, Lcross_LD, Lcross_RD, Xcross_LD, Xcross_RD, Tcross_LD, Tcross_RD;
    int GoalLeft_X, GoalLeft_Y, GoalRight_X, GoalRight_Y;
    int RobotCoor_X, RobotCoor_Y, BallCoor_X, BallCoor_Y, HeadingCoor;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<main_interface::msg::VisionBall>::SharedPtr vision_;
    
};

#endif