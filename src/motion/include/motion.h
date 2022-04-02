#ifndef _MOTION_H_
#define _MOTION_H_

#include "rclcpp/rclcpp.hpp"
#include "main_interface/msg/walk_command.hpp"
#include "main_interface/msg/head_command.hpp"
#include "main_interface/msg/motion_command.hpp"

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <string.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>

#define	PORT	3838
#define	BUFLEN	4096
#define BUFSIZE	1024

#define PORTM	5000	//motion
#define PORTH	5001	//head
#define HOST	"localhost"

 //Socket LUA head&motion
int	sockmotion;
int	sockhead;
struct	sockaddr_in addrmotion;
struct	sockaddr_in addrhead;
struct	hostent *localserver;
char	dataMotion[20];
char	dataHead[20];
char	line[10];

class robotMotion : public rclcpp::Node
{
public:
    robotMotion(); //: Node("robot_motion")
    void walkCommand(const main_interface::msg::WalkCommand::SharedPtr msg);
    void headCommand(const main_interface::msg::HeadCommand::SharedPtr msg);
    void motionCommand(const main_interface::msg::MotionCommand::SharedPtr msg);
    void motion(char line[2]);
    double Walk(double x, double y, double a);
    void headMove(double pan, double tilt);

private:
    rclcpp::Subscription<main_interface::msg::WalkCommand>::SharedPtr walk_command_;
    rclcpp::Subscription<main_interface::msg::HeadCommand>::SharedPtr head_command_;
    rclcpp::Subscription<main_interface::msg::MotionCommand>::SharedPtr motion_command_;

};

#endif