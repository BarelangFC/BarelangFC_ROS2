#include "motion.h"

void initSendDataHead()
{
    sockhead = socket(AF_INET, SOCK_DGRAM, 0);
    localserver = gethostbyname(HOST);
    bzero((char *) &addrhead, sizeof(addrhead));
    addrhead.sin_family = AF_INET;
    addrhead.sin_port = htons(PORTH);
}

void initSendDataMotion()
{
    sockmotion = socket(AF_INET, SOCK_DGRAM, 0);
    localserver = gethostbyname(HOST);
    bzero((char *) &addrmotion, sizeof(addrmotion));
    addrmotion.sin_family = AF_INET;
    addrmotion.sin_port = htons(PORTM);
}

robotMotion::robotMotion() : rclcpp::Node("robot_motion")
{
    walk_command_ = this -> create_subscription<main_interface::msg::WalkCommand>(
        "robot_walk",10,
        std::bind(&robotMotion::walkCommand, this, std::placeholders::_1));
    RCLCPP_INFO(this -> get_logger(), "robot_walk has been started.");

    head_command_ = this -> create_subscription<main_interface::msg::HeadCommand>(
        "robot_head",10,
        std::bind(&robotMotion::headCommand, this, std::placeholders::_1));
    RCLCPP_INFO(this -> get_logger(), "robot_head has been started.");

    motion_command_ = this -> create_subscription<main_interface::msg::MotionCommand>(
        "robot_motion",10,
        std::bind(&robotMotion::motionCommand, this, std::placeholders::_1));
    RCLCPP_INFO(this -> get_logger(), "robot_motion has been started.");    
}

void robotMotion::walkCommand(const main_interface::msg::WalkCommand::SharedPtr msg)
{
    Walk(msg->x, msg->y, msg->a);
}

void robotMotion::headCommand(const main_interface::msg::HeadCommand::SharedPtr msg)
{
    headMove(msg->pan, msg->tilt);
}

void robotMotion::motionCommand(const main_interface::msg::MotionCommand::SharedPtr msg)
{
    motion(&msg->motion[0]);
}

void robotMotion::motion(char line[2])
{
    char awalan[50];
    strcpy(awalan, "motion");
    sprintf(dataMotion, "%s,%s",awalan, line);
    sendto(sockmotion, dataMotion, strlen(dataMotion), 0, (struct sockaddr *) &addrmotion, sizeof(addrmotion));
    printf("  data motion = %s,%s,%s\n", dataMotion,awalan,line);
}

double robotMotion::Walk(double x, double y, double a)
{
    char line[50];

    strcpy(line, "walk");
    sprintf(dataMotion, "%s,%.2f,%.2f,%.2f", line, x, y, a);
    sendto(sockmotion, dataMotion, strlen(dataMotion), 0,(struct sockaddr *) &addrmotion, sizeof(addrmotion));
    printf("  data walk = %s\n", dataMotion);
}

void robotMotion::headMove(double pan, double tilt)
{
    sprintf(dataHead, "%.2f,%.2f", pan, tilt);
    sendto(sockhead, dataHead, strlen(dataHead), 0,(struct sockaddr *) &addrhead, sizeof(addrhead));
    printf("  data head = %s\n", dataHead);
}

int main(int argc, char **argv)
{
    initSendDataHead();
    initSendDataMotion();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotMotion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}