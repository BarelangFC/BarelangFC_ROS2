#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "rclcpp/rclcpp.hpp"
#include "main_interface/msg/accelero.hpp"
#include "main_interface/msg/gyroscope.hpp"
#include "main_interface/msg/angle.hpp"
#include "main_interface/msg/button.hpp"
#include "main_interface/msg/serv_pos.hpp"

class robotSensor : public rclcpp::Node 
{
public:
    robotSensor();
    void accelero();
    void gyroscope();
    void button();
    void angle();
    void servoPos();
    void getSensor();
    void getServoPos();


private:
    rclcpp::TimerBase::SharedPtr timer_1, timer_2, timer_3, timer_4, timer_5;
    rclcpp::Publisher<main_interface::msg::Accelero>::SharedPtr accelero_;
    rclcpp::Publisher<main_interface::msg::Gyroscope>::SharedPtr gyroscope_;
    rclcpp::Publisher<main_interface::msg::Angle>::SharedPtr angle_;
    rclcpp::Publisher<main_interface::msg::Button>::SharedPtr button_;
    rclcpp::Publisher<main_interface::msg::ServPos>::SharedPtr servo_pos_;

    double accelero_x, accelero_y, accelero_z, gyroscope_x, gyroscope_y, gyroscope_z = 0;
    int roll, pitch, yaw, strategy, kill = 0;

    double  head1, head2,
            Larm1, Larm2, Larm3,
            Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6,
            Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6,
            Rarm1, Rarm2, Rarm3 = 0;
};

#endif