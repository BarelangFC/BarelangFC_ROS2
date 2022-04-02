#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "main_interface/msg/vision_ball.hpp"
#include "main_interface/msg/button.hpp"
#include "main_interface/msg/walk_command.hpp"
#include "main_interface/msg/head_command.hpp"
#include "main_interface/msg/motion_command.hpp"
#include "main_interface/msg/coordination.hpp"
#include "main_interface/msg/robot_pos.hpp"
#include "main_interface/msg/delta_pos.hpp"

class robotStrategy : public rclcpp::Node
{
public:
    robotStrategy(); //: Node("robot_strategy")
    void BallCoor(const main_interface::msg::VisionBall::SharedPtr msg);
    void button(const main_interface::msg::Button::SharedPtr msg);
    void delta_pos(const main_interface::msg::DeltaPos::SharedPtr msg);
    
    void robot_pos();
    void walk_command();
    void head_command();
    void motion_command();
    void coordination();

    void motion(std::string motion);
    void headMove(double pan, double tilt);
    void Walk(double x, double y, double a);

    void main_strategy();
    int ballLost(int thresshold);
    void searchBallRectang(double atas, double kanan, double bawah, double kiri);
    void trackBall();
    void followBall(int mode);
    void ballPositioning(double setPointX, double setPointY, double speed);
    void kick(int mode);

private:
    // Head movement
    /**********************************/
    double   head_pan   =  0.0
            ,head_tilt  =  0.0
            ,pan_rate   = -0.05
            ,tilt_rate  = -0.05;
    /**********************************/
    // Follow Ball
    /**********************************/
    double  jalan		= 0.040		
		    ,lari		= 0.050		
		    ,kejar		= 0.060		
		    ,kejar_mid	= 0.070
		    ,kejar_max	= 0.080;
    /**********************************/
    // Ball Positioning
    /**********************************/
    double  p_pan_tendang   = -0.25, 
            p_pan_oper      = -0.53, 
            p_tilt_tendang  = -0.25, 
            p_tilt_oper     = -0.53, 
            ball_positioning_speed  = 0.0;
    /**********************************/
    // Track Ball
    /**********************************/
    double  ball_panKP  = 0.06, 
            ball_panKD  = 0.0000505, 
            ball_tiltKP = 0.05,	
            ball_tiltKD = 0.0000755,
            frame_X     = 1280,
            frame_Y     = 720;
    /**********************************/
    // Case
    /**********************************/
    int     first_state_condition = 0;
    int     strategy_number, kill = 0;
    /**********************************/
    // Coordination
    /**********************************/
    bool refresh        = false
    , play              = false;
    int robot_id        = 1
    , grid_ball         = 0
    , back_in           = 0
    , semeh             = 0 
    , grid              = 0
    , state_condition   = 0
    , robot_status      = 0
    , f_ball            = 0
    , state             = 0
    , last_state        = 0;
    /**********************************/
    
    double walk_x, walk_y, walk_a = 0;
    int ball_x, ball_y, ball_pos_x, ball_pos_y, ball_coor_x, ball_coor_y,
        deltaPos_X, deltaPos_Y, robotPos_X, robotPos_Y, initialPos_X,
        initialPos_Y = 0;

    std::string motion_ = "";

    rclcpp::TimerBase::SharedPtr timer_1, timer_2, timer_3, timer_4, timer_5;

    rclcpp::Subscription<main_interface::msg::VisionBall>::SharedPtr vision_;
    rclcpp::Subscription<main_interface::msg::Button>::SharedPtr button_;
    rclcpp::Subscription<main_interface::msg::DeltaPos>::SharedPtr delta_pos_;

    rclcpp::Publisher<main_interface::msg::WalkCommand>::SharedPtr walk_command_;
    rclcpp::Publisher<main_interface::msg::HeadCommand>::SharedPtr head_command_;
    rclcpp::Publisher<main_interface::msg::MotionCommand>::SharedPtr motion_command_;
    rclcpp::Publisher<main_interface::msg::Coordination>::SharedPtr coordination_;
    rclcpp::Publisher<main_interface::msg::RobotPos>::SharedPtr robot_pos_;

};

#endif