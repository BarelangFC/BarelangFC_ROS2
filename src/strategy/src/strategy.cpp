#include "strategy.h"
#include <unistd.h>
#define PI 3.1415926535897932384626433832795

robotStrategy::robotStrategy() : rclcpp::Node("robot_strategy")
{
    walk_command_ = this->create_publisher<main_interface::msg::WalkCommand>("robot_walk", 10);
    timer_1 = this->create_wall_timer(std::chrono::milliseconds(50),
                                      std::bind(&robotStrategy::walk_command, this));
    RCLCPP_INFO(this->get_logger(), "robot_walk has been started.");

    head_command_ = this->create_publisher<main_interface::msg::HeadCommand>("robot_head", 10);
    timer_2 = this->create_wall_timer(std::chrono::milliseconds(50),
                                      std::bind(&robotStrategy::head_command, this));
    RCLCPP_INFO(this->get_logger(), "robot_head has been started.");

    motion_command_ = this->create_publisher<main_interface::msg::MotionCommand>("robot_motion", 10);
    timer_3 = this->create_wall_timer(std::chrono::milliseconds(50),
                                      std::bind(&robotStrategy::motion_command, this));
    RCLCPP_INFO(this->get_logger(), "robot_motion has been started.");

    coordination_ = this->create_publisher<main_interface::msg::Coordination>("robot_coordination", 10);
    timer_4 = this->create_wall_timer(std::chrono::milliseconds(50),
                                      std::bind(&robotStrategy::coordination, this));
    RCLCPP_INFO(this->get_logger(), "robot_coordination has been started.");

    robot_pos_ = this->create_publisher<main_interface::msg::RobotPos>("robot_pos", 10);
    timer_5 = this->create_wall_timer(std::chrono::milliseconds(50),
                                      std::bind(&robotStrategy::robot_pos, this));
    RCLCPP_INFO(this->get_logger(), "robot_pos has been started.");

    vision_ = this->create_subscription<main_interface::msg::VisionBall>(
        "robot_vision", 10,
        std::bind(&robotStrategy::BallCoor, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "robot_vision has been started.");

    button_ = this->create_subscription<main_interface::msg::Button>(
        "robot_button", 10,
        std::bind(&robotStrategy::button, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "robot_button has been started.");

    delta_pos_ = this->create_subscription<main_interface::msg::DeltaPos>(
        "robot_delta", 10,
        std::bind(&robotStrategy::delta_pos, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "robot_delta has been started.");
}

void robotStrategy::delta_pos(const main_interface::msg::DeltaPos::SharedPtr msg)
{
    deltaPos_X = msg->x;
    deltaPos_Y = msg->y;
}

void robotStrategy::robot_pos()
{
    auto msg = main_interface::msg::RobotPos();
    msg.x = initialPos_X + deltaPos_X;
    msg.y = initialPos_Y + deltaPos_Y;
    robot_pos_->publish(msg);
}

void robotStrategy::walk_command()
{
    auto msg = main_interface::msg::WalkCommand();
    msg.x = walk_x;
    msg.y = walk_y;
    msg.a = walk_a;
    walk_command_->publish(msg);
}

void robotStrategy::head_command()
{
    auto msg = main_interface::msg::HeadCommand();
    msg.pan = head_pan;
    msg.tilt = head_tilt;
    head_command_->publish(msg);
}

void robotStrategy::motion_command()
{
    auto msg = main_interface::msg::MotionCommand();
    msg.motion = &motion_[0];
    motion_command_->publish(msg);
}

void robotStrategy::coordination()
{
    auto msg = main_interface::msg::Coordination();
    msg.state_condition = state_condition;
    coordination_->publish(msg);
    main_strategy();
}

void robotStrategy::BallCoor(const main_interface::msg::VisionBall::SharedPtr msg)
{
    ball_x = msg->x;
    ball_y = msg->y;
}

void robotStrategy::button(const main_interface::msg::Button::SharedPtr msg)
{
    strategy_number = msg->strategy;
    kill = msg->kill;
}

void robotStrategy::Walk(double x, double y, double a)
{
    walk_a = a;
    walk_x = x;
    walk_y = y;
}

void robotStrategy::headMove(double pan, double tilt)
{
    head_pan = pan;
    head_tilt = tilt;
}

void robotStrategy::motion(std::string motion)
{
    motion_ = motion;
}

int count_ball_lost, count_ball_found, return_ball_val = 0;
int robotStrategy::ballLost(int threshold)
{
    if (ball_x == -1 && ball_y == -1)
    {
        count_ball_found = 0;
        count_ball_lost++;
        if (count_ball_lost >= threshold)
        {
            return_ball_val = 1;
        }
    }
    else
    {
        count_ball_lost = 0;
        count_ball_found++;
        if (count_ball_found > 1)
        {
            return_ball_val = 0;
        }
    }
    return return_ball_val;
}

bool neckX;
void robotStrategy::searchBallRectang(double atas, double kanan, double bawah, double kiri)
{
    if (neckX)
    {
        head_pan += pan_rate;
        if (head_pan >= kiri || head_pan <= kanan)
        {
            pan_rate *= -1;
            neckX = false;
        }
    }
    else
    {
        head_tilt += tilt_rate;
        if (head_tilt <= atas || head_tilt >= bawah)
        {
            tilt_rate *= -1;
            neckX = true;
        }
    }

    if (head_pan >= kiri)
    {
        head_pan = kiri;
    }
    else if (head_pan <= kanan)
    {
        head_pan = kanan;
    }
    if (head_tilt <= atas)
    {
        head_tilt = atas;
    }
    else if (head_tilt >= bawah)
    {
        head_tilt = bawah;
    }
    printf("%.2f,%.2f\n",head_pan, head_tilt);

    headMove(head_pan, head_tilt);
}

double intPanB = 0, dervPanB = 0, errorPanB = 0, preErrPanB = 0,
       PPanB = 0, IPanB = 0, DPanB = 0,
       intTiltB = 0, dervTiltB = 0, errorTiltB = 0, preErrTiltB = 0,
       PTiltB = 0, ITiltB = 0, DTiltB = 0,
       dtB = 0.04;
double B_Pan_err_diff, B_Pan_err, B_Tilt_err_diff, B_Tilt_err, B_PanAngle, B_TiltAngle,
    pOffsetB, iOffsetB, dOffsetB,
    errorPanBRad, errorTiltBRad,
    offsetSetPointBall;
void robotStrategy::trackBall()
{
    if (ball_x != -1 && ball_y != -1)
    { // printf("Tracking");
        ball_pos_x = ball_coor_x;
        ball_pos_y = ball_coor_y;
        offsetSetPointBall = (int)((head_tilt * 30) + 54);
        if (offsetSetPointBall > 36)
            offsetSetPointBall = 36;
        else if (offsetSetPointBall < 0)
            offsetSetPointBall = 0;

        errorPanB = (double)ball_x - ((frame_X / 2) + offsetSetPointBall); // 160
        errorTiltB = (double)ball_y - (frame_Y / 2);                       // 120
        errorPanB *= -1;
        errorTiltB *= -1;
        errorPanB *= (90 / (double)frame_X);  // pixel per angle
        errorTiltB *= (60 / (double)frame_Y); // pixel per angle

        errorPanBRad = (errorPanB * PI) / 180;
        errorTiltBRad = (errorTiltB * PI) / 180;
        // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanB, errorTiltB);
        // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanBRad, errorTiltBRad);
        // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

        B_Pan_err_diff = errorPanBRad - B_Pan_err;
        B_Tilt_err_diff = errorTiltBRad - B_Tilt_err;

        // PID pan ==========================================================
        PPanB = B_Pan_err * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
        intPanB += B_Pan_err * dtB;
        IPanB = intPanB * 0.0;
        dervPanB = B_Pan_err_diff / dtB;
        // DPanB = dervPanB * kamera.panKD;
        DPanB = dervPanB * ball_panKD;
        B_Pan_err = errorPanBRad;
        head_pan += (PPanB + IPanB + DPanB);

        // PID tilt ==========================================================
        PTiltB = B_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

        intTiltB += B_Tilt_err * dtB;
        ITiltB = intTiltB * 0.0;

        dervTiltB = B_Tilt_err_diff / dtB;
        DTiltB = dervTiltB * ball_tiltKD;

        preErrTiltB = errorTiltB;
        B_Tilt_err = errorTiltBRad;
        head_tilt += (PTiltB + ITiltB + DTiltB) * -1;

        if (head_pan >= 1.6)
        {
            head_pan = 1.6;
        }
        else if (head_pan <= -1.6)
        {
            head_pan = -1.6;
        }
        if (head_tilt <= -2.0)
        {
            head_tilt = -2.0;
        }
        else if (head_tilt >= -0.4)
        {
            head_tilt = -0.4;
        }

        headMove(head_pan, head_tilt); // printf("head_pan = %.2f \t head_tilt = %.2f\n", head_pan, head_tilt);
    }
}

// Follow Ball ===============================================================================
int	countReadyKick;
double	SetPointPan = 0,
	SetPointTilt = -0.8,//-0.08
	errorfPan,
	errorfTilt,
	PyMove = 0,
	PxMove = 0,
	PaMove = 0;
void robotStrategy::followBall(int mode){ //0 normal, 1 sambil belok
	trackBall();
	if	(head_tilt >= SetPointTilt) { head_tilt = SetPointTilt; }
	else if	(head_tilt < -2.0) { head_tilt = -2.0; }

	errorfPan  = head_pan - SetPointPan;
	errorfTilt = head_tilt - SetPointTilt;

	if (head_tilt >= SetPointTilt && head_pan < 0.4 && head_pan > -0.4 && ball_x != -1 && ball_y != -1) { //Stop(bola sudah dekat)
		countReadyKick++;
	} else { //Kejar Bola(bola masih jauh)
		countReadyKick = 0;
	}

	if (countReadyKick >= 1) { //5
		PxMove = 0.0; //jalan ditempat
		PyMove = errorfPan * 0.040; //0.045
		PaMove = errorfPan * 0.20; //0.30; //0.045
	} else {
		if (head_tilt < -1.5) {
			PxMove = kejar_max; //0.08
		} else if (head_tilt >= -1.5 && head_tilt < -1.4) {
			PxMove = kejar_mid; //0.07
		} else if (head_tilt > -1.0) {
			PxMove = lari; //0.05
		} else {
			PxMove = kejar; //0.06
		}
		//PxMove = errorfTilt * 0.1 * -13; //Robot besar 0.13, robot kecil 0.1
		//PxMove = 0.06 / -1.6 * posTilt; //0.04-0.06
		PyMove = errorfPan * 0.40;//0.125; //0.045
		PaMove = errorfPan * 0.30;//0.25; //0.35; //0.045
	}

	if (mode == 0) { // Mode differential walking
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
	else if (mode == 1) { // Mode omnidirectional walking
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("CCCCCCCC\n");
			Walk(PxMove, PyMove, PaMove);
		} else { 					//printf("DDDDDDDD\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
}


double errorPosX,
    errorPosY,
    PxMoveBallPos,
    PyMoveBallPos,
    PaMoveBallPos;
bool ballPos = false;
void robotStrategy::ballPositioning(double setPointX, double setPointY, double speed)
{
    errorPosX = head_pan - setPointX;
    errorPosY = head_tilt - setPointY;

    if ((errorPosX > -0.10 && errorPosX < 0.10) && (errorPosY > -0.10))
    { //&& errorPosY < 0.10)) { //sudah sesuai
        PyMoveBallPos = 0.00;
        PxMoveBallPos = 0.00;
        ballPos = true;
    }
    else
    { // belum sesuai
        ballPos = false;
        if ((head_pan >= 1.0 && head_tilt >= -1.2) || (head_pan <= -1.0 && head_tilt >= -1.2))
        { // bola disamping //pan tilt kircok (polar)
            PxMoveBallPos = -0.03;
            PyMoveBallPos = errorPosX * 0.08; // 0.12;
        }
        else
        {
            // Xmove
            if (head_tilt > setPointY)
            { //> (setPointY + 0.1)) { //kelebihan
                PxMoveBallPos = -0.03;
            }
            else if (head_tilt >= (setPointY - 0.1) && head_tilt <= setPointY)
            { //<= (setPointY + 0.1)) { //sudah dalam range
                PxMoveBallPos = 0.00;
            }
            else if (head_tilt >= (setPointY - 0.3) && head_tilt < (setPointY - 0.1))
            { // bola sudah dekat
                PxMoveBallPos = errorPosY * -speed;
                if (PxMoveBallPos >= 0.015)
                {
                    PxMoveBallPos = 0.015;
                }
                else if (PxMoveBallPos <= 0.00)
                {
                    PxMoveBallPos = 0.00;
                }
            }
            else
            {                                              // bola masih jauh
                PxMoveBallPos = head_tilt * (0.08 / -1.6); // 0.05
            }

            // Ymove
            if (head_tilt >= (setPointY - 0.03))
            { //> (setPointY + 0.1)) { //kelebihan
                PyMoveBallPos = 0.00;
            }
            else
            {
                if (head_pan >= (setPointX - 0.1) && head_pan <= (setPointX + 0.1))
                { // sudah dalam range
                    PyMoveBallPos = 0.00;
                }
                else
                {                                     // belum dalam range
                    PyMoveBallPos = errorPosX * 0.08; // 0.08;//0.12;
                }
            }
        }
    }
    Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
}

bool tendang, kiri, kanan = false;
void robotStrategy::kick(int mode)
{
    if (mode == 3 || mode == 4)
    {
        if (mode == 3)
        {
            kanan = true;
            kiri = false;
        } // arah kanan
        else if (mode == 4)
        {
            kiri = true;
            kanan = false;
        } // arah kiri
    }
    else
    {
        if (head_pan >= 0 && kanan == false && kiri == false)
        { // kiri
            kiri = true;
            kanan = false;
        }
        else if (head_pan <= 0 && kanan == false && kiri == false)
        { // kanan
            kanan = true;
            kiri = false;
        }
    }

    if (kiri)
    { // kiri
        // if (head_pan >= 0) { //kiri
        if (ballPos)
        { // printf("ball pos left true\n");
            motion("0");
            if (mode == 1 || mode == 2)
            {
                usleep(700000); // 6
                motion("1");
            }
            else if (mode == 3 || mode == 4)
            {
                sleep(1); // 10
                // motion("3");
                motion("4");
            }
            else if (mode == 5 || mode == 6)
            {
                usleep(700000);
                motion("5");
            }
            tendang = true;
        }
        else
        {
            ballPositioning(-p_pan_tendang, p_tilt_tendang, ball_positioning_speed); // 0.15
        }
    }
    if (kanan)
    { // kanan
        //} else { //kanan
        if (ballPos)
        { // printf("ball pos right true\n");
            motion("0");
            if (mode == 1 || mode == 2)
            {
                usleep(700000); // 7
                motion("2");
            }
            else if (mode == 3 || mode == 4)
            {
                sleep(1); // 10
                // motion("4");
                motion("3");
            }
            else if (mode == 5 || mode == 6)
            {
                usleep(700000); // 7
                motion("6");
            }
            tendang = true;
        }
        else
        {
            ballPositioning(p_pan_tendang, p_tilt_tendang, ball_positioning_speed); // 0.15
        }
    }
}

void robotStrategy::main_strategy()
{   
    if (kill == 1)
    {
        state = 1;
        RCLCPP_INFO(this->get_logger(), "Kill");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Run");
        state = 0;
    }

    if (last_state != state)
    {
        if (state == 1)
        {
            play = false;
        }
        else if (state == 0)
        {
            motion("8");
            motion("0");
            play = true;
            if (strategy_number != 4)
            {
                state_condition = first_state_condition;
            }
            else
            {
                state_condition = 150;
            }
        }last_state = state;
    }

    if (play)
    {
        switch (state_condition)
        {
        case 0:
            if (ballLost(20))
            {
                searchBallRectang(-1.6, -1.6, -0.8, 1.6);
            }
            else
            {
                tendang = false;
                state_condition = 1;
            }
            break;
        case 1:
            if (ballLost(20))
            {
                state_condition = 0;
            }
            else
            {
                followBall(0);
                if (tendang)
                {
                    state_condition = 0;
                }
                else
                {
                    kick(4);
                }
            }
            break;
        case 150:
            motion("0");
            if (ballLost(20))
            {
                searchBallRectang(-1.6, -1.6, -0.8, 1.6);
            }
            else
            {
                trackBall();
            }
            break;
        default:
            break;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotStrategy>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}