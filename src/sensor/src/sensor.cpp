#include "sensor.h"

robotSensor::robotSensor() : rclcpp::Node("robot_sensor")
{
    accelero_ = this->create_publisher<main_interface::msg::Accelero>("robot_accelero", 10);
    timer_1 = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&robotSensor::accelero, this));
    RCLCPP_INFO(this->get_logger(), "robot_accelero has been started.");

    gyroscope_ = this->create_publisher<main_interface::msg::Gyroscope>("robot_gyroscope", 10);
    timer_2 = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&robotSensor::gyroscope, this));
    RCLCPP_INFO(this->get_logger(), "robot_gyroscope has been started.");

    angle_ = this->create_publisher<main_interface::msg::Angle>("robot_angle", 10);
    timer_3 = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&robotSensor::angle, this));
    RCLCPP_INFO(this->get_logger(), "robot_angle has been started.");

    button_ = this->create_publisher<main_interface::msg::Button>("robot_button", 10);
    timer_4 = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&robotSensor::button, this));
    RCLCPP_INFO(this->get_logger(), "robot_vision has been started.");

    servo_pos_ = this->create_publisher<main_interface::msg::ServPos>("robot_servo", 10);
    timer_5 = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&robotSensor::servoPos, this));
    RCLCPP_INFO(this->get_logger(), "robot_servo has been started.");
}

void robotSensor::accelero()
{
    auto msg = main_interface::msg::Accelero();
    msg.x = accelero_x;
    msg.y = accelero_y;
    msg.z = accelero_z;
    accelero_->publish(msg);
}

void robotSensor::gyroscope()
{
    auto msg = main_interface::msg::Gyroscope();
    msg.x = gyroscope_x;
    msg.y = gyroscope_y;
    msg.z = gyroscope_z;
    gyroscope_->publish(msg);
}

void robotSensor::angle()
{
    auto msg = main_interface::msg::Angle();
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    angle_->publish(msg);
}

void robotSensor::button()
{
    auto msg = main_interface::msg::Button();
    msg.kill = kill;
    msg.strategy = strategy;
    button_->publish(msg);
}

void robotSensor::servoPos()
{
    auto msg = main_interface::msg::ServPos();
    msg.l_leg1 = Lleg1;
    msg.l_leg2 = Lleg2;
    msg.l_leg3 = Lleg3;
    msg.l_leg4 = Lleg4;
    msg.l_leg5 = Lleg5;
    msg.l_leg6 = Lleg6;
    msg.r_leg1 = Rleg1;
    msg.r_leg2 = Rleg2;
    msg.r_leg3 = Rleg3;
    msg.r_leg4 = Rleg4;
    msg.r_leg5 = Rleg5;
    msg.r_leg6 = Rleg6;
    servo_pos_->publish(msg);
}

void robotSensor::getSensor()
{
    char line[100];
    char *spr;
    int countParse = 0;

    FILE *outputfp;
    outputfp = fopen("/home/nvidia/BarelangFC_ROS2/src/motion/src/source_code/Player/SensorNote", "r");
    fscanf(outputfp, "%s", line);
    spr = strtok(line, ";");
    while (spr != NULL)
    {
        if (countParse == 0)
        {
            sscanf(spr, "%lf", &accelero_x);
        } // x					#dpn-blkng
        else if (countParse == 1)
        {
            sscanf(spr, "%lf", &accelero_y);
        } // y					#kiri-kanan
        else if (countParse == 2)
        {
            sscanf(spr, "%lf", &accelero_z);
        } // z
        else if (countParse == 3)
        {
            sscanf(spr, "%lf", &gyroscope_x);
        } //				#kiri-kanan
        else if (countParse == 4)
        {
            sscanf(spr, "%lf", &gyroscope_y);
        } //				#dpn-blkng
        else if (countParse == 5)
        {
            sscanf(spr, "%lf", &gyroscope_z);
        } //
        else if (countParse == 6)
        {
            sscanf(spr, "%lf", &roll);
        } // roll
        else if (countParse == 7)
        {
            sscanf(spr, "%lf", &pitch);
        } // pitch
        else if (countParse == 8)
        {
            sscanf(spr, "%lf", &yaw);
        } // yaw
        else if (countParse == 9)
        {
            sscanf(spr, "%d", &strategy);
        } //
        else if (countParse == 10)
        {
            sscanf(spr, "%d", &kill);
            countParse = -1;
        } //
        spr = strtok(NULL, ";");
        countParse++;
    }   
    fclose(outputfp);
}

void robotSensor::getServoPos()
{
    char line[200];
    char *sprs;
    int countParses = 0;

    FILE *outputfp;
    outputfp = fopen("/home/nvidia/BarelangFC_ROS2/src/motion/src/source_code/Player/PosNote", "r");
    fscanf(outputfp, "%s", line);
    sprs = strtok(line, ";");
    while (sprs != NULL)
    {
        if (countParses == 0)
        {
            sscanf(sprs, "%lf", &head1);
        } // ID=19
        else if (countParses == 1)
        {
            sscanf(sprs, "%lf", &head2);
        } // ID=20
        else if (countParses == 2)
        {
            sscanf(sprs, "%lf", &Larm1);
        } // ID=2
        else if (countParses == 3)
        {
            sscanf(sprs, "%lf", &Larm2);
        } // ID=4
        else if (countParses == 4)
        {
            sscanf(sprs, "%lf", &Larm3);
        } // ID=6
        else if (countParses == 5)
        {
            sscanf(sprs, "%lf", &Lleg1);
        } // ID=8
        else if (countParses == 6)
        {
            sscanf(sprs, "%lf", &Lleg2);
        } // ID=10
        else if (countParses == 7)
        {
            sscanf(sprs, "%lf", &Lleg3);
        } // ID=12
        else if (countParses == 8)
        {
            sscanf(sprs, "%lf", &Lleg4);
        } // ID=14
        else if (countParses == 9)
        {
            sscanf(sprs, "%lf", &Lleg5);
        } // ID=16
        else if (countParses == 10)
        {
            sscanf(sprs, "%lf", &Lleg6);
        } // ID=18
        else if (countParses == 11)
        {
            sscanf(sprs, "%lf", &Rleg1);
        } // ID=7
        else if (countParses == 12)
        {
            sscanf(sprs, "%lf", &Rleg2);
        } // ID=9
        else if (countParses == 13)
        {
            sscanf(sprs, "%lf", &Rleg3);
        } // ID=11
        else if (countParses == 14)
        {
            sscanf(sprs, "%lf", &Rleg4);
        } // ID=13
        else if (countParses == 15)
        {
            sscanf(sprs, "%lf", &Rleg5);
        } // ID=15
        else if (countParses == 16)
        {
            sscanf(sprs, "%lf", &Rleg6);
        } // ID=17
        else if (countParses == 17)
        {
            sscanf(sprs, "%lf", &Rarm1);
        } // ID=1
        else if (countParses == 18)
        {
            sscanf(sprs, "%lf", &Rarm2);
        } // ID=3
        else if (countParses == 19)
        {
            sscanf(sprs, "%lf", &Rarm3);
            countParses = -1;
        } // ID=5
        sprs = strtok(NULL, ";");
        countParses++;
    }
    fclose(outputfp);    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotSensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}