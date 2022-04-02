#include "vision.h"

robotVision::robotVision() : rclcpp::Node("robot_vision")
{
	processing_vision();

    vision_ = this->create_publisher<main_interface::msg::VisionBall>("robot_vision", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&robotVision::BallCoor, this));
    RCLCPP_INFO(this->get_logger(), "robot_vision has been started.");
}

void robotVision::BallCoor()
{
    auto msg = main_interface::msg::VisionBall();
    msg.x = Ball_X;
    msg.y = Ball_Y;
    vision_->publish(msg);
}

void die(char *s) {
    perror(s);
    exit(1);
}

void initialize_vision() {
    if ((v=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        die("socket");
    }
    memset((char *) &si_meV, 0, sizeof(si_meV));
    si_meV.sin_family = AF_INET;
    si_meV.sin_port = htons(PORT_VISION);
    si_meV.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(v, (struct sockaddr*)&si_meV, sizeof(si_meV)) == -1) {
        die("bind");
    }
}

void robotVision::processing_vision() {
	fflush(stdout);
	bzero(recvVision,sizeof(recvVision));
	if ((recv_lenV = recvfrom(v, recvVision, BUFLEN_VISION, 0, (struct sockaddr *) &si_otherV, &slenV)) == -1) {
		die("recvfrom()");
	}
	printf("Data BallCoor : %s\n",recvVision);
	index_vision = 0;
	parseVision = strtok (recvVision,",");

	while (parseVision != NULL) {
		index_vision++;
		if (index_vision == 1)      { sscanf(parseVision,"%d", &Ball_X); }
		else if (index_vision == 2) { sscanf(parseVision,"%d", &Ball_Y); }
		else if (index_vision == 3) { sscanf(parseVision,"%d", &Ball_D); }
		else if (index_vision == 4) { sscanf(parseVision,"%d", &Goal_X); }
		else if (index_vision == 5) { sscanf(parseVision,"%d", &Goal_Y); }
		else if (index_vision == 6) { sscanf(parseVision,"%d", &Goal_D); }
		//else if (index_vision == 4) { sscanf(parseVision,"%d", &Goal_LD); }
		//else if (index_vision == 5) { sscanf(parseVision,"%d", &Goal_RD); }
		//else if (index_vision == 8) { sscanf(parseVision,"%d", &Pinalty_D); }
		//else if (index_vision == 9) { sscanf(parseVision,"%d", &Lcross_LD); }
		//else if (index_vision == 10) { sscanf(parseVision,"%d", &Lcross_RD); }
		//else if (index_vision == 11) { sscanf(parseVision,"%d", &Xcross_LD); }
		//else if (index_vision == 12) { sscanf(parseVision,"%d", &Xcross_RD); }
		//else if (index_vision == 13) { sscanf(parseVision,"%d", &Tcross_LD); }
		//else if (index_vision == 14) { sscanf(parseVision,"%d", &Tcross_RD); }
		parseVision = strtok (NULL,",");
	}
	if (Ball_D != NULL){
		cntData = 0;
		dataAkhir[0] = Ball_D;
	}
	if (Ball_D == NULL){
		Ball_D = dataAkhir[0];
	}
/*		if (Ball_X == -1 && Ball_Y == -1){
		if(cntData > 30){
			Ball_D = 0;
		}else{
			cntData++;
		}
	}
*/
	if (Goal_D != NULL){
		dataAkhir[1] = Goal_D;
	}
	if (Goal_D == NULL){
		Goal_D = dataAkhir[1];
	}
/*
	if (Goal_RD != NULL){
		dataAkhir[2] = Goal_RD;
	}
	if (Goal_RD == NULL){
		Goal_RD = dataAkhir[2];
	}
*/

	if (Ball_D == -2147483648 || Ball_D == 2147483647) {
		Ball_D = 0;
	}
	if (Goal_LD == -2147483648 || Goal_LD == 2147483647) {
		Goal_LD = 0;
	}
	if (Goal_RD == -2147483648 || Goal_RD == 2147483647) {
		Goal_RD = 0;
	}
	if (Pinalty_D == -2147483648 || Pinalty_D == 2147483647) {
		Pinalty_D = 0;
	}
	if (Lcross_LD == -2147483648 || Lcross_LD == 2147483647) {
		Lcross_LD = 0;
	}
	if (Lcross_RD == -2147483648 || Lcross_RD == 2147483647) {
		Lcross_RD = 0;
	}
	if (Xcross_LD == -2147483648 || Xcross_LD == 2147483647) {
		Xcross_LD = 0;
	}
	if (Xcross_RD == -2147483648 || Xcross_RD == 2147483647) {
		Xcross_RD = 0;
	}
	if (Tcross_LD == -2147483648 || Tcross_LD == 2147483647) {
		Tcross_LD = 0;
	}
	if (Tcross_RD == -2147483648 || Tcross_RD == 2147483647) {
		Tcross_RD = 0;
	}

}

int main(int argc, char **argv)
{
	initialize_vision();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotVision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}