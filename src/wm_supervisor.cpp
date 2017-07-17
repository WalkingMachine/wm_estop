/*
 * wm_supervisor.cpp
 *
 *  Created on: May 11, 2016
 *      Author: xhache
 */

#include "wm_supervisor.h"

namespace wm {
	wmSupervisor::wmSupervisor(ros::NodeHandle nh_, wm::SerialManager *manager) {
		//Initialise to Running status
		status_ = RUN;
		bRunning = true;

		serialManager = manager;
		eStopService_ = nh_.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("estop_service");
	}

	wmSupervisor::~wmSupervisor() {
	}

	void wmSupervisor::stateMachine() {
		switch(status_){
			case RUN:
				if(!bReceivedState || bIsTimedOut){
					status_ = STOP;
				}
				break;
			case STOP:
				if(bReceivedState && !bIsTimedOut){
					status_ = RUN;
				}
				break;
		}
	}

	void wmSupervisor::actions() {
		static T_Status lastStatus = status_;

		if(lastStatus != status_) {
			lastStatus = status_;
			switch (status_) {
				case RUN:
					ROS_WARN("Received start signal.");
					break;
				case STOP:
					ROS_WARN("Received stop signal.");
					break;
			}
		}
		if(status_ == RUN && !bRunning){
			std_srvs::SetBool srv;
			srv.request.data = (unsigned char)true;
			eStopService_.call(srv);
			bRunning = true;
		}else if(status_ == STOP && bRunning){
			std_srvs::SetBool srv;
			srv.request.data = (unsigned char)false;
			eStopService_.call(srv);
			bRunning = false;
		}
	}

	void wmSupervisor::loop(){
		//read serial
		bReceivedState = serialManager->serialHandler();
		//read timeout
		bIsTimedOut = serialManager->watchDogHandler();
		//manage state machine
		stateMachine();
		//actions
		actions();
		//send feedback
		serialManager->sendStatus(bRunning);
	}

} //end namespace wm

int main(int argc, char **argv){
	//initialise ros
	ros::init(argc, argv, "wm_supervisor_node");
	ros::NodeHandle nh;

	//read parameters
	std::string tty = ros::param::param<std::string>("/" + ros::this_node::getName() + "/port_name", PORTNAME);

	//start serial
	ROS_INFO("Triing to open serial communication with %s serial port.", tty);
	wm::SerialManager serialManager(tty);

	//start supervisor
	wm::wmSupervisor supervisor(nh, &serialManager);

	ROS_INFO("Waiting for material E-Stop to communicate ...");
	while(serialManager.SerialAvailable());

	ROS_INFO("E-Stop Found !");

	//enter in ros loop
	while (ros::ok()) {
		ros::spinOnce();
		supervisor.loop();
	}

	return 0;
}
