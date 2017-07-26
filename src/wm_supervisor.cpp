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

		safeVelocitySub_ = nh_.subscribe("cmd_vel", 10, &wmSupervisor::safeVelocityCallback, this);
		safeVelocityPub_ = nh_.advertise<geometry_msgs::Twist>("safe_cmd_vel", 1);

		FLWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/drive0/cmd0", 1);
		FRWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/drive1/cmd1", 1);
		RLWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/drive2/cmd2", 1);
		RRWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/drive3/cmd3", 1);

		estop_state_publisher = nh_.advertise<std_msgs::Bool>("supervisor/estop_state", 1);
	}

	wmSupervisor::~wmSupervisor() {
	}

	void wmSupervisor::safeVelocityCallback(const geometry_msgs::Twist& msg)
	{
		if (status_ != STOP)
		{
			safeVelocityPub_.publish(msg);
		}

		else
		{
			roboteq_msgs::Command cmd;
			cmd.mode = cmd.MODE_VELOCITY;
			cmd.setpoint = 0.0;

			FLWdrivePub_.publish(cmd);
			FRWdrivePub_.publish(cmd);
			RLWdrivePub_.publish(cmd);
			RRWdrivePub_.publish(cmd);
		}
		return;
	}


	void wmSupervisor::stateMachine() {
		switch (status_) {
			case RUN:
				if (!bReceivedState || bIsTimedOut) {
					status_ = STOP;
				}
				break;
			case STOP:
				if (bReceivedState && !bIsTimedOut) {
					status_ = RUN;
				}
				break;
		}
	}

	void wmSupervisor::actions() {
		static T_Status lastStatus = status_;

		if (lastStatus != status_) {
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
		if (status_ == RUN && !bRunning) {
			std_srvs::SetBool srv;
			srv.request.data = (unsigned char) true;
			eStopService_.call(srv);
			bRunning = true;
		} else if (status_ == STOP && bRunning) {
			std_srvs::SetBool srv;
			srv.request.data = (unsigned char) false;
			eStopService_.call(srv);
			bRunning = false;
		}
	}

	void wmSupervisor::loop() {
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
		//publish message
		message.data = status_ == RUN;
		estop_state_publisher.publish(message);
	}

} //end namespace wm

int main(int argc, char **argv) {
	//initialise ros
	ros::init(argc, argv, "wm_supervisor_node");
	ros::NodeHandle nh;

	//read parameters
	std::string tty = ros::param::param<std::string>("/" + ros::this_node::getName() + "/port_name", PORTNAME);

	//start serial
	ROS_INFO("Trying to open serial communication with %s serial port.", tty.c_str());
	wm::SerialManager serialManager(tty);

	//start supervisor
	wm::wmSupervisor supervisor(nh, &serialManager);

	ROS_INFO("Waiting for material E-Stop to communicate ...");
	while (serialManager.SerialAvailable());

	ROS_INFO("E-Stop Found !");

	ros::Rate rate(24.);

	//enter in ros loop
	while (ros::ok()) {
		ros::spinOnce();
		supervisor.loop();
		rate.sleep();
	}

	return 0;
}
