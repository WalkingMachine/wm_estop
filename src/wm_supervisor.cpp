/*
 * wm_supervisor.cpp
 *
 *  Created on: May 11, 2016
 *      Author: xhache
 */

#include "wm_supervisor/wm_supervisor.h"

namespace wm {
	wmSupervisor::wmSupervisor(ros::NodeHandle nh_) {
		//Initialise to Running status
		status_ = RUN;
		lastArduinoMessageTime = 0;
		startSignalSub_ = nh_.subscribe("start_button_msg", 1, &wmSupervisor::startSignalCallback,
		                                this);       //Arduino E-Stop publish actual state on this every time
		stopSignalSrv_ = nh_.advertiseService("safety_stop_srv", &wmSupervisor::stopSignalService,
		                                      this);       //Arduino call this service when E-Stop is engaged
		eStopService_ = nh_.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("e-stop_service");

	}

	wmSupervisor::~wmSupervisor() {
	}

//	bool wmSupervisor::robotStatusService(wm_supervisor::robotStatus::Request& req, wm_supervisor::robotStatus::Response& res)
//	{
//		boost::lock_guard<boost::mutex> guard(mtx_);
//
//		if (status_ == wm::STATUS_OK)
//		{
//			res.status = res.STATUS_OK;
//		}
//		else if (status_ == wm::STOP_COMMANDED)
//		{
//			res.status = res.STOP_COMMANDED;
//		}
//
//		return true;
//	}

	bool wmSupervisor::stopSignalService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
		if (req.data) {
			ROS_WARN("Received stop signal.");
			status_ = STOP;
			res.success = true;
		} else {
			ROS_INFO("Received start signal.");
			status_ = RUN;
			res.success = true;
		}
		return true;
	}

	void wmSupervisor::startSignalCallback(const std_msgs::Bool &msg) {
		time_t timer;
		time(&timer);
		//read actual system time
		lastArduinoMessageTime = (unsigned long) timer * 1000;  //time in ms

		if (!msg.data && status_ == RUN) {
			ROS_WARN("Received stop signal.");
			status_ = STOP;
		}
	}

	bool wmSupervisor::watchdogHandler() {
		if (status_ != TIMED_OUT) {
			time_t timer;
			time(&timer);
			//read actual system time
			unsigned long sysTime = (unsigned long) timer * 1000;  //time in ms

			if (sysTime - getLastArduinoMessageTime() > WATCHDOG) {
				ROS_WARN("Arduino Timed Out !!");
				status_ = TIMED_OUT;
			}
		}
		return status_ != TIMED_OUT;
	}

	unsigned long wmSupervisor::getLastArduinoMessageTime() { return lastArduinoMessageTime; }

} //end namespace wm

int main(int argc, char **argv)
{
	//initialise ros
	ros::init(argc, argv, "wm_supervisor_node");
	ros::NodeHandle nh;

	//start supervisor
	wm::wmSupervisor supervisor(nh);

	ROS_INFO("Waiting for material E-Stop");
	while (supervisor.getLastArduinoMessageTime() == 0) {
		ros::spinOnce();
	}

	ROS_INFO("E-Stop Found !");

	//enter in ros loop
	while (ros::ok()) {
		supervisor.watchdogHandler();
		ros::spinOnce();
	}

	return 0;
}
