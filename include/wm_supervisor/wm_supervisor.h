/*
 * wm_supervisor.h
 *
 *  Created on: May 11, 2016
 *      Author: xhache
 */

#ifndef WM_SUPERVISOR_H_
#define WM_SUPERVISOR_H_

#include <string>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include <ctime>
#include <boost/thread/lock_guard.hpp>

#define WATCHDOG 1000    //ms

namespace wm{

	const std::string STOP_STR = "stop";

	typedef enum {
		RUN,
		STOP,
		TIMED_OUT
	} T_Status;

	class wmSupervisor
	{
	public:
		wmSupervisor(ros::NodeHandle nh_);

		~wmSupervisor();

		bool stopSignalService(std_srvs::SetBool::Request &, std_srvs::SetBool::Response &);

		unsigned long getLastArduinoMessageTime();

		bool watchdogHandler();

	private:
		unsigned long lastArduinoMessageTime;

		void startSignalCallback(const std_msgs::Bool &);

		ros::NodeHandle nh_;
		ros::ServiceServer stopSignalSrv_;
		ros::ServiceClient eStopService_;
		ros::Subscriber startSignalSub_;
		T_Status status_;

		boost::mutex mtx_;
	};
} //namespace wm
#endif /* WM_SUPERVISOR_H_ */
