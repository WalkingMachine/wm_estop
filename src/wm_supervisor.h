/*
 * wm_supervisor.h
 *
 *  Created on: May 11, 2016
 *      Author: xhache
 */

#ifndef WM_SUPERVISOR_H_
#define WM_SUPERVISOR_H_

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <ctime>
#include <string>
#include "SerialManager.h"
#include "roboteq_msgs/Command.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"

#define PORTNAME "/dev/ttyUSB0"

namespace wm{

	typedef enum {
		RUN,
		STOP
	} T_Status;

	class wmSupervisor
	{
	public:
		wmSupervisor(ros::NodeHandle nh_, wm::SerialManager *manager);
		~wmSupervisor();

		void loop();

	private:
		ros::NodeHandle nh_;
		ros::ServiceClient eStopService_;

		SerialManager *serialManager;

		//inputs
		bool bReceivedState;
		bool bIsTimedOut;
		bool bRunning;
		ros::Subscriber safeVelocitySub_;
		ros::Publisher safeVelocityPub_, FLWdrivePub_, FRWdrivePub_, RLWdrivePub_, RRWdrivePub_;

		//state machine
		T_Status status_;

		//manage estop state machine
		void stateMachine();
		//manage estop reactions
		void actions();

		void safeVelocityCallback(const geometry_msgs::Twist& msg);


	};
} //namespace wm
#endif /* WM_SUPERVISOR_H_ */
