/*
 * GPadTeleop2.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: usama
 *      email:	usiraj@gmail.com
 *      Improved version of Teleoperation of robot using gamepad.
 */

#include <gpadteleop/GamePadTeleop.h>

int main(int argc,char** argv){
	ros::init(argc,argv,"GPadTeleop");
	gpadteleop::GamePadTeleop teleop;
	ros::spin();
	return 0;
}

