/*
 * GamePadTeleop.h
 *
 *  Created on: Oct 10, 2014
 *      Author: usama
 */
#ifndef GAMEPADTELEOP_H_
#define GAMEPADTELEOP_H_
/*
 * RECONFIGUREABLE GAMEPAD TELEOPERATION
 *
 */
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <gpadteleop/GamePadConfig.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread/mutex.hpp>
#include <string>

#define TELEOP_SPEED_SLOW 1
#define TELEOP_SPEED_MEDIUM 2
#define TELEOP_SPEED_FAST 3

namespace gpadteleop{

struct Velocity_2D {
	double linear;
	double angular;
};

struct GamePadTeleop_params {
	int axis_linear;
	int axis_angular;
	int axis_linear_speed;
	int axis_angular_speed;
	int btn_soft_brake;
	int btn_hard_brake;
	int btn_lin_mode;
	bool linmode_btn_released;
	bool angmode_btn_released;
	int btn_ang_mode;
	int btn_lock;
	bool lock_btn_released;
	bool linspd_btn_released;
	bool angspd_btn_released;
	int btn_speedmode_lin;
	int btn_speedmode_ang;

	short mode_linear;
	short mode_angular;

	bool lin_mode_speed;
	bool ang_mode_speed;

	bool brake;
	bool lock;

	double percent_lin_inc;	// increment of linear speed in speed mode
	double percent_ang_inc; // increment of angular speed in speed mode

	double cap_lin_slow;	// in m/s
	double cap_lin_medium;	// in m/s
	double cap_lin_fast;	// in m/s
	double cap_ang_slow;	// in rad/s
	double cap_ang_medium;	// in rad/s
	double cap_ang_fast;	// in rad/s

	int pub_freq;

	std::string srv_motors_enable;
	std::string srv_motors_disable;
};


class GamePadTeleop {
public:
	GamePadTeleop();
	virtual ~GamePadTeleop();

	// reconfigure
	void reconfig(GamePadConfig &config,uint32_t level);	// callback to reconfigure parameter change
	void timed_publish(const ros::TimerEvent& te);			// publishing of command velocities at specific publishing frequency
	void gamepad(const sensor_msgs::Joy::ConstPtr& gpad);	// callback for handling gamepad commands
protected:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

	dynamic_reconfigure::Server<GamePadConfig> config_server;
	bool reconfiguring;
	boost::mutex mutx_configure;
	boost::mutex mtx_velocity;
	ros::Timer timer;

	GamePadTeleop_params gamepad_params;
	Velocity_2D velocity;
private:
	void checkButtons(const sensor_msgs::Joy::ConstPtr& gpad);
	void toggleLinSpeedMax();
	void toggleAngSpeedMax();
	void applyBrakes(const bool apply);
	void haltRobot();
	void enableRobot();
	void disableRobot();
	double max_linear();
	double max_angular();
	double double_bounded(const double num,const double check) const;
};

}

#endif /* GAMEPADTELEOP_H_ */
