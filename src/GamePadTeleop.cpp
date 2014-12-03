/*
 * GamePadTeleop.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: usama
 */

#include <gpadteleop/GamePadTeleop.h>
#include <gpadteleop/GamePadLevels.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

namespace gpadteleop{
	GamePadTeleop::GamePadTeleop()
	:n("~"),config_server(n),reconfiguring(false)
	{
		this->velocity.linear = 0;
		this->velocity.angular = 0;
		this->gamepad_params.brake = false;
		this->gamepad_params.lock = true;
		this->gamepad_params.lock_btn_released = true;
		this->gamepad_params.linmode_btn_released = true;
		this->gamepad_params.angmode_btn_released = true;
		this->gamepad_params.linspd_btn_released = true;
		this->gamepad_params.angspd_btn_released = true;
		this->gamepad_params.mode_angular = TELEOP_SPEED_SLOW;
		this->gamepad_params.mode_linear = TELEOP_SPEED_SLOW;
		this->gamepad_params.lin_mode_speed = false;
		this->gamepad_params.ang_mode_speed = false;
		// now to parameters and more
		this->n.getParam("publish_freq",this->gamepad_params.pub_freq);
		// re-mapped advertise publish and subscribe
		std::string resolvepubtopic = ros::names::remap("cmd_vel");
		std::string resolvesubtopic = ros::names::remap("joy");
		ROS_INFO("Publishing to topic: %s",resolvepubtopic.c_str());
		ROS_INFO("Subscribing to topic: %s",resolvesubtopic.c_str());
		this->pub = this->n.advertise<geometry_msgs::Twist>(resolvepubtopic,1,true);
		this->sub = this->n.subscribe<sensor_msgs::Joy>(resolvesubtopic,1,&GamePadTeleop::gamepad,this);
		// setting dynamic reconfigure
		this->config_server.setCallback( boost::bind(&GamePadTeleop::reconfig,this,_1,_2));
		// setting timer
		this->timer = this->n.createTimer(ros::Duration(double(1)/double(this->gamepad_params.pub_freq)),
				&GamePadTeleop::timed_publish,this );
	}
	GamePadTeleop::~GamePadTeleop() {
		this->timer.stop();
		this->config_server.clearCallback();
		this->pub.shutdown();
		this->sub.shutdown();
		this->n.shutdown();
	}
	void GamePadTeleop::reconfig(GamePadConfig &config,uint32_t level){
		/*
		 * to reconfigure different local parameters managed by dynamic reconfigure
		 */
		this->reconfiguring = true;
		boost::mutex::scoped_lock lock(this->mutx_configure);
		// checking for different levels and doing appropriate actions
		if ( level & GamePadLevels::GPADLEVEL_AXIS_LIN ){
			this->gamepad_params.axis_linear = config.axis_linear;
			ROS_INFO("Linear Axis is : %d",this->gamepad_params.axis_linear);
		} if ( level & GamePadLevels::GPADLEVEL_AXIS_LINSPEED ){
			this->gamepad_params.axis_linear_speed = config.axis_linear_speed;
			ROS_INFO("Linear Speed Axis is : %d",this->gamepad_params.axis_linear_speed);
		} if ( level & GamePadLevels::GPADLEVEL_AXIS_ANG ){
			this->gamepad_params.axis_angular = config.axis_angular;
			ROS_INFO("Angular Axis is : %d",this->gamepad_params.axis_angular);
		} if ( level & GamePadLevels::GPADLEVEL_AXIS_ANGSPEED){
			this->gamepad_params.axis_angular_speed = config.axis_angular_speed;
			ROS_INFO("Angular Speed Axis is : %d",this->gamepad_params.axis_angular_speed);
		} if ( level & GamePadLevels::GPADLEVEL_BTN_LOCK){
			this->gamepad_params.btn_lock = config.btn_robot_lock;
			ROS_INFO("Lock Toggle Button is : %d",this->gamepad_params.btn_lock);
		} if ( level & GamePadLevels::GPADLEVEL_BTN_SFTBRK){
			this->gamepad_params.btn_soft_brake = config.btn_soft_brake;
			ROS_INFO("Soft Brake Button is : %d",this->gamepad_params.btn_soft_brake);
		} if ( level & GamePadLevels::GPADLEVEL_BTN_HRDBRK){
			this->gamepad_params.btn_hard_brake = config.btn_hard_brake;
			ROS_INFO("Hard Brake Button is : %d",this->gamepad_params.btn_hard_brake);
		} if ( level & GamePadLevels::GPADLEVEL_BTN_TGLLIN){
			this->gamepad_params.btn_speedmode_lin = config.btn_speed_mode_lin;
			ROS_INFO("Linear Speed Mode Button is : %d",this->gamepad_params.btn_speedmode_lin);
		} if ( level & GamePadLevels::GPADLEVEL_BTN_TGLANG){
			this->gamepad_params.btn_speedmode_ang = config.btn_speed_mode_ang;
			ROS_INFO("Angular Speed Mode Button is : %d",this->gamepad_params.btn_speedmode_ang);
		} if ( level & GamePadLevels::GPADLEVEL_BTN_LINMODE){
			this->gamepad_params.btn_lin_mode = config.btn_lin_mode;
			ROS_INFO("Linear Level Button is : %d",this->gamepad_params.btn_lin_mode);
		} if ( level & GamePadLevels::GPADLEVEL_BTN_ANGMODE){
			this->gamepad_params.btn_ang_mode = config.btn_ang_mode;
			ROS_INFO("Angular Level Button is : %d",this->gamepad_params.btn_ang_mode);
		} if ( level & GamePadLevels::GPADLEVEL_MAXLIN) {
			this->gamepad_params.cap_lin_slow = config.percent_lin_slow * config.max_lin_vel;
			this->gamepad_params.cap_lin_medium = config.percent_lin_medium * config.max_lin_vel;
			this->gamepad_params.cap_lin_fast = config.percent_lin_fast * config.max_lin_vel;
			this->gamepad_params.percent_lin_inc = config.percent_lin_inc;
			ROS_INFO("Changing maximum linear velocities - fast: %f medium: %f slow: %f increment: %f",
					this->gamepad_params.cap_lin_fast,this->gamepad_params.cap_lin_medium,
					this->gamepad_params.cap_lin_slow,this->gamepad_params.percent_lin_inc);
		} if ( level & GamePadLevels::GPADLEVEL_MAXANG ){
			this->gamepad_params.cap_ang_slow = config.percent_ang_slow * config.max_ang_vel;
			this->gamepad_params.cap_ang_medium = config.percent_ang_medium * config.max_ang_vel;
			this->gamepad_params.cap_ang_fast = config.percent_ang_fast * config.max_ang_vel;
			this->gamepad_params.percent_ang_inc = config.percent_ang_inc;
			ROS_INFO("Changing maximum linear velocities - fast: %f medium: %f slow: %f increment: %f",
					this->gamepad_params.cap_ang_fast,this->gamepad_params.cap_ang_medium,
					this->gamepad_params.cap_ang_slow,this->gamepad_params.percent_ang_inc);
		} if ( level & GamePadLevels::GPADLEVEL_MOTORE ){
			this->gamepad_params.srv_motors_enable = config.srv_motors_enable;
			ROS_INFO("Service for enabling motors is : %s",this->gamepad_params.srv_motors_enable.c_str());
		} if ( level & GamePadLevels::GPADLEVEL_MOTORD){
			this->gamepad_params.srv_motors_disable = config.srv_motors_disable;
			ROS_INFO("Service for disabling motors is : %s",this->gamepad_params.srv_motors_disable.c_str());
		} if ( level & GamePadLevels::GPADLEVEL_PUBFREQ){
			this->gamepad_params.pub_freq = config.publish_freq;
			this->timer.setPeriod(ros::Duration(double(1)/double(this->gamepad_params.pub_freq)));
			ROS_INFO("New Publish Frequency is : %d Hz",this->gamepad_params.pub_freq);
		}

		this->reconfiguring = false;
	}
	void GamePadTeleop::timed_publish(const ros::TimerEvent& te){
		/*
		 * publishes command velocity in message of type geometry_msgs/Twist to
		 * appropriate topic (usually cmd_vel)
		*/
		geometry_msgs::Twist twist;

		twist.linear.x = this->velocity.linear;
		twist.angular.z = this->velocity.angular;

		this->pub.publish(twist);
	}
	void GamePadTeleop::gamepad(const sensor_msgs::Joy::ConstPtr& gpad){
		/*
		 * gets the current state of game-pad and set appropriate state of object
		 */
		// CHECK FOR BUTTONS FIRST
		this->checkButtons(gpad);
		// CHECK FOR AXES
		if ( !(this->gamepad_params.lock) && !(this->gamepad_params.brake) ){
			// linear
			if ( this->gamepad_params.lin_mode_speed ){
				double increment = gpad->axes[this->gamepad_params.axis_linear_speed] * this->max_linear() * this->gamepad_params.percent_lin_inc;
				double newvel = this->velocity.linear + increment;
				this->velocity.linear = this->double_bounded(newvel,this->max_linear());
			} else {
				double percentage = gpad->axes[this->gamepad_params.axis_linear];
				this->velocity.linear = percentage * this->max_linear();
			}
			// angular
			if ( this->gamepad_params.ang_mode_speed ){
				double increment = gpad->axes[this->gamepad_params.axis_angular_speed] * this->max_angular() * this->gamepad_params.percent_ang_inc;
				double newvel = this->velocity.angular + increment;
				this->velocity.angular = this->double_bounded(newvel,this->max_angular());
			} else {
				double percentage = gpad->axes[this->gamepad_params.axis_angular];
				this->velocity.angular = percentage * this->max_angular();
			}
		}
	}
	// private methods
	void GamePadTeleop::checkButtons(const sensor_msgs::Joy::ConstPtr& gpad ){
		// brakes and lock
		this->applyBrakes(gpad->buttons[this->gamepad_params.btn_soft_brake] || gpad->buttons[this->gamepad_params.btn_hard_brake]);
		if ( (gpad->buttons[this->gamepad_params.btn_hard_brake] == 1) && !(this->gamepad_params.lock)){
			this->disableRobot();
		}
		if ( (this->gamepad_params.lock_btn_released) && (gpad->buttons[this->gamepad_params.btn_lock]) ){
			// now toggle
			if ( this->gamepad_params.lock ){
				this->enableRobot();
			} else {
				this->haltRobot();
				this->disableRobot();
				this->gamepad_params.brake = false;
			}
			this->gamepad_params.lock_btn_released = false;
		}
		// max speed toggle
		if ( (this->gamepad_params.linmode_btn_released) && (gpad->buttons[this->gamepad_params.btn_lin_mode]) ){
			this->toggleLinSpeedMax();
			this->gamepad_params.linmode_btn_released = false;
		}
		if ( (this->gamepad_params.angmode_btn_released) && (gpad->buttons[this->gamepad_params.btn_ang_mode]) ){
			this->toggleAngSpeedMax();
			this->gamepad_params.angmode_btn_released = false;
		}
		// speed mode toggle
		if ( (this->gamepad_params.linspd_btn_released) && (gpad->buttons[this->gamepad_params.btn_speedmode_lin])){
			this->gamepad_params.lin_mode_speed = (this->gamepad_params.lin_mode_speed)? 0:1;
			this->gamepad_params.linspd_btn_released = false;
		}
		if ( (this->gamepad_params.angspd_btn_released) && (gpad->buttons[this->gamepad_params.btn_speedmode_ang])){
			this->gamepad_params.ang_mode_speed = (this->gamepad_params.ang_mode_speed)? 0:1;
			this->gamepad_params.angspd_btn_released = false;
		}
		// release buttons
		if ( !(this->gamepad_params.lock_btn_released) && !(gpad->buttons[this->gamepad_params.btn_lock]) ){
			this->gamepad_params.lock_btn_released = true;
		}
		if ( !(this->gamepad_params.linmode_btn_released) && !(gpad->buttons[this->gamepad_params.btn_lin_mode]) ){
			this->gamepad_params.linmode_btn_released = true;
		}
		if ( !(this->gamepad_params.angmode_btn_released) && !(gpad->buttons[this->gamepad_params.btn_ang_mode]) ){
			this->gamepad_params.angmode_btn_released = true;
		}
		if ( !(this->gamepad_params.linspd_btn_released) && !(gpad->buttons[this->gamepad_params.btn_speedmode_lin])){
			this->gamepad_params.linspd_btn_released = true;
		}
		if ( !(this->gamepad_params.angspd_btn_released) && !(gpad->buttons[this->gamepad_params.btn_speedmode_ang])){
			this->gamepad_params.angspd_btn_released = true;
		}
	}
	void GamePadTeleop::toggleLinSpeedMax(){
		switch(this->gamepad_params.mode_linear){
		case TELEOP_SPEED_SLOW:
			this->gamepad_params.mode_linear = TELEOP_SPEED_MEDIUM;
			ROS_INFO("Linear Speed Mode changed to MEDIUM, cap is %f m/s",this->gamepad_params.cap_lin_medium);
			break;
		case TELEOP_SPEED_MEDIUM:
			this->gamepad_params.mode_linear = TELEOP_SPEED_FAST;
			ROS_INFO("Linear Speed Mode changed to FAST, cap is %f m/s",this->gamepad_params.cap_lin_fast);
			break;
		case TELEOP_SPEED_FAST:
			this->gamepad_params.mode_linear = TELEOP_SPEED_SLOW;
			ROS_INFO("Linear Speed Mode changed to SLOW, cap is %f m/s",this->gamepad_params.cap_lin_slow);
			break;
		default:
			break;
		}
	}
	void GamePadTeleop::toggleAngSpeedMax(){
		switch(this->gamepad_params.mode_angular){
		case TELEOP_SPEED_SLOW:
			this->gamepad_params.mode_angular = TELEOP_SPEED_MEDIUM;
			ROS_INFO("Angular Speed Mode changed to MEDIUM, cap is %f rad/s",this->gamepad_params.cap_ang_medium);
			break;
		case TELEOP_SPEED_MEDIUM:
			this->gamepad_params.mode_angular = TELEOP_SPEED_FAST;
			ROS_INFO("Angular Speed Mode changed to FAST, cap is %f rad/s",this->gamepad_params.cap_ang_fast);
			break;
		case TELEOP_SPEED_FAST:
			this->gamepad_params.mode_angular = TELEOP_SPEED_SLOW;
			ROS_INFO("Angular Speed Mode changed to SLOW, cap is %f rad/s",this->gamepad_params.cap_ang_slow);
			break;
		default:
			break;
		}
	}
	void GamePadTeleop::applyBrakes(const bool apply){
		if ( apply ){
			if (this->gamepad_params.brake != true){
				this->haltRobot();
			}
		} else {
			if ( this->gamepad_params.brake != false){
				this->gamepad_params.brake = false;
			}
		}
	}
	void GamePadTeleop::haltRobot(){
		this->gamepad_params.brake = true;
		boost::mutex::scoped_lock lock(this->mtx_velocity);
		this->velocity.linear = 0;
		this->velocity.angular = 0;
	}
	void GamePadTeleop::enableRobot(){
		std_srvs::Empty empty;
		this->gamepad_params.lock = false;
		ros::service::call(this->gamepad_params.srv_motors_enable,empty);
		ROS_INFO("Robot Enabled!");
	}
	void GamePadTeleop::disableRobot(){
		std_srvs::Empty empty;
		ros::service::call(this->gamepad_params.srv_motors_disable,empty);
		this->gamepad_params.lock = true;
		ROS_INFO("Robot Disabled!");
	}
	double GamePadTeleop::max_linear(){
		double vel = 0;
		switch ( this->gamepad_params.mode_linear){
		case TELEOP_SPEED_SLOW:
			vel = this->gamepad_params.cap_lin_slow;
			break;
		case TELEOP_SPEED_MEDIUM:
			vel = this->gamepad_params.cap_lin_medium;
			break;
		case TELEOP_SPEED_FAST:
			vel = this->gamepad_params.cap_lin_fast;
			break;
		default:
			break;
		}
		return vel;
	}
	double GamePadTeleop::max_angular(){
		double vel = 0;
		switch ( this->gamepad_params.mode_angular){
		case TELEOP_SPEED_SLOW:
			vel = this->gamepad_params.cap_ang_slow;
			break;
		case TELEOP_SPEED_MEDIUM:
			vel = this->gamepad_params.cap_ang_medium;
			break;
		case TELEOP_SPEED_FAST:
			vel = this->gamepad_params.cap_ang_fast;
			break;
		default:
			break;
		}
		return vel;
	}
	double GamePadTeleop::double_bounded(const double num,const double check) const{
		double checkvel = 0;
		double signedcheck = 0;
		if ( num < 0 ){
			checkvel = -1.0 * num;
			signedcheck = -1.0 * check;
		} else {
			checkvel = num;
			signedcheck = check;
		}
		 double retvel = (checkvel > check)? signedcheck : num;
		 return retvel;
	}
}

