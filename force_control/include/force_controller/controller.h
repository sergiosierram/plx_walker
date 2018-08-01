#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#define DEFAULT_LEFT_AXIS_X			1
#define DEFAULT_LEFT_AXIS_Y			0
#define DEFAULT_RIGHT_AXIS_X		2
#define DEFAULT_RIGHT_AXIS_Y	    3
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0

class SimpleController{
public:
		SimpleController();
		double l_scale_, a_scale_;
		int left_x_, left_y_, right_x_, right_y_;
		ros::NodeHandle nh_;
		ros::Publisher vel_pub_;
		ros::Subscriber frc_left_sub_, frc_right_sub;
		std::string cmd_vel_topic_, frc_left_topic_, frc_right_topic_;
		void frcCallback(const geometry_msgs::Wrench::ConstPtr& frc_l, const geometry_msgs::Wrench::ConstPtr& frc_r);

};
#endif
