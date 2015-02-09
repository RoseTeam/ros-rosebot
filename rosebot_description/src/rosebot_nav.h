#ifndef _ROSEBOT_NAV_HH_
#define _ROSEBOT_NAV_HH_

#include <map>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include "rosebot.h"

class RosebotNav{

public:

	RosebotNav();

	shared_ptr<Rosebot> robot_;

    ros::NodeHandle nh_;

    // Update Rate
	double update_rate_;

	nav_msgs::Odometry odom_;
	geometry_msgs::Twist cmd_vel_;


	ros::Subscriber odometry_subscriber_;
	ros::Subscriber goal_subscriber_;
	ros::Publisher cmd_vel_publisher_;


	// DiffDrive stuff
	void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void publishCmdVel();
  
};

#endif


