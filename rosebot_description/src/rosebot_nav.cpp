
#include <algorithm>
#include <assert.h>

#include "rosebot_nav.h"
#include <math.h>

#include <tf/transform_broadcaster.h>

Vec coordsToVec(Vec v)
{
	return Vec(v[0]*1000, v[1]*1000);
}

Vec vecToCoords(Vec v)
{
	return v / 1000;
}

RosebotNav::RosebotNav()
{
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("a node ROS has not been initialized");
	}
	ROS_INFO("RosebotNav started");

	robot_ = shared_ptr<Rosebot>(new Rosebot(Vec(1000,250)));

	goal_subscriber_ 		= nh_.subscribe("goal", 5, &RosebotNav::goalCallback, this);
	odometry_subscriber_ 	= nh_.subscribe("odom", 1, &RosebotNav::odomCallback, this);
	cmd_vel_publisher_ 		= nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

}


void RosebotNav::publishCmdVel()
{

	double v = 0;
	double w = 0;
	if( true )
	{
		double deltaT = 1./update_rate_;

		Vec pos = robot_->pos_;
		//prev_v = copy.copy(robot_->_v_)
		//prev_orient = robot_->orient_ * 1 # copie
		robot_->PreBouge(deltaT*5);
		//next_orient = robot_->orient_
		v = robot_->_v_ *  robot_->orient_ * 0.001;

		if(v > 0)
			v = std::min<double>( robot_->v_max, v );
		else
			v = std::max<double>(-robot_->v_max, v);

		//robot_->Avance(deltaT,deltaT,0)
		//robot_->pos = pos

		w = robot_->_w_;
		if(w > 0)
			w = std::min<double>( w, robot_->w_max);
		else
			w = std::max<double>( w, -robot_->w_max);

		//print 'Goal', self.goal, robot_->but_
		//print robot_->orient_, robot_->_v_
		//robot_->orient_ = coordsToVec(prev_orient)/1000.
		//robot_->_v_ = prev_v

	}

	cmd_vel_.linear.x = v;
	cmd_vel_.angular.z = w;

	cmd_vel_publisher_.publish(cmd_vel_);
}

void RosebotNav::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	Vec pos = Vec(msg->pose.pose.position.x, msg->pose.pose.position.y)*1000;
	double vdirecte = msg->twist.twist.linear.x* 1000.;
	double vn = msg->twist.twist.linear.y * 1000.;

	Vec vit = robot_->orient_ * vdirecte + vn * robot_->orient_.nml();

	robot_->move(pos);
	robot_->_v_ = vit;
	tf::Quaternion qt( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
	double orient = qt.getAngle();
	robot_->orient_ = Vec(cos(orient), sin(orient));

	robot_->w = msg->twist.twist.angular.z;

	publishCmdVel();
}

void RosebotNav::goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	Vec goal = {msg->x, msg->y};
	double goal_orient = msg->theta;
	goal = coordsToVec(goal);
	robot_->AssBut(&goal);

	ROS_INFO("-D- goalCallback: %.2f,%.2f", msg->x, msg->y);
}
//############################################################

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RosebotNav");
	RosebotNav nav_node;
	ros::spin();

}
