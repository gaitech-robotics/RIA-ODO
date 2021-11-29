#ifndef _RC_CONTROL_H_
#define _RC_CONTROL_H_

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/Twist.h>

namespace ria_teleop
{
	class RC
	{
		public:
			RC(const ros::NodeHandle& n, const ros::NodeHandle& p);
			ros::NodeHandle _nh;
			ros::NodeHandle _pnh;
			ros::Subscriber rc_sub;
			ros::Publisher vel_pub;
			geometry_msgs::Twist vel;
			bool left_start,left_enable;
			bool right_start,right_enable;

		private:
			std::string _rc_topic,_cmd_vel_topic;
			double _linear_max,_angular_max;
			int _channels_num,_channels_max,_channels_min,_channels_mid;
			int _left_down2up,_left_left2right;
			int _right_down2up,_right_left2right;
			int _left_start,_left_mode;
			int _right_start,_right_mode;
			float k_l,b_l,k_a,b_a;
			void RCValueCB(const mavros_msgs::RCIn::ConstPtr& msg);
	};
}

#endif 
