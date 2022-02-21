#ifndef _SET_RATE_H_
#define _SET_RATE_H_

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>

namespace ria_teleop
{
	class setRate
	{
		public:
			setRate(const ros::NodeHandle& n, const ros::NodeHandle& p);
			ros::NodeHandle _nh;
			ros::NodeHandle _pnh;
			ros::ServiceClient set_rate;
            mavros_msgs::StreamRate stream_rate;
            void callSetrateSrv();
	
		private:
			std::string _srv_topic;
			int _rate_value, _stream_id;
            
	};
}

#endif 
