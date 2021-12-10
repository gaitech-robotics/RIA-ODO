/*
Copyright (c) 2021 Gaitech Robotics
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "riaodo_teleop/rc_control.h"

namespace ria_teleop
{
	RC::RC(const ros::NodeHandle& n, const ros::NodeHandle& p)
    :
    _nh(n),_pnh(p),
	left_start(false),left_enable(false),
	right_start(false),right_enable(false),
	k_l(0),b_l(0),
	k_a(0),b_a(0)
	{
		_pnh.param<std::string>("rc_topic", _rc_topic , "/mavros/rc/in");
		_pnh.param<std::string>("cmd_vel_topic", _cmd_vel_topic, "/cmd_vel");
		_pnh.param<double>("linear_max", _linear_max, 1.5);
		_pnh.param<double>("angular_max", _angular_max, 1.0);
		_pnh.param<int>("channels_num", _channels_num, 8);
		_pnh.param<int>("channels_value_max", _channels_max, 1990);
		_pnh.param<int>("channels_value_min", _channels_min, 970);
		_pnh.param<bool>("add_error", _add_error, false);
		_pnh.param<int>("error_value", _error_value, 5);
		_pnh.param<int>("left_down2up", _left_down2up, 2);
		_pnh.param<int>("left_left2right", _left_left2right, 3);
		_pnh.param<int>("right_down2up", _right_down2up, 1);
		_pnh.param<int>("right_left2right", _right_left2right, 0);
		_pnh.param<int>("left_start", _left_start, 6);
		_pnh.param<int>("left_mode", _left_mode, 7);
		_pnh.param<int>("right_start", _right_start, 5);
		_pnh.param<int>("right_mode", _right_mode, 4);

		if(_channels_max <= _channels_min){
			ROS_ERROR_STREAM("Channels max value should set bigger than channels min value!");
		}

		_channels_mid = _channels_min + (_channels_max - _channels_min)/2;
		vel.linear.x = 0;
		vel.linear.y = 0;
		vel.linear.z = 0;
		vel.angular.x = 0;
		vel.angular.x = 0;
		vel.angular.x = 0;

		k_l = (2 * _linear_max)/(_channels_max - _channels_min);
		b_l = _linear_max - (k_l * _channels_max);
		k_a = (2 * _angular_max)/(_channels_max - _channels_min);
		b_a = _angular_max - (k_a * _channels_max);


		vel_pub = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic,50,true);
		rc_sub = _nh.subscribe<mavros_msgs::RCIn>(_rc_topic, 50, &RC::RCValueCB, this);	
		
	}

	void RC::RCValueCB(const mavros_msgs::RCIn::ConstPtr& msg){
		int channels_value[_channels_num];
		for(int i=0;i<_channels_num;i++){
			channels_value[i] = msg->channels[i];
		}
		if (channels_value[_left_start] == _channels_max && channels_value[_right_start] == _channels_max ){
			ROS_ERROR_STREAM("Only can enable one side switch at the same time!");
			left_start = false;
			right_start = false;
		}
		else if (channels_value[_left_start] == _channels_max){
			left_start = true;
			right_start = false;
		} else if (channels_value[_right_start] == _channels_max){
			left_start = false;
			right_start = true;
		} else{
			left_start = false;
			right_start = false;
		}			
		
		if (_add_error){
			if (channels_value[ _left_down2up] >= (_channels_min - _error_value) 
			  && channels_value[_left_down2up] <= (_channels_min + _error_value)){
				if (channels_value[ _left_left2right] >= (_channels_max - _error_value)
				 && channels_value[ _left_left2right] <= (_channels_max + _error_value)){
				left_enable = true;
				}else {
					left_enable = false;
				}
			} else {
			left_enable = false;
			}

			if (channels_value[ _right_down2up] <= (_channels_min + _error_value)
			 && channels_value[ _right_down2up] >= (_channels_min - _error_value)){
				if(channels_value[ _right_left2right] <= (_channels_min + _error_value)
				&& channels_value[ _right_left2right] >= (_channels_min - _error_value)){
				right_enable = true;
				}else {
				right_enable = false;
				}
			}else {
			right_enable = false;
			}
		}else{
			if (channels_value[ _left_down2up] = _channels_min ){
				if(channels_value[ _left_left2right] = _channels_max){
				left_enable = true;
				}else {
					left_enable = false;
				}
			} else {
			left_enable = false;
			}

			if (channels_value[ _right_down2up] = _channels_min){
				if(channels_value[ _right_left2right] = _channels_min){
				right_enable = true;
				}else {
				right_enable = false;
				}
			}else {
			right_enable = false;
			}
		}
		

		if (left_start){
			if (left_enable){
				if (channels_value[_left_mode] == _channels_max) {
				vel.linear.x = channels_value[_right_down2up]*k_l+b_l;
				vel.angular.z = 0;
				}
				else if (channels_value[_left_mode] == _channels_mid) {
					vel.linear.x = 0;
					vel.angular.z = channels_value[_right_left2right]*k_a+b_a;	
				}else if (channels_value[_left_mode] == _channels_min) {
					vel.linear.x = channels_value[_right_down2up]*k_l+b_l;
					vel.angular.z = channels_value[_right_left2right]*k_a+b_a;
				}
			}else{
				vel.linear.x = 0;
				vel.angular.z = 0;
			}
		}
		else if (right_start){
			if (right_enable){
				if (channels_value[_right_mode]== _channels_max){
					vel.linear.x = channels_value[_left_down2up]*k_l+b_l;
					vel.angular.z = 0;
				} else if (channels_value[_right_mode]== _channels_mid){
					vel.linear.x = 0;
					vel.angular.z = channels_value[_left_left2right]*k_a+b_a;
				} else if (channels_value[_right_mode]== _channels_min) {
					vel.linear.x = channels_value[_left_down2up]*k_l+b_l;
					vel.angular.z = channels_value[_left_left2right]*k_a+b_a;
				}	
			} else {
				vel.linear.x = 0;
				vel.angular.z = 0;
			}	
		}else {
			vel.linear.x = 0;
			vel.angular.z = 0;
		}	

		vel_pub.publish(vel);	 
	}

}


