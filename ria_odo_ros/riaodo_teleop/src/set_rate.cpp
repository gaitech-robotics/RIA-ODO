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

#include "riaodo_teleop/set_rate.h"

namespace ria_teleop
{
	setRate::setRate(const ros::NodeHandle& n, const ros::NodeHandle& p)
    :
    _nh(n),_pnh(p)
	{
		_pnh.param<std::string>("srv_topic", _srv_topic , "/mavros/set_stream_rate");
		_pnh.param<int>("rate_value", _rate_value, 10);
		_pnh.param<int>("steam_id", _stream_id, 0);


		set_rate = _nh.serviceClient<mavros_msgs::StreamRate>(_srv_topic);
	}

    void setRate::callSetrateSrv(){
        stream_rate.request.stream_id = _stream_id;
        stream_rate.request.message_rate = _rate_value;
        stream_rate.request.on_off = true;
        if (set_rate.call(stream_rate)){
            ROS_INFO("Mavros set stream rate successfully!");
        }else {
            ROS_WARN("Mavros set stream rate unsuccessfully! Please call the service again!");
        }
    }
    

}


