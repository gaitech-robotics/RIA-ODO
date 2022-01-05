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

#include <riaodo_wheel_support_motion.h>
#include <cmath>

WheelSupMotion::WheelSupMotion() : nh(), pnh_("~")
{
    pnh_.param<std::string>("imu_topic", imu_topic, "/wit/imu_data");
    pnh_.param<std::string>("parent_frame", parent_frame, "riaodo_base_link");
    pnh_.param<std::string>("child_frame_1", child_frame_1, "left_wheel_support_link");
    pnh_.param<std::string>("child_frame_2", child_frame_2, "right_wheel_support_link");
    pnh_.param("imu_max_angle", imu_max_angle, 0.104719758);
    pnh_.param("wheel_support_max_angle", wheel_support_max_angle, 0.2617993);
    sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, &WheelSupMotion::ImuCallback, this);
}

double WheelSupMotion::Quaternion2EulerYaw(const double x, const double y, const double z, const double w){
    Eigen::Quaterniond q;
    q.x()= x;
    q.y()= y;
    q.z()= z;
    q.w()= w;
    double siny_cosp = 2.0 * (q.w()*q.z()+q.x()*q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y()*q.y()+q.z()*q.z());
    double yaw = atan2(siny_cosp,cosy_cosp);
    //Eigen::Vector3d eular = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return yaw;
}

void WheelSupMotion::tfPublish(const std::string child_frame, const double ry){

    static tf2_ros::TransformBroadcaster tf_br;
    geometry_msgs::TransformStamped tf_transformStamped;
    tf2::Quaternion q;
    
    tf_transformStamped.header.stamp = ros::Time::now();
    tf_transformStamped.header.frame_id = parent_frame;
    tf_transformStamped.child_frame_id = child_frame;
    tf_transformStamped.transform.translation.x = 0.0;
    if (child_frame == "left_wheel_support_link"){
        tf_transformStamped.transform.translation.y = 0.2515;
    }
    if (child_frame == "right_wheel_support_link"){
        tf_transformStamped.transform.translation.y = -0.2515;
    }
    tf_transformStamped.transform.translation.z = 0.133;
    q.setRPY(0, ry, 0);
    tf_transformStamped.transform.rotation.x = q.x();
    tf_transformStamped.transform.rotation.y = q.y();
    tf_transformStamped.transform.rotation.z = q.z();
    tf_transformStamped.transform.rotation.w = q.w();

    tf_br.sendTransform(tf_transformStamped);
    
}

void WheelSupMotion::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    const double x = msg->orientation.x;
    const double y = msg->orientation.y;
    const double z = msg->orientation.z;
    const double w = msg->orientation.w;
    double rz = Quaternion2EulerYaw(x,y,z,w);
    if(fabs(rz) > imu_max_angle){
        if (rz > 0){
            rz = imu_max_angle;
        }else{
            rz = -imu_max_angle;
        }
    }
    double wheelsup_ry = (wheel_support_max_angle/imu_max_angle)*rz; 

    tfPublish(child_frame_1, -wheelsup_ry);
    tfPublish(child_frame_2, wheelsup_ry);
}

int main(int argc,char** argv){
    ros::init(argc, argv, "riaodo_wheel_support_motion");
    WheelSupMotion wheel_support_motion;
    ros::Rate r(50.0);
    while (ros::ok()){
        ros::spin();
        r.sleep();
    }
    return 0;
}