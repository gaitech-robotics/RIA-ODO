#ifndef __RIAODO_WHEEL_SUPPORT_MOTION_H
#define __RIAODO_WHEEL_SUPPORT_MOTION_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class WheelSupMotion
{
    public:
        WheelSupMotion();
    private:
        void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        double Quaternion2EulerYaw(const double x,const double y,const double z,const double w);
        void tfPublish(const std::string child_frame, const double ry);
        std::string imu_topic;
        std::string parent_frame;
        std::string child_frame_1;
        std::string child_frame_2;
        double imu_max_angle;
        double wheel_support_max_angle;
        ros::NodeHandle nh;
        ros::NodeHandle pnh_;    
        ros::Subscriber sub;
};

#endif
