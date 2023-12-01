#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <types/basic.hpp>
#include <filter/Eskf.hpp>

namespace dataproxy {

class EskfProxy
{
public:
    EskfProxy();

    void onImu(const sensor_msgs::ImuConstPtr& msg);
    void onWheel(const nav_msgs::OdometryConstPtr& msg);

    void pubEskfState(const ros::TimerEvent& e);

    filter::State* getEskf() { return eskf; }

protected:
    
    ros::NodeHandle nh;
    ros::Subscriber mImuSub;
    ros::Subscriber mWheelSub;

    ros::Publisher vis_R;
    ros::Publisher vis_p;
    ros::Publisher vis_v;
    ros::Publisher vis_w;

    ros::Publisher vis_RW;
    ros::Publisher vis_hr;
    
    ros::Publisher vis_odom;

    ros::Timer timer;

    filter::State* eskf;

    bool is_init;

    std::shared_ptr<utils::logger::Logger> lg;

};

}
