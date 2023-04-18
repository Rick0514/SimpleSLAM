#pragma once

#include <ros/ros.h>
#include <types/basic.hpp>
#include <nav_msgs/Odometry.h>

#include <dataproxy/DataProxy.hpp>

namespace dataproxy
{

class EkfOdomProxy : public DataProxy<Odometry>
{
public:
    explicit EkfOdomProxy(ros::NodeHandle& nh, int size);

    void subscribe(const nav_msgs::OdometryConstPtr& msg);

    ~EkfOdomProxy() {};
};
    
} // namespace dataproxy




