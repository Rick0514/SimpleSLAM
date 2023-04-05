#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <dataproxy/DataProxy.hpp>

namespace dataproxy
{

template<bool UseBag=false>
class EkfOdomProxy : public DataProxy<Odometry, UseBag>
{
public:
    explicit EkfOdomProxy(ros::NodeHandle& nh, int size);

    void subscribe(const nav_msgs::OdometryConstPtr& msg);

    ~EkfOdomProxy() {};
};
    
} // namespace dataproxy




