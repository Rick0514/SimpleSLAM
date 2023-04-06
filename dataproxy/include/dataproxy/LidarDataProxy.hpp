#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <types/PCLTypes.hpp>
#include <dataproxy/DataProxy.hpp>

using namespace PCLTypes;

namespace dataproxy
{

template <typename PCType, bool UseBag=false>
class LidarDataProxy : public DataProxy<PCType, UseBag>
{
private:
    ros::Subscriber mSub;

public:

    explicit LidarDataProxy(ros::NodeHandle& nh, int size);

    void subscribe(const sensor_msgs::PointCloud2ConstPtr&);

    ~LidarDataProxy(){};
};

} // namespace dataproxy


