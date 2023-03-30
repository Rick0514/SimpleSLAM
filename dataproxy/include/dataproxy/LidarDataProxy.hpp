#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <types/PCLTypes.hpp>
#include <dataproxy/DataProxy.hpp>

using namespace PCLTypes;

namespace dataproxy
{

template <typename T>
class LidarDataProxy : public DataProxy<T>
{
public:

    explicit LidarDataProxy(ros::NodeHandle& nh, int size);

    void subscribe(const sensor_msgs::PointCloud2ConstPtr&);

    ~LidarDataProxy(){};
};

} // namespace dataproxy

