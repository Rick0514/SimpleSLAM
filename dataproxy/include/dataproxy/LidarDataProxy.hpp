#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <dataproxy/DataProxy.hpp>

namespace dataproxy
{

template <typename T>
class LidarDataProxy : public DataProxy<T>
{
private:
    std::shared_ptr<ros::NodeHandle> mNh;

public:

    explicit LidarDataProxy(std::shared_ptr<ros::NodeHandle>& nh, int size);

    void subscribe(const sensor_msgs::PointCloud2ConstPtr&);

    ~LidarDataProxy();
};

} // namespace dataproxy


