#include <dataproxy/LidarDataProxy.hpp>

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dataproxy
{

template <typename T>
LidarDataProxy<T>::LidarDataProxy(std::shared_ptr<ros::NodeHandle>& nh, int size) : DataProxy<T>(size), mNh(nh)
{
    mNh->subscribe("/lidar_points", 5, &LidarDataProxy<T>::subscribe, this);
}

template <typename T>
void LidarDataProxy<T>::subscribe(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // preprocess
    T cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, indices);
    mDataPtr->push_back(cloud);
}

template <typename T>
LidarDataProxy<T>::LidarDataProxy::~LidarDataProxy(){}


} // namespace dataproxy
