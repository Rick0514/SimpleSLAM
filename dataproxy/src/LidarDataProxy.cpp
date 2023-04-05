#include <dataproxy/LidarDataProxy.hpp>

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dataproxy
{

template <typename T, bool UseBag>
LidarDataProxy<T, UseBag>::LidarDataProxy(ros::NodeHandle& nh, int size) : DataProxy<T, UseBag>(size)
{
    auto pc_sub = nh.subscribe("/lidar_points", 5, &LidarDataProxy<T, UseBag>::subscribe, this);
}

template <typename T, bool UseBag>
void LidarDataProxy<T, UseBag>::subscribe(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // preprocess
    auto cloud = std::make_shared<T>();
    pcl::fromROSMsg(*msg, *cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    this->mDataPtr->template push_back<UseBag>(std::move(cloud));
}

template class LidarDataProxy<PCxyz>;
template class LidarDataProxy<PCxyzi>;

template class LidarDataProxy<PCxyz, true>;
template class LidarDataProxy<PCxyzi, true>;

} // namespace dataproxy
