#include <dataproxy/LidarDataProxy.hpp>

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dataproxy
{

template <typename PCType, bool UseBag>
LidarDataProxy<PCType, UseBag>::LidarDataProxy(ros::NodeHandle& nh, int size) : DataProxy<PCType, UseBag>(size)
{
    this->mLg->info("get in LidarDataProxy");
    mSub = nh.subscribe("/lidar_points", 5, &LidarDataProxy<PCType, UseBag>::subscribe, this);
}

template <typename PCType, bool UseBag>
void LidarDataProxy<PCType, UseBag>::subscribe(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // preprocess
    auto cloud = std::make_shared<PCType>();
    pcl::fromROSMsg(*msg, *cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    this->mDataPtr->template push_back<UseBag>(std::move(cloud));
}

template class LidarDataProxy<PC<Pxyz>>;
template class LidarDataProxy<PC<Pxyzi>>;

template class LidarDataProxy<PC<Pxyz>, true>;
template class LidarDataProxy<PC<Pxyzi>, true>;

} // namespace dataproxy
