#include <dataproxy/LidarDataProxy.hpp>

#include <pcl/pcl_config.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcp/pcp.hpp>
#include <utils/Shared_ptr.hpp>

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
    auto cloud = pcl::make_shared<PCType>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
    pcp::removeNaNFromPointCloud(*cloud);
    pcp::voxelDownSample<typename PCType::PointType>(cloud, 0.5f);
#if PCL_VERSION_COMPARE(<=, 1, 10, 0)
    auto stdcloud = utils::make_shared_ptr(cloud);
    this->mDataPtr->template push_back<UseBag>(std::move(stdcloud));
#else
    this->mDataPtr->template push_back<UseBag>(std::move(cloud));
#endif

}

template class LidarDataProxy<PC<Pxyz>>;
template class LidarDataProxy<PC<Pxyzi>>;

template class LidarDataProxy<PC<Pxyz>, true>;
template class LidarDataProxy<PC<Pxyzi>, true>;

} // namespace dataproxy
