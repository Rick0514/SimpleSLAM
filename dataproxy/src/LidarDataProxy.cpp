#include <dataproxy/LidarDataProxy.hpp>

#include <pcl/pcl_config.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcp/pcp.hpp>
#include <utils/Shared_ptr.hpp>

namespace dataproxy
{

template <typename PCType, bool UseBag>
LidarDataProxy<PCType, UseBag>::LidarDataProxy(ros::NodeHandle& nh, int size) :
    DataProxy<PCType, UseBag>(size),
    mVisType(VisType::None)
{
    this->mLg->info("get in LidarDataProxy");
    mSub = nh.subscribe("/lidar_points", 5, &LidarDataProxy<PCType, UseBag>::subscribe, this);

    mPubAligned = nh.advertise<sensor_msgs::PointCloud2>("/aligned", 1);
    mVisPCThd = std::make_unique<utils::trd::ResidentThread>(&LidarDataProxy<PCType, UseBag>::visPCHandler, this);
}

template <typename PCType, bool UseBag>
void LidarDataProxy<PCType, UseBag>::subscribe(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    auto cloud = pcl::make_shared<PCType>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
    pcp::removeNaNFromPointCloud(*cloud);
    pcp::voxelDownSample<typename PCType::PointType>(cloud, 0.7f);
#if PCL_VERSION_COMPARE(<=, 1, 10, 0)
    auto stdcloud = utils::make_shared_ptr(cloud);
    this->mDataPtr->template push_back<UseBag>(std::move(stdcloud));
#else
    this->mDataPtr->template push_back<UseBag>(std::move(cloud));
#endif

}

template <typename PCType, bool UseBag>
void LidarDataProxy<PCType, UseBag>::visPCHandler()
{
    std::unique_lock<std::mutex> lk(mVisLock);
    mVisCV.wait(lk, [=](){ return mVisType != VisType::None; });
    
    switch(mVisType){
        case VisType::Aligned: {
            // trans
            typename PCType::Ptr aligned(pcl::make_shared<PCType>());
            pcl::transformPointCloud(*mAlignedKF.pc, *aligned, mAlignedKF.pose.matrix());
            
            sensor_msgs::PointCloud2 rospc;
            pcl::toROSMsg(*aligned, rospc);
            rospc.header.frame_id = "map";
            mPubAligned.publish(rospc);
            break;
        }
        case VisType::GlobalMap: {
            break;
        }
        default: {}
    }
    
    mVisType = VisType::None;
}

template <typename PCType, bool UseBag>
void LidarDataProxy<PCType, UseBag>::setVisAligned(const typename PCType::Ptr& pc, const Pose6d& p)
{
    std::lock_guard<std::mutex> lk(mVisLock);
    mVisType = VisType::Aligned;
    mAlignedKF.pose = p;
    mAlignedKF.pc = pc;
    mVisCV.notify_all();
}

template class LidarDataProxy<PC<Pxyz>>;
template class LidarDataProxy<PC<Pxyzi>>;

template class LidarDataProxy<PC<Pxyz>, true>;
template class LidarDataProxy<PC<Pxyzi>, true>;

} // namespace dataproxy
