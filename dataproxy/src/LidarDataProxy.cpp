#include <dataproxy/LidarDataProxy.hpp>

#include <pcl/pcl_config.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcp/pcp.hpp>
#include <utils/Shared_ptr.hpp>

namespace dataproxy
{

LidarDataProxy::LidarDataProxy(ros::NodeHandle& nh, int size) :
    DataProxy<pc_t>(size),
    mVisType(VisType::None)
{
    this->mLg->info("get in LidarDataProxy");
    mSub = nh.subscribe("/lidar_points", 5, &LidarDataProxy::subscribe, this);

    mPubAligned = nh.advertise<sensor_msgs::PointCloud2>("/aligned", 1);
    mVisPCThd = std::make_unique<utils::trd::ResidentThread>(&LidarDataProxy::visPCHandler, this);
}

void LidarDataProxy::subscribe(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    auto cloud = pcl::make_shared<pc_t>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
    pcp::removeNaNFromPointCloud(*cloud);
    pcp::voxelDownSample<pt_t>(cloud, 0.7f);
#if PCL_VERSION_COMPARE(<=, 1, 10, 0)
    auto stdcloud = utils::make_shared_ptr(cloud);
    this->mDataPtr->template push_back<constant::usebag>(std::move(stdcloud));
#else
    this->mDataPtr->template push_back<constant::usebag>(std::move(cloud));
#endif

}

void LidarDataProxy::visPCHandler()
{
    std::unique_lock<std::mutex> lk(mVisLock);
    mVisCV.wait(lk, [=](){ return mVisType != VisType::None; });
    
    switch(mVisType){
        case VisType::Aligned: {
            // trans
            pc_t::Ptr aligned(new pc_t());
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

    if(mVisType != VisType::Exit)   mVisType = VisType::None;
}

void LidarDataProxy::setVisAligned(const pc_t::Ptr& pc, const pose_t& p)
{
    std::lock_guard<std::mutex> lk(mVisLock);
    mVisType = VisType::Aligned;
    mAlignedKF.pose = p;
    mAlignedKF.pc = pc;
    mVisCV.notify_all();
}

LidarDataProxy::~LidarDataProxy()
{
    mLg->info("prepare exit lidar proxy!!");
    {
        std::lock_guard<std::mutex> lk(mVisLock);
        mVisType = VisType::Exit;
        mVisCV.notify_all();
    }
    mLg->info("exit lidar proxy!!");
}

} // namespace dataproxy
