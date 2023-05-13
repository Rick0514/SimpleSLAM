#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <dataproxy/LidarDataProxy.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcp/pcp.hpp>
#include <utils/Shared_ptr.hpp>

#include <config/params.hpp>

namespace dataproxy
{

class LidarDataProxy::Ros
{
public:
    ros::Subscriber mSub;
    ros::Publisher mPubAligned;
    ros::Publisher mPubGlobal;

    sensor_msgs::PointCloud2 mGlobalMap;

    // because ldp contains ros, so ldp's life must be longer than object ros
    LidarDataProxy* ldp;

    Ros(ros::NodeHandle& nh);
    void subscribe(const sensor_msgs::PointCloud2ConstPtr&);
};

LidarDataProxy::Ros::Ros(ros::NodeHandle& nh)
{
    auto cfg = config::Params::getInstance();
    std::string lidar_topic = cfg["dataproxy"]["lidar"];
    std::string vis_align_topic = cfg["vis"]["align"];
    std::string vis_submap_topic = cfg["vis"]["submap"];

    mSub = nh.subscribe(lidar_topic, 5, &Ros::subscribe, this);

    mPubAligned = nh.advertise<sensor_msgs::PointCloud2>(vis_align_topic, 1);
    mPubGlobal = nh.advertise<sensor_msgs::PointCloud2>(vis_submap_topic, 1);
    // wait a bit, make all pub connect with sub!!
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void LidarDataProxy::Ros::subscribe(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    auto cloud = std::make_shared<pc_t>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
    ldp->subscribe(cloud);
}

void LidarDataProxy::subscribe(const std::shared_ptr<pc_t>& pc)
{
    pcp::removeNaNFromPointCloud(*pc);
    mDataPtr->template push_back<constant::usebag>(std::move(pc));
}

LidarDataProxy::LidarDataProxy(ros::NodeHandle& nh, int size) :
    DataProxy<pc_t>(size),
    mVisType(VisType::None),
    mAlignedScan(pcl::make_shared<pc_t>()),
    mRosImpl(std::make_unique<Ros>(nh))
{
    mRosImpl->ldp = this;
    mVisPCThd = std::make_unique<utils::trd::ResidentThread>(&LidarDataProxy::visPCHandler, this);
}

void LidarDataProxy::visPCHandler()
{
    std::unique_lock<std::mutex> lk(mVisLock);
    mVisCV.wait(lk, [=](){ return mVisType != VisType::None; });
    
    switch(mVisType){
        case VisType::Aligned: {
            // trans
            if(mRosImpl->mPubAligned.getNumSubscribers() > 0)
            {
                sensor_msgs::PointCloud2 rospc;
                pcl::toROSMsg(*mAlignedScan, rospc);
                rospc.header.frame_id = "map";
                mRosImpl->mPubAligned.publish(rospc);
            }
            break;
        }
        case VisType::GlobalMap: {
            // mLg->info("vis is notified, show submap!!");
            if(mRosImpl->mPubGlobal.getNumSubscribers() > 0){
                // mLg->info("get rviz sub, show submap!!");
                mRosImpl->mGlobalMap.header.frame_id = "map";
                mRosImpl->mPubGlobal.publish(mRosImpl->mGlobalMap);           
            }
            break;
        }
        default: {}
    }

    if(mVisType != VisType::Exit)   mVisType = VisType::None;
}

void LidarDataProxy::setVisAligned(const pc_t::Ptr& pc, const pose_t& pose)
{
    std::lock_guard<std::mutex> lk(mVisLock);
    mVisType = VisType::Aligned;
    pcp::transformPointCloud<pt_t>(pc, *mAlignedScan, pose.cast<float>());
    // pcl::transformPointCloud(*pc, *mAlignedScan, pose.cast<float>());
    mVisCV.notify_one();
}

void LidarDataProxy::setVisGlobalMap(const pc_t::ConstPtr& g)
{
    std::lock_guard<std::mutex> lk(mVisLock);
    mVisType = VisType::GlobalMap;
    pcl::toROSMsg(*g, mRosImpl->mGlobalMap);
    mVisCV.notify_one();    
    // mLg->info("info vis to show submap!!");
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
