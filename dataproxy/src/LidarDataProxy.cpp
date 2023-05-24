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

    // because ldp contains ros, so ldp's life must be longer than object ros
    LidarDataProxy* ldp;

    Ros(ros::NodeHandle& nh);
    void subscribe(const sensor_msgs::PointCloud2ConstPtr&);
};

LidarDataProxy::Ros::Ros(ros::NodeHandle& nh)
{
    auto cfg = config::Params::getInstance();
    std::string lidar_topic = cfg["dataproxy"]["lidar"];

    mSub = nh.subscribe(lidar_topic, 5, &Ros::subscribe, this);
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
    mRosImpl(std::make_unique<Ros>(nh))
{
    mRosImpl->ldp = this;
}

LidarDataProxy::~LidarDataProxy()
{
    mLg->info("exit lidar proxy!!");
}

} // namespace dataproxy
