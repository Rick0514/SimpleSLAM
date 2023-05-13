#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <dataproxy/RelocDataProxy.hpp>

namespace dataproxy {

class RelocDataProxy::Ros
{
public:
    ros::Subscriber mSub;
    RelocDataProxy* rdp;

    Ros(ros::NodeHandle& nh);
    void subscriber(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);

};

RelocDataProxy::Ros::Ros(ros::NodeHandle& nh)
{
    mSub = nh.subscribe("/initialpose", 1, &Ros::subscriber, this);
}

RelocDataProxy::RelocDataProxy(ros::NodeHandle& nh) : DataProxy<void>(0), mRosImpl(std::make_unique<Ros>(nh))
{
    mRosImpl->rdp = this;
}
    
void RelocDataProxy::registerFunc(const RelocFuncType& f)
{
    mRelocFunc = f;   
}

void RelocDataProxy::Ros::subscriber(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pc)
{
    auto p = pc->pose.pose.position;
    auto q = pc->pose.pose.orientation;
    pose_t ep;
    ep.setIdentity();
    ep.translate(V3<scalar_t>(p.x, p.y, p.z));
    ep.rotate(Qt<scalar_t>(q.w, q.x, q.y, q.z));

    std::stringstream ss;
    ss << "get init pose:\n" << ep.matrix();
    rdp->mLg->info("{}", ss.str());
    
    rdp->mRelocFunc(ep);
}

RelocDataProxy::~RelocDataProxy() = default;

}