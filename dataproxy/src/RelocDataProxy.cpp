#include <dataproxy/RelocDataProxy.hpp>

namespace dataproxy {

RelocDataProxy::RelocDataProxy(ros::NodeHandle& nh) : DataProxy<void>(0)
{
    mSub = nh.subscribe("/initialpose", 1, &RelocDataProxy::subscriber, this);
}
    
void RelocDataProxy::registerFunc(const std::function<void(Pose6d&)>& f)
{
    mRelocFunc = f;   
}

void RelocDataProxy::subscriber(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pc)
{
    auto p = pc->pose.pose.position;
    auto q = pc->pose.pose.orientation;
    Pose6d ep;
    ep.setIdentity();
    ep.translate(V3d(p.x, p.y, p.z));
    ep.rotate(Qd(q.w, q.x, q.y, q.z));

    std::stringstream ss;
    ss << "get init pose:\n" << ep.matrix();
    mLg->info("{}", ss.str());
    
    mRelocFunc(ep);
}


}