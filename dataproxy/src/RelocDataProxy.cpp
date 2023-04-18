#include <dataproxy/RelocDataProxy.hpp>

namespace dataproxy {

RelocDataProxy::RelocDataProxy(ros::NodeHandle& nh) : DataProxy<void>(0)
{
    mSub = nh.subscribe("/initialpose", 1, &RelocDataProxy::subscriber, this);
}
    
void RelocDataProxy::registerFunc(const RelocFuncType& f)
{
    mRelocFunc = f;   
}

void RelocDataProxy::subscriber(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pc)
{
    auto p = pc->pose.pose.position;
    auto q = pc->pose.pose.orientation;
    pose_t ep;
    ep.setIdentity();
    ep.translate(V3<scalar_t>(p.x, p.y, p.z));
    ep.rotate(Qt<scalar_t>(q.w, q.x, q.y, q.z));

    std::stringstream ss;
    ss << "get init pose:\n" << ep.matrix();
    mLg->info("{}", ss.str());
    
    mRelocFunc(ep);
}


}