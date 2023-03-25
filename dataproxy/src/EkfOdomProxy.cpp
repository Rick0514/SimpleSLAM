#include <dataproxy/EkfOdomProxy.hpp>

namespace dataproxy
{

EkfOdomProxy::EkfOdomProxy(ros::NodeHandle& nh, int size)
: DataProxy<Odometry>(size)
{
    auto ekf_sub = nh.subscribe("/ekf_odom", 50, &EkfOdomProxy::subscribe, this);
}

void EkfOdomProxy::subscribe(const nav_msgs::OdometryConstPtr& msg)
{
    auto odom = std::make_shared<Odometry>();
    odom->stamp = msg->header.stamp.toSec();
    
    auto p = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;

    Pose6d pose;
    pose.setIdentity();
    pose.pretranslate(V3d(p.x, p.y, p.z));
    pose.rotate(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    odom->odom = pose;
    this->mDataPtr->template push_back<UseBag>(std::move(odom));
}

}
