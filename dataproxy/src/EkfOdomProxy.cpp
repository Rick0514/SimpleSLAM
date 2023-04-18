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

    pose_t pose;
    pose.setIdentity();
    pose.pretranslate(V3<scalar_t>(p.x, p.y, p.z));
    pose.rotate(Qt<scalar_t>(q.w, q.x, q.y, q.z));
    odom->odom = pose;
    this->mDataPtr->template push_back<constant::usebag>(std::move(odom));
}

}
