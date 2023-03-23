#include <dataproxy/EkfOdomProxy.hpp>

namespace dataproxy
{

template <bool UseBag>
EkfOdomProxy<UseBag>::EkfOdomProxy(ros::NodeHandle& nh, int size)
: DataProxy<Odometry, UseBag>(size)
{
    auto ekf_sub = nh.subscribe("/ekf_odom", 50, &EkfOdomProxy::subscribe, this);
}

template <bool UseBag>
void EkfOdomProxy<UseBag>::subscribe(const nav_msgs::OdometryConstPtr& msg)
{
    Odometry odom;
    odom.stamp = msg->header.stamp.toSec();
    
    auto p = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;

    Pose6d pose;
    pose.setIdentity();
    pose.pretranslate(V3d(p.x, p.y, p.z));
    pose.rotate(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    odom.odom = pose;
    this->mDataPtr->push_back(odom);
}

template class EkfOdomProxy<true>;
template class EkfOdomProxy<false>;

}
