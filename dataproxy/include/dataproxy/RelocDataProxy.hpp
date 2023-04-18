#pragma once
#include <functional>

#include <ros/ros.h>
#include <types/basic.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <dataproxy/DataProxy.hpp>

namespace dataproxy {

using namespace EigenTypes;

class RelocDataProxy : public DataProxy<void>
{

private:

    using RelocFuncType = std::function<void(pose_t&)>;

    ros::Subscriber mSub;
    RelocFuncType mRelocFunc;

public:

    RelocDataProxy() = delete;
    RelocDataProxy(ros::NodeHandle& nh);
    
    void registerFunc(const RelocFuncType&);
    void subscriber(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);

};

}