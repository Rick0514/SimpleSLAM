#pragma once
#include <functional>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <dataproxy/DataProxy.hpp>
#include <types/EigenTypes.hpp>

namespace dataproxy {

using namespace EigenTypes;

class RelocDataProxy : public DataProxy<void>
{

private:
    ros::Subscriber mSub;
    std::function<void(Pose6d&)> mRelocFunc;

public:

    RelocDataProxy() = delete;
    RelocDataProxy(ros::NodeHandle& nh);
    
    void registerFunc(const std::function<void(Pose6d&)>&);
    void subscriber(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);

};

}