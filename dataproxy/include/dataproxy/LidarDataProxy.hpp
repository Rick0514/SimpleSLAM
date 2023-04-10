#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <types/PCLTypes.hpp>
#include <dataproxy/DataProxy.hpp>

#include <utils/Thread.hpp>

using namespace PCLTypes;

namespace dataproxy
{

enum class VisType : int
{
    None,
    Aligned,
    GlobalMap
};

template <typename PCType, bool UseBag=false>
class LidarDataProxy : public DataProxy<PCType, UseBag>
{
public:
    using KF = KeyFrame<typename PCType::PointType>;
private:
    ros::Subscriber mSub;
    ros::Publisher mPubAligned;

    std::unique_ptr<utils::trd::ResidentThread>  mVisPCThd;

    KF mAlignedKF;
    
    VisType mVisType;
    std::mutex mVisLock;
    std::condition_variable mVisCV;
    
public:

    LidarDataProxy() = delete;
    LidarDataProxy(ros::NodeHandle& nh, int size);

    void subscribe(const sensor_msgs::PointCloud2ConstPtr&);

    void visPCHandler();

    void setVisAligned(const typename PCType::Ptr&, const Pose6d&);

    ~LidarDataProxy(){};
};

} // namespace dataproxy


