#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <types/basic.hpp>
#include <dataproxy/DataProxy.hpp>

#include <utils/Thread.hpp>

using namespace PCLTypes;

namespace dataproxy
{

enum class VisType : int
{
    None,
    Aligned,
    GlobalMap,
    Exit
};

class LidarDataProxy : public DataProxy<pc_t>
{
public:
    using Base = DataProxy<pc_t>;
    using KF = KeyFrame;
private:
    ros::Subscriber mSub;
    ros::Publisher mPubAligned;

    std::unique_ptr<utils::trd::ResidentThread> mVisPCThd;

    KF mAlignedKF;
    
    VisType mVisType;
    std::mutex mVisLock;
    std::condition_variable mVisCV;
    
public:

    LidarDataProxy() = delete;
    LidarDataProxy(ros::NodeHandle& nh, int size);

    void subscribe(const sensor_msgs::PointCloud2ConstPtr&);

    void visPCHandler();

    void setVisAligned(const pc_t::Ptr&, const pose_t&);

    ~LidarDataProxy();
};

} // namespace dataproxy


