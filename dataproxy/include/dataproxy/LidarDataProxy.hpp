#pragma once

#include <types/basic.hpp>
#include <dataproxy/DataProxy.hpp>

#include <utils/Thread.hpp>

using namespace PCLTypes;

namespace ros { class NodeHandle; }

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

    class Ros;
    std::unique_ptr<Ros> mRosImpl;

    std::unique_ptr<utils::trd::ResidentThread> mVisPCThd;

    pc_t::Ptr mAlignedScan;
    
    VisType mVisType;
    std::mutex mVisLock;
    std::condition_variable mVisCV;
    
public:

    LidarDataProxy() = delete;
    LidarDataProxy(ros::NodeHandle& nh, int size);

    void subscribe(const std::shared_ptr<pc_t>& pc);

    void visPCHandler();

    void setVisAligned(const pc_t::Ptr& pc, const pose_t& pose);
    void setVisGlobalMap(const pc_t::ConstPtr&);
    
    ~LidarDataProxy();
};

} // namespace dataproxy


