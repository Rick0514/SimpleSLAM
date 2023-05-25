#include <dataproxy/Vis.hpp>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <utils/Thread.hpp>
#include <pcp/pcp.hpp>

namespace dataproxy {

class Vis::Ros
{

private:

    std::unique_ptr<utils::trd::ResidentThread> _pc_thd;
    std::string _pc_name;
    std::mutex _pc_lock;
    std::condition_variable _pc_cv;

public:

    ros::NodeHandle& _nh;
    std::unordered_map<std::string, ros::Publisher> _pubmap;
    std::unordered_map<std::string, pc_t> _pcmap;

    std::shared_ptr<utils::logger::Logger> _lg;

    Ros(ros::NodeHandle& nh);

    void visPCHandler();
    void notifyPC(const std::string& name, const pc_t& pc);

    ~Ros();

};

Vis::Ros::Ros(ros::NodeHandle& nh) : _nh(nh)
{
    _lg = utils::logger::Logger::getInstance();
    _pc_thd = std::make_unique<utils::trd::ResidentThread>(&Ros::visPCHandler, this);
}

void Vis::Ros::visPCHandler()
{
    std::unique_lock<std::mutex> lk(_pc_lock);
    _pc_cv.wait(lk, [&](){ return !_pc_name.empty() || _lg->isProgramExit(); });

    // trans to ros pc
    if(_pcmap.count(_pc_name)){
        sensor_msgs::PointCloud2 rospc;
        pcl::toROSMsg(_pcmap[_pc_name], rospc);
        rospc.header.frame_id = "map";
        _pubmap[_pc_name].publish(rospc);
        _pc_name.clear();
    }
}

void Vis::Ros::notifyPC(const std::string &name, const pc_t &pc)
{
    std::unique_lock<std::mutex> lk(_pc_lock, std::try_to_lock);
    
    if(lk.owns_lock()){
        _pcmap[name] = pc;
        _pc_name = name;
        _pc_cv.notify_one();
    }
}

Vis::Ros::~Ros(){
    _pc_thd->Stop();
    std::lock_guard<std::mutex> lk(_pc_lock);
    _pc_name = "exit";
    _pc_cv.notify_one();
}

Vis::Vis(ros::NodeHandle& nh) : mRosImpl(std::make_unique<Ros>(nh))
{
    lg = utils::logger::Logger::getInstance();
}

void Vis::registerPCPub(const std::string &name)
{
    if(mRosImpl->_pubmap.count(name) == 0){
        mRosImpl->_pubmap[name] = mRosImpl->_nh.advertise<sensor_msgs::PointCloud2>(name, 1, true);
    }
}

void Vis::publishPC(const std::string& name, const pc_t& pc)
{
    mRosImpl->notifyPC(name, pc);
}

void Vis::publishPC(const std::string& name, const pc_t& pc, const pose_t& pose)
{
    pc_t align;
    pcp::transformPointCloud<pt_t>(pc, align, pose.cast<float>());
    publishPC(name, align);
}

Vis::~Vis()
{
    lg->info("vis exit!");
}

}