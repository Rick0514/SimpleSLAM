#pragma once

#include <types/basic.hpp>
#include <utils/Logger.hpp>

#include <memory>

namespace ros { class NodeHandle; }

namespace dataproxy {

class Vis final
{

private:
    class Ros;
    std::unique_ptr<Ros> mRosImpl;
    
    std::shared_ptr<utils::logger::Logger> lg;

public:

    Vis(ros::NodeHandle& nh);

    void registerPCPub(const std::string& name);

    void publishPC(const std::string& name, const pc_t& pc);
    void publishPC(const std::string& name, const pc_t& pc, const pose_t& pose);

    void publishOdom(const EigenTypes::Pose6d& p, double t);

    ~Vis();

};

}

