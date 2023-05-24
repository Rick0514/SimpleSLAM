#pragma once

#include <types/basic.hpp>
#include <dataproxy/DataProxy.hpp>

#include <utils/Thread.hpp>

using namespace PCLTypes;

namespace ros { class NodeHandle; }

namespace dataproxy
{

class LidarDataProxy : public DataProxy<pc_t>
{
public:
    using Base = DataProxy<pc_t>;
    
private:

    class Ros;
    std::unique_ptr<Ros> mRosImpl;

public:

    LidarDataProxy() = delete;
    LidarDataProxy(ros::NodeHandle& nh, int size);

    void subscribe(const std::shared_ptr<pc_t>& pc);

    ~LidarDataProxy();
};

} // namespace dataproxy


