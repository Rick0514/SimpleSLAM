#pragma once
#include <functional>

#include <types/basic.hpp>
#include <dataproxy/DataProxy.hpp>

namespace ros { class NodeHandle; }

namespace dataproxy {

using namespace EigenTypes;

class RelocDataProxy : public DataProxy<void>
{
private:

    using RelocFuncType = std::function<void(pose_t&)>;

    class Ros;
    std::unique_ptr<Ros> mRosImpl;

    RelocFuncType mRelocFunc;

public:

    RelocDataProxy() = delete;
    ~RelocDataProxy();
    
    RelocDataProxy(ros::NodeHandle& nh);
    
    void registerFunc(const RelocFuncType&);
};

}