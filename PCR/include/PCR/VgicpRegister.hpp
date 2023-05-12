#pragma once
#include <PCR/PointCloudRegister.hpp>

namespace PCR {

class VgicpRegister : public PCR::PointCloudRegister
{
protected:

    struct Vgicp;
    std::shared_ptr<Vgicp> vgicp_;

public:

    using typename PointCloudRegister::PC_Ptr;
    using typename PointCloudRegister::PC_cPtr;

    VgicpRegister();

    std::shared_ptr<Vgicp> getPtr() { return vgicp_; }

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) override;

    ~VgicpRegister();
};

}
