#pragma once
#include <pclomp/fast_vgicp.hpp>
#include <PCR/PointCloudRegister.hpp>

namespace PCR {

class VgicpRegister : public PCR::PointCloudRegister
{
public:
    using vgicp_t = fast_gicp::FastVGICP<pt_t, pt_t>::Ptr;

protected:
    vgicp_t vgicp_;

public:

    using typename PointCloudRegister::PC_Ptr;
    using typename PointCloudRegister::PC_cPtr;

    VgicpRegister();

    vgicp_t getPtr() { return vgicp_; }

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) override;

};

}
