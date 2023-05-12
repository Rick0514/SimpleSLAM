#include <pclomp/fast_vgicp.hpp>
#include <PCR/VgicpRegister.hpp>

namespace PCR {

struct VgicpRegister::Vgicp
{
    using vgicp_t = fast_gicp::FastVGICP<pt_t, pt_t>;
    vgicp_t vgicp_;

    Vgicp()
    {
        vgicp_.setResolution(1.0);
        vgicp_.setNumThreads(constant::numCores);
    }
};

VgicpRegister::VgicpRegister() : vgicp_(std::make_shared<Vgicp>()) {}
    

bool VgicpRegister::scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res)
{
    vgicp_->vgicp_.setInputTarget(dst);
    vgicp_->vgicp_.setInputSource(src);

    pc_t aligned;
    vgicp_->vgicp_.align(aligned, res.matrix().cast<float>());
    res.matrix() = vgicp_->vgicp_.getFinalTransformation().cast<scalar_t>();

    return vgicp_->vgicp_.hasConverged();
}

VgicpRegister::~VgicpRegister() {}

}