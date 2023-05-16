#include <pclomp/fast_vgicp.hpp>
#include <PCR/VgicpRegister.hpp>

namespace PCR {

struct VgicpRegister::Vgicp
{
    using vgicp_t = fast_gicp::FastVGICP<pt_t, pt_t>;
    vgicp_t vgicp_;

    Vgicp()
    {
        vgicp_.setResolution(constant::numCores);
        vgicp_.setNumThreads(1);
    }
};

VgicpRegister::VgicpRegister() : vgicp_(std::make_unique<Vgicp>()) {}
    
void VgicpRegister::initForLC()
{
    vgicp_->vgicp_.setMaxCorrespondenceDistance(150);
    vgicp_->vgicp_.setMaximumIterations(100);
    vgicp_->vgicp_.setTransformationEpsilon(1e-6);
    vgicp_->vgicp_.setEuclideanFitnessEpsilon(1e-6);
    vgicp_->vgicp_.setRANSACIterations(0);
}

bool VgicpRegister::scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res)
{
    vgicp_->vgicp_.setInputTarget(dst);
    vgicp_->vgicp_.setInputSource(src);

    pc_t aligned;
    vgicp_->vgicp_.align(aligned, res.matrix().cast<float>());
    res.matrix() = vgicp_->vgicp_.getFinalTransformation().cast<scalar_t>();

    return vgicp_->vgicp_.hasConverged();
}

scalar_t VgicpRegister::getFitnessScore()
{
    return vgicp_->vgicp_.getFitnessScore();
}

VgicpRegister::~VgicpRegister() = default;

}