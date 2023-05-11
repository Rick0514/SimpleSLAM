#include <PCR/VgicpRegister.hpp>

namespace PCR {

VgicpRegister::VgicpRegister(){
    vgicp_ = pcl::make_shared<vgicp_t>();
    vgicp_->setResolution(1.0);
    vgicp_->setNumThreads(constant::numCores);
}

bool VgicpRegister::scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res)
{
    vgicp_->setInputTarget(dst);
    vgicp_->setInputSource(src);

    pc_t aligned;
    vgicp_->align(aligned, res.matrix().cast<float>());
    res.matrix() = vgicp_->getFinalTransformation().cast<scalar_t>();

    return vgicp_->hasConverged();
}

}