#include <PCR/NdtRegister.hpp>

namespace PCR
{

NdtRegister::NdtRegister()
{
    _ndt_omp.reset(new pclomp::NormalDistributionsTransform<pt_t, pt_t>());
    _ndt_omp->setResolution(resol);
    _ndt_omp->setNumThreads(numCores);
    _ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
}

bool NdtRegister::scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res)
{
    _ndt_omp->setInputTarget(dst);
    _ndt_omp->setInputSource(src);

    PC_Ptr aligned(new pc_t());

    _ndt_omp->align(*aligned, res.matrix().cast<float>());
    res.matrix() = _ndt_omp->getFinalTransformation().cast<scalar_t>();

    return _ndt_omp->hasConverged();
}

} // namespace PCR
