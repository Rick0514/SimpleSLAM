#include <PCR/NdtRegister.hpp>

namespace PCR
{

template<typename PointType>
NdtRegister<PointType>::NdtRegister()
{
    _ndt_omp.reset(new pclomp::NormalDistributionsTransform<PointType, PointType>());
    _ndt_omp->setResolution(resol);
    _ndt_omp->setNumThreads(1);
    _ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
}

template<typename PointType>
bool NdtRegister<PointType>::scan2Map(PC_cPtr& src, PC_cPtr& dst, Pose6d& res)
{
    _ndt_omp->setInputTarget(dst);
    _ndt_omp->setInputSource(src);

    typename pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());

    _ndt_omp->align(*aligned, res.matrix().cast<float>());
    Eigen::Matrix4f mf = _ndt_omp->getFinalTransformation();
    res.matrix() = mf.cast<double>();

    spdlog::debug("align fitness: {}", _ndt_omp->getFitnessScore());

    return _ndt_omp->hasConverged();
}
    
PCRTemplateInstantiateExplicitly(NdtRegister)

} // namespace PCR
