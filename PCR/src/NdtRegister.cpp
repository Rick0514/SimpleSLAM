#include <PCR/NdtRegister.hpp>


namespace PCR
{

template<typename PointType>
NdtRegister<PointType>::NdtRegister()
{
    PointCloudRegister<PointType>();
    _ndt_omp.reset(new pclomp::NormalDistributionsTransform<PointType, PointType>());
    _ndt_omp->setResolution(resol);
    _ndt_omp->setNumThreads(4);
    _ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
}

template<typename PointType>
bool NdtRegister<PointType>::scan2Map(PC_Ptr src, PC_Ptr dst, Pose6d& res)
{
    _ndt_omp->setInputTarget(dst);
    _ndt_omp->setInputSource(src);

    typename pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());

    _ndt_omp->align(*aligned, res.matrix());

    return _ndt_omp->hasConverged();
}
    
} // namespace PCR
