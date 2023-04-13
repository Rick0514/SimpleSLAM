#pragma once
#include <pclomp/ndt_omp.h>
#include <PCR/PointCloudRegister.hpp>

namespace PCR
{

template<typename PointType>
class NdtRegister : public PointCloudRegister<PointType>
{
private:
    
    static constexpr float resol{1.0}; 
    typename pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr _ndt_omp;

public:

    using typename PointCloudRegister<PointType>::PC_Ptr;
    using typename PointCloudRegister<PointType>::PC_cPtr;

    NdtRegister();

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, Pose6d& res) override;

};
    
} // namespace PCR


