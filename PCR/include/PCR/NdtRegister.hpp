#pragma once
#include <pclomp/ndt_omp.h>
#include <PCR/PointCloudRegister.hpp>

namespace PCR
{

class NdtRegister : public PointCloudRegister
{
private:
    
    static constexpr float resol{1.0}; 
    typename pclomp::NormalDistributionsTransform<pt_t, pt_t>::Ptr _ndt_omp;

public:

    using typename PointCloudRegister::PC_Ptr;
    using typename PointCloudRegister::PC_cPtr;

    NdtRegister();

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) override;

};
    
} // namespace PCR


