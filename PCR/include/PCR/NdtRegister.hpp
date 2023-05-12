#pragma once
#include <PCR/PointCloudRegister.hpp>

namespace PCR
{

class NdtRegister : public PointCloudRegister
{
private:
    
    static constexpr float resol{1.0};

    struct Ndt;
    std::unique_ptr<Ndt> _ndt;

public:

    using typename PointCloudRegister::PC_Ptr;
    using typename PointCloudRegister::PC_cPtr;

    NdtRegister();

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) override;

    ~NdtRegister();
};
    
} // namespace PCR


