#pragma once
#include <PCR/PointCloudRegister.hpp>

namespace PCR {

class VgicpRegister : public PointCloudRegister
{
protected:

    struct Vgicp;
    std::unique_ptr<Vgicp> vgicp_;

public:

    using typename PointCloudRegister::PC_Ptr;
    using typename PointCloudRegister::PC_cPtr;

    VgicpRegister();

    void initForLC();

    virtual scalar_t getFitnessScore() override;
    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) override;

    ~VgicpRegister();
};

}
