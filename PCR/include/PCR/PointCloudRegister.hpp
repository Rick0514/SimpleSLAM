#pragma once

#include <types/basic.hpp>
#include <utils/Logger.hpp>

namespace PCR
{
using namespace EigenTypes;
using namespace PCLTypes;

class PointCloudRegister
{

protected:
    bool isConverge;
    std::shared_ptr<utils::logger::Logger> lg;

public:
    using Ptr = std::shared_ptr<PointCloudRegister>;
    using cPtr = std::shared_ptr<const PointCloudRegister>;

    using PC_Ptr = typename PC<pt_t>::Ptr;
    using PC_cPtr = typename PC<pt_t>::ConstPtr;

public:
    PointCloudRegister(){
        lg = utils::logger::Logger::getInstance();
    }
    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) = 0;

    virtual ~PointCloudRegister(){}
};

}

