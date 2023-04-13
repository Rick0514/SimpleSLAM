#pragma once

#include <types/PCLTypes.hpp>
#include <macro/templates.hpp>

#include <utils/Logger.hpp>

namespace PCR
{
using namespace EigenTypes;
using namespace PCLTypes;

template<typename PointType>
class PointCloudRegister
{

protected:
    bool isConverge;
    std::shared_ptr<utils::logger::Logger> lg;

public:
    using Ptr = std::shared_ptr<PointCloudRegister<PointType>>;
    using cPtr = std::shared_ptr<const PointCloudRegister<PointType>>;

    using PC_Ptr = typename PC<PointType>::Ptr;
    using PC_cPtr = typename PC<PointType>::ConstPtr;

public:
    PointCloudRegister(){
        lg = utils::logger::Logger::getInstance();
    }
    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, Pose6d& res) = 0;

    virtual ~PointCloudRegister(){}
};

}

