#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <types/EigenTypes.hpp>
#include <macro/templates.hpp>

#include <utils/Logger.hpp>

namespace PCR
{
using namespace EigenTypes;

template<typename PointType>
class PointCloudRegister
{

protected:
    bool isConverge;
    std::shared_ptr<utils::logger::Logger> lg;

public:
    using Ptr = std::shared_ptr<PointCloudRegister<PointType>>;
    using cPtr = std::shared_ptr<const PointCloudRegister<PointType>>;
    using PC_Ptr = typename pcl::PointCloud<PointType>::Ptr;
    using PC_cPtr = const typename pcl::PointCloud<PointType>::Ptr;

public:
    PointCloudRegister(){
        lg = utils::logger::Logger::getInstance();
    }
    virtual bool scan2Map(PC_cPtr& src, PC_cPtr& dst, Pose6d& res) = 0;

    virtual ~PointCloudRegister(){}
};

}

