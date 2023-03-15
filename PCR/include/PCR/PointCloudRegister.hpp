#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <types/EigenTypes.hpp>
#include <macro/templates.hpp>

namespace PCR
{
using namespace EigenTypes;

template<typename PointType>
class PointCloudRegister
{

protected:
    bool isConverge;

public:
    using Ptr = std::shared_ptr<PointCloudRegister<PointType>>;
    using PC_Ptr = typename pcl::PointCloud<PointType>::Ptr;

public:
    PointCloudRegister(){}
    virtual bool scan2Map(PC_Ptr src, PC_Ptr dst, Pose6d& res) = 0;

};

}

