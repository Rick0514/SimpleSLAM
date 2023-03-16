#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <types/EigenTypes.hpp>
#include <macro/templates.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>

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
    using cPtr = std::shared_ptr<const PointCloudRegister<PointType>>;
    using PC_Ptr = typename pcl::PointCloud<PointType>::Ptr;
    using PC_cPtr = const typename pcl::PointCloud<PointType>::Ptr;

public:
    PointCloudRegister(){
#ifdef LOG_LEVEL
        spdlog::set_level(static_cast<spdlog::level::level_enum>(LOG_LEVEL));
#endif
    }
    virtual bool scan2Map(PC_cPtr& src, PC_cPtr& dst, Pose6d& res) = 0;

};

}

