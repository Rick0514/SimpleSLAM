#pragma once
#include <types/EigenTypes.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace PCLTypes
{
    using Pxyz = pcl::PointXYZ;
    using Pxyzi = pcl::PointXYZI;

    template<typename PointType>
    using PC = pcl::PointCloud<PointType>;

    using PCxyz = pcl::PointCloud<Pxyz>;
    using PCxyzi = pcl::PointCloud<Pxyzi>;

    template<typename PointType, typename Scalar=double>
    struct KeyFrame
    {
        EigenTypes::Pose6<Scalar> pose;
        typename PC<PointType>::Ptr pc;    
    };

} // namespace PCLTypes
