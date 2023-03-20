#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace PCLTypes
{
    using Pxyz = pcl::PointXYZ;
    using Pxyzi = pcl::PointXYZI;

    using PCxyz = pcl::PointCloud<Pxyz>;
    using PCxyzi = pcl::PointCloud<Pxyzi>;
} // namespace PCLTypes
