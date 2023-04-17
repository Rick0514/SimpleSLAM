#pragma once

// for point cloud preprocess
#include <pcl/filters/filter.h>
#include <types/PCLTypes.hpp>
#include <pcl/filters/voxel_grid.h>

namespace pcp {
using namespace EigenTypes;
using namespace PCLTypes;

template<typename PointType>
void voxelDownSample(typename PC<PointType>::Ptr& cloud, float grid_size){
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(grid_size, grid_size, grid_size);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*cloud);
}

template<typename PointType>
void removeNaNFromPointCloud(PC<PointType> &cloud)
{
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(cloud, cloud, index);
}

template<typename PointType, typename Scalar=float>
typename PC<PointType>::Ptr transformPointCloud(typename PC<PointType>::Ptr cloudIn, Pose6<Scalar> trans)
{
    typename PC<PointType>::Ptr cloudOut(new PC<PointType>());

    Pose6<float> tr;
    if constexpr (std::is_same_v<Scalar, float>){
        tr = trans;
    }else{
        tr = trans.template cast<float>();
    }

    auto cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

#pragma omp parallel for num_threads(4)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto pfrom = cloudIn->points[i].getVector3fMap();
        auto pto = cloudOut->points[i].getVector3fMap();
        pto = tr * pfrom;
        if constexpr (std::is_same_v<PointType, Pxyzi>){
            cloudOut->points[i].intensity = cloudIn->points[i].intensity;
        }
    }
    return cloudOut;
}

}
