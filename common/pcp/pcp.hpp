#pragma once

// for point cloud preprocess
#include <pcl/filters/filter.h>
#include <types/PCLTypes.hpp>
#include <pcl/filters/voxel_grid.h>

namespace pcp {
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

}
