#pragma once

// for point cloud preprocess
#include <pcl/filters/filter.h>
#include <types/PCLTypes.hpp>
#include <pcl/io/pcd_io.h>
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

#pragma omp parallel for num_threads(2)
    for (int i = 0; i < cloudSize; ++i)
    {
        Eigen::Map<Eigen::Vector4f, Eigen::Aligned> pfrom = cloudIn->points[i].getVector4fMap();
        Eigen::Map<Eigen::Vector4f, Eigen::Aligned> pto = cloudOut->points[i].getVector4fMap();
        pto.noalias() = tr * pfrom;
        if constexpr (std::is_same_v<PointType, Pxyzi>){
            cloudOut->points[i].intensity = cloudIn->points[i].intensity;
        }
    }
    return cloudOut;
}

// assume it wont throw exception
template <typename PointType>
void savePCDFile(const std::string& file, const typename PC<PointType>::Ptr& cloud)
{
    pcl::io::savePCDFileBinary(file, *cloud);
}

template <typename PointType>
void loadPCDFile(const std::string& file, typename PC<PointType>::Ptr& cloud)
{
    pcl::io::loadPCDFile<PointType>(file, *cloud);
}

// adaptor from https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelHashMap.hpp
class VoxelDownSampleV2
{
protected:
    using Voxel = Eigen::Vector3i;
    
    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
        }
    };

    std::unordered_map<Voxel, std::vector<int>, VoxelHash> _map;

    float _grid_size;
    int _max_voxel_pts;
    int _min_voxel_pts;

public:

    VoxelDownSampleV2() = default;
    VoxelDownSampleV2(float gs, int max_vp=10, int min_vp=0) : _grid_size(gs), _max_voxel_pts(max_vp), _min_voxel_pts(min_vp) {}

    template <typename PointType>
    void filter(const typename PC<PointType>::ConstPtr& cloudIn, PC<PointType>& cloudOut)
    {
        _map.clear();

        for(int i=0; i<cloudIn->points.size(); i++){
            Eigen::Vector3f p = cloudIn->points[i].getVector3fMap();
            Voxel vx = (p / _grid_size).cast<int>();
            if(_map.find(vx) == _map.end()){
                std::vector<int> vb;
                vb.emplace_back(i);
                _map.emplace(vx, vb);
            }else{
                if(_map[vx].size() < _max_voxel_pts)    _map[vx].emplace_back(i);
            }
        }

        // return the first one
        typename PC<PointType>::Ptr out = pcl::make_shared<PC<PointType>>();
        // worse case
        cloudOut.points.reserve(_max_voxel_pts * _map.size());

        for(const auto& [vx, vb] : _map){
            if(vb.size() > _min_voxel_pts){
                auto sid = vb.front();
                Eigen::Vector3f p;
                p.setZero();

                for(auto&& idx : vb)    p += cloudIn->points[idx].getVector3fMap();
                p /= vb.size();

                PointType pt = cloudIn->points[vb.front()];
                pt.getVector3fMap() = p;
                cloudOut.points.emplace_back(pt);
                // cloudOut.points.emplace_back(cloudIn->points[sid]);
            }
        }
    }

};

}