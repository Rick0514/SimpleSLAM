#pragma once

// for point cloud preprocess
#include <pcl/filters/filter.h>
#include <types/PCLTypes.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>

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
void voxelDownSample(const typename PC<PointType>::ConstPtr& cloudIn, PC<PointType>& cloudOut, float grid_size){
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(grid_size, grid_size, grid_size);
    voxelgrid.setInputCloud(cloudIn);
    voxelgrid.filter(cloudOut);
}

template<typename PointType>
void removeNaNFromPointCloud(PC<PointType> &cloud)
{
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(cloud, cloud, index);
}

template<typename PointType, typename Scalar=float>
void transformPointCloud(const PC<PointType>& cloudIn, PC<PointType>& cloudOut, Pose6<Scalar> trans)
{
    Pose6<float> tr;
    if constexpr (std::is_same_v<Scalar, float>){
        tr = trans;
    }else{
        tr = trans.template cast<float>();
    }

    cloudOut.header = cloudIn.header;

    auto cloudSize = cloudIn.size();
    cloudOut.resize(cloudSize);

    #pragma omp parallel for num_threads(2)
    for (int i = 0; i < cloudSize; ++i)
    {
        pcl::Vector3fMapConst pfrom = cloudIn.points[i].getVector3fMap();
        pcl::Vector3fMap pto = cloudOut.points[i].getVector3fMap();
        pto.noalias() = tr * pfrom;
        if constexpr (std::is_same_v<PointType, Pxyzi>){
            cloudOut.points[i].intensity = cloudIn.points[i].intensity;
        }
    }
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

// adapt from https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelHashMap.hpp
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
    float _max_range{1000.0f};

public:

    VoxelDownSampleV2() = default;
    VoxelDownSampleV2(float gs, int max_vp=20, int min_vp=0) : _grid_size(gs), _max_voxel_pts(max_vp), _min_voxel_pts(min_vp) {}

    template <typename PointType>
    typename PC<PointType>::Ptr filter(const typename PC<PointType>::ConstPtr& cloud)
    {
        _map.clear();

        for(int i=0; i<cloud->size(); i++){
            Eigen::Vector3f p = cloud->points[i].getVector3fMap();
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
        out->reserve(_max_voxel_pts * _map.size());

        for(const auto& [vx, vb] : _map){
            if(vb.size() > _min_voxel_pts){
                // auto sid = vb.front();
                // PointType pt = cloud->points[sid];

                // bool select = true;
                // for(int i=0; i<3; i++){
                //     if(std::isnan(sp.data[i]) || sp.data[i] < -_max_range || sp.data[i] > _max_range)
                //     {
                //         select = false;
                //         break;
                //     }
                // }
                // if(select)  out->emplace_back(sp);

                Eigen::Vector3f p;
                p.setZero();

                for(auto&& idx : vb)    p += cloud->points[idx].getVector3fMap();
                p /= vb.size();

                PointType pt = cloud->points[vb.front()];
                pt.getVector3fMap() = p;
                out->emplace_back(pt);
                // out.points.emplace_back(pt);
            }
        }
        return out;
    }

};

// simplified pcl implementation
class VoxelDownSampleV3
{
protected:

    float _grid_size;
    float _inverse_grid_size;

    Eigen::Array3f _max_p, _min_p;

public:
    VoxelDownSampleV3() = default;
    VoxelDownSampleV3(float gs) : _grid_size(gs) {
        _inverse_grid_size = 1.0f / _grid_size;
    }

    template <typename PointType>
    void filter(const typename PC<PointType>::ConstPtr& cloud, typename PC<PointType>::Ptr& out)
    {
        if(cloud->empty())  return;

        out->header = cloud->header;
        out->sensor_origin_ = cloud->sensor_origin_;
        out->sensor_orientation_ = cloud->sensor_orientation_;

        _max_p = cloud->points[0].getArray3fMap();
        _min_p = _max_p;

        // find max and min first
        for(const auto& p : cloud->points){
            pcl::Array3fMapConst pt = p.getArray3fMap();
            _max_p = _max_p.max(pt);
            _min_p = _min_p.min(pt);
        }

        Eigen::Array3i max_b, min_b;
        min_b[0] = static_cast<int> (std::floor (_min_p[0] * _inverse_grid_size));
        max_b[0] = static_cast<int> (std::floor (_max_p[0] * _inverse_grid_size));
        min_b[1] = static_cast<int> (std::floor (_min_p[1] * _inverse_grid_size));
        max_b[1] = static_cast<int> (std::floor (_max_p[1] * _inverse_grid_size));
        min_b[2] = static_cast<int> (std::floor (_min_p[2] * _inverse_grid_size));
        max_b[2] = static_cast<int> (std::floor (_max_p[2] * _inverse_grid_size));

        int dx = max_b[0] - min_b[0] + 1;
        int dy = max_b[1] - min_b[1] + 1;

        std::unordered_map<size_t, std::vector<size_t>> _map;

        for(size_t i=0; i<cloud->size(); i++){
            const auto& pt = cloud->points[i];
            size_t ijk0 = static_cast<size_t> (std::floor (pt.x * _inverse_grid_size) - static_cast<float> (min_b[0]));
            size_t ijk1 = static_cast<size_t> (std::floor (pt.y * _inverse_grid_size) - static_cast<float> (min_b[1]));
            size_t ijk2 = static_cast<size_t> (std::floor (pt.z * _inverse_grid_size) - static_cast<float> (min_b[2]));

            size_t idx = ijk0 + ijk1 * dx + ijk2 * dx * dy;

            if(_map.count(idx) == 0){
                _map.emplace(idx, std::vector<size_t>{i});
            }else{
                _map[idx].emplace_back(i);
            }
        }
        
        PC<PointType>& out_ = *out;
        PC<PointType> out_tmp;

        if(cloud.get() == out.get())
            out_ = out_tmp;

        out_.clear();
        out_.reserve(_map.size());

        // for(const auto& [_, vb] : _map){
        //     // Compute the centroid leaf index
        //     Eigen::Vector3f p;
        //     p.setZero();

        //     for(auto&& idx : vb)    p += cloud->points[idx].getVector3fMap();
        //     p /= vb.size();

        //     PointType pt = cloud->points[vb.front()];
        //     pt.getVector3fMap() = p;
        //     out->emplace_back(pt);
        // }

        for(const auto& [_, vb] : _map){
            pcl::CentroidPoint<PointType> centroid;

            // fill in the accumulator with leaf points
            for (auto&& idx : vb)
                centroid.add(cloud->points[idx]);  
            
            PointType pt;
            centroid.get(pt);
            out_.emplace_back(pt);
        }

        if(cloud.get() == out.get())    pcl::copyPointCloud(out_tmp, *out);
    }

    std::string getMaxMin()
    {
        std::stringstream ss;
        ss << "min: " << _min_p.transpose() << " "
           << "max: " << _max_p.transpose() << std::endl;
        return ss.str();
    }
};

}