#pragma once

#include <utils/Logger.hpp>
#include <types/PCLTypes.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace backend
{

using namespace utils;
using namespace PCLTypes;

template <typename PointType>
class Backend
{
private:

    typename pcl::PointCloud<PointType>::Ptr mSubMap;
    typename pcl::KdTreeFLANN<PointType>::Ptr mSubMapKdtree;

    std::shared_ptr<logger::Logger> mLg;

public:

    // pcd mode
    Backend(std::string pcd_file);

    const typename pcl::KdTreeFLANN<PointType>::Ptr& getSubMapKdtree() const;
    const typename pcl::PointCloud<PointType>::Ptr& getSubMap() const;
    
    ~Backend(){};
};
    
} // namespace backend



