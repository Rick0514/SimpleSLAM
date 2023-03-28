#pragma once

#include <types/PCLTypes.hpp>
#include <utils/Logger.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace backend
{

using namespace PCLTypes;
using namespace utils;

class Backend
{
private:

    PCxyz::Ptr mSubMap;
    pcl::KdTreeFLANN<Pxyz>::Ptr mSubMapKdtree;

    std::shared_ptr<logger::Logger> mLg;

public:

    // pcd mode
    Backend(std::string pcd_file);

    const pcl::KdTreeFLANN<Pxyz>::Ptr& getSubMapKdtree() const;
    const PCxyz::Ptr& getSubMap() const;
    
    ~Backend(){};
};
    
} // namespace backend



