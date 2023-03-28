#include <backend/Backend.hpp>

#include <pcl/io/pcd_io.h>

#include <utils/Logger.hpp>

namespace backend
{

Backend::Backend(std::string pcd_file)
{
    // load global map mode
    mSubMap = pcl::make_shared<PCxyz>();
    mSubMapKdtree = pcl::make_shared<pcl::KdTreeFLANN<Pxyz>>();
    
    if(pcl::io::loadPCDFile<Pxyz>(pcd_file, *mSubMap) == -1)
    {
        auto msg = fmt::format("can't load globalmap from: {}", pcd_file);
        mLg->error(msg);
        throw std::runtime_error(msg);
    }    
}

const pcl::KdTreeFLANN<Pxyz>::Ptr& Backend::getSubMapKdtree() const
{
    return mSubMapKdtree;
}

const PCxyz::Ptr& Backend::getSubMap() const
{
    return mSubMap;
}

}