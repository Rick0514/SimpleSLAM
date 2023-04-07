#include <backend/Backend.hpp>

#include <macro/templates.hpp>
#include <pcl/io/pcd_io.h>

namespace backend
{

using namespace PCLTypes;

template<typename PointType>
Backend<PointType>::Backend(std::string pcd_file)
{
    mLg = logger::Logger::getInstance();
    // load global map mode
    mSubMap = pcl::make_shared<PC<PointType>>();
    mSubMapKdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
    
    if(pcl::io::loadPCDFile<PointType>(pcd_file, *mSubMap) == -1)
    {
        auto msg = fmt::format("can't load globalmap from: {}", pcd_file);
        mLg->error(msg);
        throw std::runtime_error(msg);
    }

    mLg->info("load map success!!");
}

template<typename PointType>
const typename pcl::KdTreeFLANN<PointType>::Ptr& Backend<PointType>::getSubMapKdtree() const
{
    return mSubMapKdtree;
}

template<typename PointType>
const typename pcl::PointCloud<PointType>::Ptr& Backend<PointType>::getSubMap() const
{
    return mSubMap;
}

PCTemplateInstantiateExplicitly(Backend);
}