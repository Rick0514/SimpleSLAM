#include <backend/Backend.hpp>
#include <frontend/LidarOdometry.hpp>
#include <memory>

namespace frontend
{

template <typename PointType, bool UseBag>
LidarOdometry<PointType, UseBag>::LidarOdometry(ConstDataProxyPtr& dp, ConstFrontendPtr& ft, ConstBackendPtr& bk)
: mDataProxyPtr(dp), mFrontendPtr(ft), mBackendPtr(bk)
{
    // xyz for temp
    mPcr.reset(new PCR::LoamRegister<Pxyz>());
}

template <typename PointType, bool UseBag>
void LidarOdometry<PointType, UseBag>::generateOdom()
{
    // get current scan
    const auto& scans = mDataProxyPtr->get();
    
    // get submap from backend -- may be need mutex !!
    const auto& submap = mBackendPtr->getSubMap();

    // make init pose
    // 1. get latest scan
    typename PointType::Ptr scan(new PointType());
    pcl::copyPointCloud(*scans->back(), *scan);
    // 2. localodom * odom2map
    auto odom2map = mFrontendPtr->get();
    
    // find closest localodom
    // think how to get the stamp
    double stamp;
    auto local_odom = mFrontendPtr->getClosestLocalOdom(stamp);

    Pose6d init_pose;
    init_pose.matrix() = local_odom->odom.matrix() * odom2map.matrix();
    
    // use pcr to get refined pose
    mPcr->scan2Map(scan, submap, init_pose);

    auto global_odom = std::make_shared<Odometry>(stamp, init_pose);
    mFrontendPtr->pushGlobalOdometry(std::move(global_odom));

    // push the refined odom to deque  

}

    
} // namespace frontend

