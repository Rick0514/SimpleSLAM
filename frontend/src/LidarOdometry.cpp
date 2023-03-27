#include <frontend/LidarOdometry.hpp>

namespace frontend
{

template <bool UseBag>
LidarOdometry<UseBag>::LidarOdometry(ConstDataProxyPtr& dp, ConstFrontendPtr& ft, ConstBackendPtr& bk)
: mDataProxyPtr(dp), mFrontendPtr(ft), mBackendPtr(bk)
{
    // xyz for temp
    mPcr.reset(new PCR::LoamRegister<Pxyz>());
}

template <bool UseBag>
void LidarOdometry<UseBag>::generateOdom()
{
    // get current scan
    const auto& scans = mDataProxyPtr->get();
    
    // get submap from backend -- may be need mutex !!
    const auto& submap = mBackendPtr->getSubMap();

    // make init pose
    // 1. get latest scan
    PCxyz::Ptr scan(new PCxyz());
    pcl::copyPointCloud(*scans->back(), *scan);
    // 2. localodom * odom2map
    auto odom2map = mFrontendPtr->get();
    // find closest localodom
    double stamp;
    Odometry localodom;
    mFrontendPtr->getClosestLocalOdom(stamp, localodom);

    Pose6d init_pose;
    init_pose.matrix() = localodom.odom.matrix() * odom2map.matrix();
    
    // use pcr to get refined pose
    mPcr->scan2Map(scan, submap, init_pose);

    mFrontendPtr->pushGlobalOdometry(Odometry(stamp, init_pose));
    // push the refined odom to deque  

}

    
} // namespace frontend

