#include <frontend/Frontend.hpp>
#include <frontend/OdometryBase.hpp>
#include <frontend/LidarOdometry.hpp>

namespace frontend {

Frontend::Frontend() : mLORunning(true) {}

// ---------------------------------------------------------------------
template<typename PointType, bool UseBag>
void Frontend::initLO(DataProxyPtr<PointType, UseBag>& dp, BackendPtr<PointType>& ed)
{
    mLO = std::make_unique<LidarOdometry<PointType, UseBag>>(dp, shared_from_this(), ed);
    mLOthdPtr = std::make_unique<std::thread>(&Frontend::LOHandler, this);
}

template<> void Frontend::initLO<Pxyz>(DataProxyPtr<Pxyz> &dp, BackendPtr<Pxyz> &ed);
template<> void Frontend::initLO<Pxyzi>(DataProxyPtr<Pxyzi> &dp, BackendPtr<Pxyzi> &ed);
template<> void Frontend::initLO<Pxyz, true>(DataProxyPtr<Pxyz, true> &dp, BackendPtr<Pxyz> &ed);
template<> void Frontend::initLO<Pxyzi, true>(DataProxyPtr<Pxyzi, true> &dp, BackendPtr<Pxyzi> &ed);

void Frontend::LOHandler()
{
    while(mLORunning.load())
    {
        mLO->generateOdom();
    }
}

// ---------------------------------------------------------------------

Odometry::Ptr Frontend::getClosestLocalOdom(double stamp) const
{
    // 1.  stamp > back()
    // 1.1 scope block ?? wait for closest local odom
    // 1.2 

    // maybe 0.02 should be subtituded to local freq
    if(stamp - mLocalOdometry->back()->stamp >= 0.02)
    {

    } 
    return Odometry::Ptr();

    // 2. stamp <= back()
}

Frontend::~Frontend()
{

}

}