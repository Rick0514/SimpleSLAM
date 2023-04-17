#include <frontend/Frontend.hpp>
#include <frontend/LidarOdometry.hpp>

namespace frontend {

Frontend::Frontend(int local_size, int global_size){
    mLocalOdometry = std::make_shared<OdomDeque>(local_size);
    mGlobalOdometry = std::make_shared<OdomDeque>(global_size);
    mOdom2Map.setIdentity();
}

void Frontend::run(OdometryBase* lo)
{
    mLO.reset(lo);
    mLOthdPtr = std::make_unique<trd::ResidentThread>([&](){
        mLO->generateOdom();
    });
}

Odometry::Ptr Frontend::getClosestLocalOdom(double stamp) const
{
    // 1.  stamp > back()
    // 1.1 scope block ?? wait for closest local odom
    // 1.2 

    // maybe 0.02 should be subtituded to local freq
    // if(stamp - mLocalOdometry->back()->stamp >= 0.02)
    // {

    // } 

    return Odometry::Ptr();

    // 2. stamp <= back()
}

Frontend::~Frontend() {
    mLocalOdometry->abort();
    mGlobalOdometry->abort();    
}


}