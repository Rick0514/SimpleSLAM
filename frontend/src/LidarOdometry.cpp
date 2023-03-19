#include <frontend/LidarOdometry.hpp>

namespace frontend
{

LidarOdometry::LidarOdometry(const std::shared_ptr<Frontend>& ft)
{
    mFrontendPtr = ft;    
}

void LidarOdometry::generateOdom()
{

    // get current scan
    // get submap from backend

    // make init pose

    // use pcr to get refined pose

    // push the refined odom to deque  
}

    
} // namespace frontend

