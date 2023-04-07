#include <frontend/Frontend.hpp>

namespace frontend {

Frontend::Frontend()
{

}

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

}