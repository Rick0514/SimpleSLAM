#include <utils/SafeDeque.hpp>
#include <types/EigenTypes.hpp>

#include <memory>

namespace frontend
{

using namespace EigenTypes;
using namespace utils;

struct Odometry
{
    double stamp;
    Pose6d odom;
    Odometry(double t, const Pose6d& p) : stamp(t), odom(p){}
};


class OdometryBase
{
protected:

    std::shared_ptr<concurrency::SafeDeque<Odometry>> mOdomDeque;
    
public:
    OdometryBase();

    virtual void generateOdom() = 0;

    ~OdometryBase();
};
    
} // namespace frontend

