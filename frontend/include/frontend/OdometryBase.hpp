#include <utils/SafeDeque.hpp>
#include <types/EigenTypes.hpp>

#include <memory>

namespace frontend
{

static constexpr bool UseBag = false;

using namespace EigenTypes;
using namespace utils;

struct Odometry
{
    double stamp;
    Pose6d odom;
    Odometry(){}
    Odometry(double t, const Pose6d& p) : stamp(t), odom(p){}
};


class OdometryBase
{
protected:

public:
    OdometryBase();

    virtual void generateOdom() = 0;

    ~OdometryBase();
};
    
} // namespace frontend

