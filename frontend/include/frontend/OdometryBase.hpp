#pragma once
#include <memory>
#include <types/EigenTypes.hpp>
#include <utils/Logger.hpp>

namespace frontend
{
using namespace utils;
using namespace EigenTypes;

struct Odometry
{
    double stamp;
    Pose6d odom;
    Odometry(){}
    Odometry(double t, const Pose6d& p) : stamp(t), odom(p){}

    using Ptr = std::shared_ptr<Odometry>;
};

class OdometryBase
{
protected:
    std::shared_ptr<logger::Logger> lg;
public:
    OdometryBase() { lg = logger::Logger::getInstance(); }

    virtual void generateOdom() = 0;

    virtual ~OdometryBase(){}
};
    
} // namespace frontend

