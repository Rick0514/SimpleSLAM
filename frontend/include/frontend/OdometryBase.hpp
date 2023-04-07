#pragma once
#include <memory>
#include <utils/Logger.hpp>

namespace frontend
{
using namespace utils;

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

