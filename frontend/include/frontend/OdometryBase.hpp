#pragma once
#include <memory>
#include <types/EigenTypes.hpp>
#include <utils/Logger.hpp>

namespace frontend
{
using namespace utils;
using namespace EigenTypes;

class OdometryBase
{
protected:
    std::shared_ptr<logger::Logger> lg;
    
public:
    OdometryBase() { lg = logger::Logger::getInstance(); }

    virtual void generateOdom() = 0;

    virtual ~OdometryBase(){ lg->info("exit odombase!!"); }
};
    
} // namespace frontend

