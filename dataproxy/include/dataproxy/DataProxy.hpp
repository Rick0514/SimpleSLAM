#pragma once

#include <types/EigenTypes.hpp>
#include <utils/SafeDeque.hpp>

using namespace EigenTypes;
using namespace utils;

namespace dataproxy
{

struct Odometry
{
    double stamp;
    Pose6d odom;
    Odometry(){}
    Odometry(double t, const Pose6d& p) : stamp(t), odom(p){}
};

template <typename T, bool UseBag>
class DataProxy
{
public:
    using DataPtr = std::shared_ptr<concurrency::SafeDeque<T, UseBag>>;

protected:
    DataPtr mDataPtr;

public:
    explicit DataProxy(int size){
        mDataPtr = std::make_shared<concurrency::SafeDeque<T, UseBag>>(size);
    }
    
    const DataPtr get() const { return mDataPtr; }  
    
    virtual ~DataProxy() {};
};
    
} // namespace dataproxy





