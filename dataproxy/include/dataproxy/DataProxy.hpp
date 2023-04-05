#pragma once

#include <memory>
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

    using Ptr = std::shared_ptr<Odometry>;
};

template <typename T, bool UseBag=false>
class DataProxy
{
public:

    using SafeDequeType = concurrency::SafeDeque<T>;
    using DataPtr = std::shared_ptr<SafeDequeType>;

protected:
    DataPtr mDataPtr;

public:
    explicit DataProxy(int size){
        mDataPtr = std::make_shared<SafeDequeType>(size);
    }
    
    DataPtr get() const { return mDataPtr; }  
    
    virtual ~DataProxy() {};
};
    
} // namespace dataproxy





