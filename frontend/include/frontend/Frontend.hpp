#pragma once

#include <types/EigenTypes.hpp>

#include <utils/SafeDeque.hpp>

#include <dataproxy/DataProxy.hpp>

namespace frontend
{
using namespace EigenTypes;
using namespace utils;
using namespace dataproxy;

class Frontend
{
protected:
    using OdomDequePtr = std::shared_ptr<concurrency::SafeDeque<Odometry>>;
    using ConstOdomDequePtrRef = const OdomDequePtr&;

    Pose6d mOdom2Map;

    std::shared_ptr<concurrency::SafeDeque<Odometry>> mLocalOdometry;
    std::shared_ptr<concurrency::SafeDeque<Odometry>> mGlobalOdometry;

public:
    Frontend();

    void publish() const;

    Pose6d get() const { return mOdom2Map; }

    Odometry::Ptr getClosestLocalOdom(double stamp) const;

    OdomDequePtr& getLocal() { return mLocalOdometry; }
    OdomDequePtr& getGlobal() { return mGlobalOdometry; } 

    // q should be pointer to iterable container of shared ptr to Odometry
    template<typename T>
    static int getClosestItem(T&& q, double stamp);

    ~Frontend(){};
};

template<typename T>
int Frontend::getClosestItem(T&& q, double stamp)
{
    if(q->empty())  return -1;
    int idx = 0;
    double m = std::abs(stamp - q->front()->stamp);

    for(int i=1; i<q->size(); i++){
        double tmp = std::abs(q->at(i)->stamp - stamp);
        if(tmp < m){
            m = tmp;
            idx = i; 
        }
    }

    return idx;
}

} // namespace frontend


