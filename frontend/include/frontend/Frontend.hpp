#pragma once

#include <types/basic.hpp>

#include <utils/SafeDeque.hpp>
#include <utils/Thread.hpp>
#include <utils/Atomic.hpp>
#include <utils/Logger.hpp>

namespace frontend
{

using namespace EigenTypes;
using namespace utils;

class LidarOdometry;

class Frontend
{
protected:

    using OdomDeque = concurrency::SafeDeque<Odometry>;
    using OdomDequePtr = std::shared_ptr<OdomDeque>;

    trd::AtomicVar<pose_t> mOdom2Map;

    OdomDequePtr mLocalOdometry;
    OdomDequePtr mGlobalOdometry;

    std::unique_ptr<LidarOdometry> mLO;
    std::unique_ptr<trd::ResidentThread> mLOthdPtr;

    std::shared_ptr<logger::Logger> lg;

public:
    Frontend() = delete;
    Frontend(int local_size, int global_size);

    void publish() const;

    auto& get() { return mOdom2Map; }

    void run(std::unique_ptr<LidarOdometry>&& lo);

    Odometry::Ptr getClosestLocalOdom(double stamp) const;

    OdomDequePtr& getLocal() { return mLocalOdometry; }
    OdomDequePtr& getGlobal() { return mGlobalOdometry; }

    ~Frontend();

    // q should be pointer to iterable container of shared ptr to Odometry
    template<typename T>
    static int getClosestItem(const T& q, double stamp)
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

};

} // namespace frontend

