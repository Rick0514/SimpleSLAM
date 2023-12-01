#pragma once

#include <types/basic.hpp>

#include <utils/SafeDeque.hpp>
#include <utils/Thread.hpp>
#include <utils/Atomic.hpp>
#include <utils/Logger.hpp>

#include <filter/Eskf.hpp>

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

    bool mOdom2MapInitFlag;
    trd::AtomicVar<pose_t> mOdom2Map;

    // OdomDequePtr mLocalOdometry;
    filter::State* mLocalOdometry;
    OdomDequePtr mGlobalOdometry;

    std::shared_ptr<logger::Logger> lg;


public:
    Frontend() = delete;
    void init() noexcept;
    // Frontend(const OdomDequePtr& local_dq, int global_size);    // if local dq is already get!!
    Frontend(int local_size, int global_size);
    Frontend(filter::State* eskf, int global_size);

    void setInitOdom2Map() noexcept { mOdom2MapInitFlag = true; }
    bool isInitOdom2Map() noexcept { return mOdom2MapInitFlag; }

    void publish() const;

    auto& get() { return mOdom2Map; }

    // Odometry::Ptr getClosestLocalOdom(double stamp) const;

    // OdomDequePtr& getLocal() { return mLocalOdometry; }
    filter::State* getLocal() { return mLocalOdometry; }
    OdomDequePtr& getGlobal() { return mGlobalOdometry; }

    ~Frontend();

    // q should be iterable container of shared ptr to Odometry with
    // accending time stamp
    template<typename T>
    static int getClosestItem(const T& q, stamp_t stamp)
    {
        if(q.empty())  return -1;
        
        int n = q.size();
        int idx = n-1;
        stamp_t m = std::abs(stamp - q.back()->stamp);

        for(int i=n-2; i>=0; i--){
            stamp_t tmp = std::abs(q.at(i)->stamp - stamp);
            if(tmp < m){
                m = tmp;
                idx = i; 
            }else   break;
        }

        return idx;
    }

};

} // namespace frontend

