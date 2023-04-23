#include <frontend/Frontend.hpp>
#include <frontend/LidarOdometry.hpp>

namespace frontend {

Frontend::Frontend(int local_size, int global_size){
    lg = logger::Logger::getInstance();

    mLocalOdometry = std::make_shared<OdomDeque>(local_size);
    mGlobalOdometry = std::make_shared<OdomDeque>(global_size);
    mOdom2Map.store(pose_t::Identity());
}

void Frontend::run(std::unique_ptr<LidarOdometry>&& lo)
{
    mLO = std::move(lo);
    mLOthdPtr = std::make_unique<trd::ResidentThread>([&](){
        mLO->generateOdom();
    });
}

Odometry::Ptr Frontend::getClosestLocalOdom(double stamp) const
{
    // 1.  stamp > back()
    // 1.1 scope block ?? wait for closest local odom
    std::unique_lock<std::mutex> lk(mLocalOdometry->getLock());
    auto& dq = mLocalOdometry->getDequeInThreadUnsafeWay();
    
    // try multiple time!!
    const std::chrono::milliseconds timeout{5};
    const int try_time{5};
    for(int i=0; i<try_time; i++){
        auto it = std::lower_bound(dq.begin(), dq.end(), stamp, [](const Odometry::Ptr& it, stamp_t v){
            return it->stamp < v;
        });

        if(it != dq.end()){
            return *it;
        }
        lk.unlock();
        std::this_thread::sleep_for(timeout);
        lk.lock();
    }

    return Odometry::Ptr();
}

Frontend::~Frontend() {
    mLocalOdometry->abort();
    mGlobalOdometry->abort();   
    lg->info("exit frontend!"); 
}


}