#include <frontend/Frontend.hpp>
#include <frontend/LidarOdometry.hpp>

namespace frontend {

void Frontend::init() noexcept
{
    lg = logger::Logger::getInstance();
    mOdom2MapInitFlag = false;
    mOdom2Map.store(pose_t::Identity());
}

Frontend::Frontend(const OdomDequePtr& local_dq, int global_size) : mLocalOdometry(local_dq)
{
    mGlobalOdometry = std::make_shared<OdomDeque>(global_size);
    init();
}

Frontend::Frontend(int local_size, int global_size){
    mLocalOdometry = std::make_shared<OdomDeque>(local_size);
    mGlobalOdometry = std::make_shared<OdomDeque>(global_size);
    init();
}

Odometry::Ptr Frontend::getClosestLocalOdom(double stamp) const
{
    // 1.  stamp > back()
    // 1.1 spin block ?? wait for closest local odom
    std::unique_lock<std::mutex> lk(mLocalOdometry->getLock());
    auto& dq = mLocalOdometry->getDequeInThreadUnsafeWay();
    
    // try multiple time!!
    const std::chrono::milliseconds timeout{5};
    const int try_time{4};
    for(int i=0; i<try_time; i++){
        auto it = std::lower_bound(dq.begin(), dq.end(), stamp, [](const Odometry::Ptr& it, stamp_t v){
            return it->stamp < v;
        });

        if(it != dq.end()){
            // pop useless odom
            while(dq.begin() != it)  dq.pop_front();
            mLocalOdometry->getCv().notify_all();
            return *it;
        }
        lk.unlock();
        std::this_thread::sleep_for(timeout);
        lk.lock();
    }

    return Odometry::Ptr();
}

Frontend::~Frontend() {
    lg->info("exit frontend!");
    mLocalOdometry->abort();
    mGlobalOdometry->abort();   
}


}