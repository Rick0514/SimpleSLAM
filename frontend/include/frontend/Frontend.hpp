#pragma once

#include <memory>
#include <types/EigenTypes.hpp>
#include <types/PCLTypes.hpp>

#include <utils/SafeDeque.hpp>
#include <utils/Thread.hpp>
#include <utils/Logger.hpp>

#include <dataproxy/DataProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

#include <frontend/OdometryBase.hpp>
#include <frontend/LidarOdometry.hpp>

namespace backend { template<typename PointType> class Backend; }

namespace frontend
{

using namespace backend;
using namespace dataproxy;

using namespace EigenTypes;
using namespace PCLTypes;
using namespace utils;

template<typename PointType, bool UseBag=false>
using LODataProxyPtr = typename LidarOdometry<PointType, UseBag>::DataProxyPtr;

template<typename PointType>
using BackendPtr = std::shared_ptr<Backend<PointType>>;

class Frontend : public std::enable_shared_from_this<Frontend>
{
protected:

    using OdomDeque = concurrency::SafeDeque<Odometry>;
    using OdomDequePtr = std::shared_ptr<OdomDeque>;

    Pose6d mOdom2Map;

    OdomDequePtr mLocalOdometry;
    OdomDequePtr mGlobalOdometry;

    std::unique_ptr<OdometryBase> mLO;
    std::unique_ptr<thread::ResidentThread> mLOthdPtr;

public:
    Frontend() = delete;
    Frontend(int local_size, int global_size);

    template<typename PointType, bool UseBag=false>
    void initLO(LODataProxyPtr<PointType, UseBag>&, BackendPtr<PointType>&);
    
    template<typename PointType, bool UseBag=false>
    void initReloc(RelocDataProxy&);

    void publish() const;

    Pose6d get() const { return mOdom2Map; }

    Odometry::Ptr getClosestLocalOdom(double stamp) const;

    OdomDequePtr& getLocal() { return mLocalOdometry; }
    OdomDequePtr& getGlobal() { return mGlobalOdometry; }

    // q should be pointer to iterable container of shared ptr to Odometry
    template<typename T>
    static int getClosestItem(const T& q, double stamp);

    ~Frontend();
    
};

template<typename T>
int Frontend::getClosestItem(const T& q, double stamp)
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

template<typename PointType, bool UseBag>
void Frontend::initLO(LODataProxyPtr<PointType, UseBag>& dp, BackendPtr<PointType>& ed)
{
    // make "this" lvalue or can't find matching constructor
    std::shared_ptr<Frontend> ft = shared_from_this();
    mLO = std::make_unique<LidarOdometry<PointType, UseBag>>(dp, ft, ed);
    mLOthdPtr = std::make_unique<thread::ResidentThread>([&](){
        mLO->generateOdom();
    });
}

template<typename PointType, bool UseBag>
void Frontend::initReloc(RelocDataProxy &rdp)
{
    auto loptr = static_cast<LidarOdometry<PointType, UseBag>*>(mLO.get());
    rdp.registerFunc(std::bind(&LidarOdometry<PointType, UseBag>::setRelocFlag, loptr, std::placeholders::_1));
}

} // namespace frontend

