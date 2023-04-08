#pragma once

#include <types/EigenTypes.hpp>
#include <types/PCLTypes.hpp>

#include <utils/SafeDeque.hpp>

#include <dataproxy/DataProxy.hpp>
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
using DataProxyPtr = std::shared_ptr<DataProxy<PC<PointType>, UseBag>>;

template<typename PointType>
using BackendPtr = std::shared_ptr<Backend<PointType>>;

class Frontend : std::enable_shared_from_this<Frontend>
{
protected:

    using OdomDequePtr = std::shared_ptr<concurrency::SafeDeque<Odometry>>;

    Pose6d mOdom2Map;

    std::shared_ptr<concurrency::SafeDeque<Odometry>> mLocalOdometry;
    std::shared_ptr<concurrency::SafeDeque<Odometry>> mGlobalOdometry;

    std::unique_ptr<OdometryBase> mLO;
    std::unique_ptr<std::thread> mLOthdPtr;

    std::atomic_bool mLORunning;

public:
    Frontend();

    template<typename PointType, bool UseBag=false>
    void initLO(DataProxyPtr<PointType, UseBag>&, BackendPtr<PointType>&);

    void publish() const;

    Pose6d get() const { return mOdom2Map; }

    Odometry::Ptr getClosestLocalOdom(double stamp) const;

    OdomDequePtr& getLocal() { return mLocalOdometry; }
    OdomDequePtr& getGlobal() { return mGlobalOdometry; } 

    // q should be pointer to iterable container of shared ptr to Odometry
    template<typename T>
    static int getClosestItem(const T& q, double stamp);

    void LOHandler();

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
void Frontend::initLO(DataProxyPtr<PointType, UseBag>& dp, BackendPtr<PointType>& ed)
{
    // make "this" lvalue or can't find matching constructor
    auto ft = shared_from_this();
    mLO = std::make_unique<LidarOdometry<PointType, UseBag>>(dp, ft, ed);
    mLOthdPtr = std::make_unique<std::thread>(&Frontend::LOHandler, this);
}

} // namespace frontend

