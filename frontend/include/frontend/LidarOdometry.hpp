#pragma once
#include <deque>
#include <atomic>
#include <frontend/OdometryBase.hpp>

#include <types/PCLTypes.hpp>
#include <types/EigenTypes.hpp>

// ------------ forward declaration ------------
namespace dataproxy { 
    template <typename T, bool> class DataProxy; 
    class RelocDataProxy;
}

namespace PCR { template<typename PointType> class PointCloudRegister; }
// ------------ forward declaration ------------

namespace frontend
{

using namespace dataproxy;
using namespace PCLTypes;

class Frontend;

template <typename PointType, bool UseBag=false>
class LidarOdometry : public OdometryBase
{

public:
    using KF = KeyFrame<PointType>;
    using DataProxyPtr = std::shared_ptr<DataProxy<PC<PointType>, UseBag>>;
    using FrontendPtr = std::shared_ptr<Frontend>;
    using RelocDataProxyPtr = std::shared_ptr<RelocDataProxy>;

private:

    DataProxyPtr mDataProxyPtr;
    FrontendPtr mFrontendPtr;

    std::unique_ptr<PCR::PointCloudRegister<PointType>> mPcr; 

    std::atomic_bool reloc;
    std::mutex mRelocLock;
    EigenTypes::Pose6d mRelocPose;

    typename PC<PointType>::Ptr mSubmap;
    std::mutex mLockMap;

    std::deque<KF> keyframes;
    int mKFnums;
    std::mutex mKFlock;
    std::condition_variable mKFcv;

public:

    void setRelocFlag(EigenTypes::Pose6d& p);

    explicit LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, RelocDataProxyPtr& rdp);

    virtual void generateOdom() override;

    auto getSubmap() { return mSubmap; }    // not thread-safe!!
    std::mutex& getSubmapLock() { return mLockMap; }

    void selectKeyFrame(KF&& kf);

    ~LidarOdometry();
};
    
} // namespace frontend

