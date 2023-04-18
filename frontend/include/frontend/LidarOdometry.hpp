#pragma once
#include <deque>
#include <atomic>
#include <types/basic.hpp>
#include <frontend/OdometryBase.hpp>

// ------------ forward declaration ------------
namespace dataproxy { 
    class LidarDataProxy; 
    class RelocDataProxy;
}

namespace PCR { class PointCloudRegister; }
// ------------ forward declaration ------------

namespace frontend
{

using namespace dataproxy;
using namespace PCLTypes;

class Frontend;

class LidarOdometry : public OdometryBase
{

public:
    using KF = KeyFrame;
    using DataProxyPtr = std::shared_ptr<LidarDataProxy>;
    using FrontendPtr = std::shared_ptr<Frontend>;
    using RelocDataProxyPtr = std::shared_ptr<RelocDataProxy>;

private:

    DataProxyPtr mDataProxyPtr;
    FrontendPtr mFrontendPtr;

    std::unique_ptr<PCR::PointCloudRegister> mPcr; 

    std::atomic_bool reloc;
    std::mutex mRelocLock;
    pose_t mRelocPose;

    pc_t::Ptr mSubmap;
    std::mutex mLockMap;

    // std::deque<KF> keyframes;
    // int mKFnums;
    // std::mutex mKFlock;
    // std::condition_variable mKFcv;

public:

    void setRelocFlag(const pose_t& p);

    explicit LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, RelocDataProxyPtr& rdp);

    virtual void generateOdom() override;

    void initSubmapFromPCD(std::string pcd_file);
    auto getSubmap() { return mSubmap; }    // not thread-safe!!
    std::mutex& getSubmapLock() { return mLockMap; }

    void selectKeyFrame(KF&& kf);

    ~LidarOdometry();
};
    
} // namespace frontend

