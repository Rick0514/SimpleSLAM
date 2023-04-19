#pragma once

#include <deque>
#include <set>

#include <utils/Atomic.hpp>
#include <utils/Thread.hpp>
#include <utils/Logger.hpp>
#include <types/basic.hpp>

namespace frontend {

using namespace utils;
using kf_t = KeyFrame;
using kfs_t = std::deque<kf_t>;

struct KeyFramesObj
{
    kfs_t keyframes;
    int mKFNums;
    std::mutex mLockKF;
    std::condition_variable mKFcv;
    std::set<index_t> mKFidx;
    KeyFramesObj() : mKFNums(0) {}
};

class MapManager
{
private:

    using KeyFramesObjPtr = std::shared_ptr<KeyFramesObj>;

    std::shared_ptr<logger::Logger> lg;    

    static constexpr float mSurroundingKeyframeSearchRadius{20.0f};

    KeyFramesObjPtr mKFObjPtr;
    
    pc_t::Ptr mSubmap;
    std::mutex mLockMap;
    std::atomic_bool mReadyUpdateMap;
    std::condition_variable mMapCv;

    trd::AtomicVar<pose_t> mCurPose;

    std::unique_ptr<utils::trd::ResidentThread> mapHandlerThd;

public:

    MapManager();
    MapManager(std::string pcd_file);

    void setCurPose(const pose_t& p) { mCurPose.store(p); }

    auto getSubmap() { return mSubmap; }    // not thread-safe!!
    std::mutex& getSubmapLock() { return mLockMap; }

    auto getKeyFrameObjPtr() { return mKFObjPtr; }

    void putKeyFrame(const KeyFrame&);

    void updateMap();

    void initSubmapFromPCD(std::string pcd_file);

    void setReadyUpdateMap() { mReadyUpdateMap.store(true); }

    ~MapManager();
};

}
