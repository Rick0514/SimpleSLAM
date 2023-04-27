#pragma once

#include <deque>
#include <set>

#include <utils/Atomic.hpp>
#include <utils/Thread.hpp>
#include <utils/Logger.hpp>
#include <types/basic.hpp>

namespace dataproxy { class LidarDataProxy; }

namespace frontend {

using namespace utils;
using kf_t = KeyFrame;
using kfs_t = std::deque<kf_t>;

struct KeyFramesObj
{
    int mKFNums;
    kfs_t keyframes;
    std::mutex mLockKF;
    std::condition_variable mKFcv;
    std::set<index_t> mSubmapIdx;
    std::vector<index_t> mClosestKfIdx;

    KeyFramesObj() : mKFNums(0) {}

    bool isSubmapEmpty() {
        std::lock_guard<std::mutex> lk(mLockKF);
        return mSubmapIdx.empty();
    }
};

class MapManager
{
private:

    using KeyFramesObjPtr = std::shared_ptr<KeyFramesObj>;
    using LidarDataProxyPtr = std::shared_ptr<dataproxy::LidarDataProxy>;

    std::shared_ptr<logger::Logger> lg;

    static constexpr float minKFGap{1.0}; 
    static constexpr float mSurroundingKeyframeSearchRadius{8.0f};

    KeyFramesObjPtr mKFObjPtr;
    
    pc_t::Ptr mSubmap;
    std::mutex mLockMap;

    trd::AtomicVar<pose_t> mCurPose;

    LidarDataProxyPtr mLidarDataProxyPtr;

    bool isMapping{true};

    std::string mSaveMapDir;
    float mGridSize;

public:

    MapManager() = delete;
    MapManager(LidarDataProxyPtr ldp=LidarDataProxyPtr());
    MapManager(std::string pcd_file);

    void setCurPose(const pose_t& p) { mCurPose.store(p); }

    auto getSubmap() { return mSubmap; }    // not thread-safe!!
    std::mutex& getSubmapLock() { return mLockMap; }

    auto getKeyFrameObjPtr() { return mKFObjPtr; }

    void putKeyFrame(const KeyFrame&);

    void updateMap();
    void saveKfs();

    void initSubmapFromPCD(std::string pcd_file);

    ~MapManager();
};

}
