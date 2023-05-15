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
    enum class Event{
        None,
        NewKFCome,
        LC
    };

    int mKFNums;
    kfs_t keyframes;
    std::mutex mLockKF;
    std::condition_variable mKFcv;
    std::set<index_t> mSubmapIdx;
    std::vector<index_t> mClosestKfIdx;
    Event mKFEvent;

    KeyFramesObj() : mKFNums(0), mKFEvent(Event::None) {}

    bool isSubmapEmpty() {
        std::lock_guard<std::mutex> lk(mLockKF);
        return mSubmapIdx.empty();
    }

    // should be locked in advance
    void newKFIsComing() {
        mKFEvent = Event::NewKFCome;
        mKFcv.notify_one();
    }

    void LCIsHappening() {
        mKFEvent = Event::LC;
        mKFcv.notify_one();
    }

    bool isEventComing() { return mKFEvent != Event::None; }
    Event getEvent() { return mKFEvent; }
    void resetEvent() { mKFEvent = Event::None; }
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
    std::condition_variable mSubmapCv;
    std::atomic_bool mSetUpdateMap;

    trd::AtomicVar<pose_t> mCurPose;
    pose_t mLastPose;

    LidarDataProxyPtr mLidarDataProxyPtr;

    bool isMapping{true};

    std::string mSaveMapDir;
    float mGridSize;

    std::unique_ptr<trd::ResidentThread> mUpdateMapThreadPtr;

    // private
    void updateMap();

public:

    MapManager();
    MapManager(std::string pcd_file);

    void registerVis(const LidarDataProxyPtr& ldp) { mLidarDataProxyPtr = ldp; }

    void setCurPose(const pose_t& p);

    auto getSubmap() { return mSubmap; }    // not thread-safe!!
    std::mutex& getSubmapLock() { return mLockMap; }

    auto getKeyFrameObjPtr() { return mKFObjPtr; }

    void putKeyFrame(const KeyFrame&);

    void saveKfs();

    void initSubmapFromPCD(std::string pcd_file);

    void notifyUpdateMap();

    ~MapManager();
};

}
