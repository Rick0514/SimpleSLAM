#pragma once

#include <deque>

#include <utils/Atomic.hpp>
#include <utils/Logger.hpp>
#include <utils/Thread.hpp>
#include <types/basic.hpp>

namespace frontend { 
    class Frontend;
    class MapManager;
    class KeyFramesObj;
}

namespace backend
{

using namespace utils;

class LoopClosureManager;

class Backend
{
private:
    using kf_t = KeyFrame;
    using kfs_t = std::deque<kf_t>;
    using KFObjPtr = std::shared_ptr<frontend::KeyFramesObj>;
    using MapManagerPtr = std::shared_ptr<frontend::MapManager>;
    using FrontendPtr = std::shared_ptr<frontend::Frontend>;
    using LCManagerPtr = std::shared_ptr<LoopClosureManager>;

    std::shared_ptr<logger::Logger> lg;

    // frontend
    FrontendPtr mFrontendPtr;
    MapManagerPtr mMapManagerPtr;
    KFObjPtr mKFObjPtr;
    LCManagerPtr mLCManagerPtr;

    // gtsam stuff
    class Gtsam;
    std::unique_ptr<Gtsam> mGtsamImpl;

    // optimize thread
    std::atomic_bool mRunning;
    std::unique_ptr<trd::ResidentThread> mOptimThread;

    std::string mSaveMapDir;

protected:

    void addOdomFactor();
    void addLoopFactor();

public:

    Backend() = delete;
    explicit Backend(const FrontendPtr&, const MapManagerPtr&, LCManagerPtr lcm=LCManagerPtr(nullptr));

    void optimHandler();

    ~Backend();
};
    
} // namespace backend
