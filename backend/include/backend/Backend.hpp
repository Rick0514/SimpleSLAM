#pragma once

#include <utils/Atomic.hpp>
#include <utils/Logger.hpp>
#include <utils/Thread.hpp>
#include <types/basic.hpp>

// add a tmp macro to enable c++11 for tbb, https://github.com/oneapi-src/oneTBB/issues/22
// clang++-12 -v to see what glibcxx version you got
// #define TBB_USE_GLIBCXX_VERSION 70500

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace frontend { 
    class Frontend;
    class MapManager;
    class KeyFramesObj;
}

namespace backend
{

using namespace utils;

// class LoopManager;

class Backend
{
private:
    using kf_t = KeyFrame;
    using kfs_t = std::deque<kf_t>;
    using KFObjPtr = std::shared_ptr<frontend::KeyFramesObj>;
    using MapManagerPtr = std::shared_ptr<frontend::MapManager>;
    using FrontendPtr = std::shared_ptr<frontend::Frontend>;
    // using LoopManagerPtr = std::unique_ptr<LoopManager>;

    std::shared_ptr<logger::Logger> lg;

    // frontend
    FrontendPtr mFrontendPtr;
    MapManagerPtr mMapManagerPtr;
    KFObjPtr mKFObjPtr;

    // LoopManagerPtr mLoopManagerPtr;

    // factor graph
    gtsam::noiseModel::Diagonal::shared_ptr gtPriorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr gtOdomNoise;

    std::unique_ptr<gtsam::ISAM2> isam2;
    gtsam::NonlinearFactorGraph factorGraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;

    // for record!!
    gtsam::Values recordEstimate;
    gtsam::NonlinearFactorGraph recordFactorGraph;

    // optimize thread
    std::atomic_bool mRunning;
    std::unique_ptr<trd::ResidentThread> mOptimThread;

    // noise --> gtsam use double
    Eigen::Matrix<double, 6, 1> priorNoise, odomNoise;

    std::string mSaveMapDir;

protected:

    void loadFactorGraph();

    void addOdomFactor();

    // TODO: loop-detect maybe another module
    void addLoopFactor();

    void myReadG2o(const std::string& file, gtsam::NonlinearFactorGraph& fg, gtsam::Values& v);

public:

    Backend() = delete;
    explicit Backend(const FrontendPtr&, const MapManagerPtr&);

    void optimHandler();

    ~Backend();
};
    
} // namespace backend



