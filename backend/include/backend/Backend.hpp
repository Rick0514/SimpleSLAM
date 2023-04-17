#pragma once

#include <utils/Atomic.hpp>
#include <utils/Logger.hpp>
#include <types/PCLTypes.hpp>

// add a tmp macro to enable c++11 for tbb, https://github.com/oneapi-src/oneTBB/issues/22
// clang++-12 -v to see what glibcxx version you got
// #define TBB_USE_GLIBCXX_VERSION 70500

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <nanoflann/kfs_adaptor.hpp>

namespace frontend { class Frontend; }

namespace backend
{

using namespace utils;
using namespace PCLTypes;
using namespace EigenTypes;

template <typename PointType>
class Backend
{
private:
    using KF = KeyFrame<PointType>;
    using Scalar = typename KF::Scalar_t;
    using FrontendPtr = std::shared_ptr<frontend::Frontend>;

    static constexpr float mSurroundingKeyframeSearchRadius{20.0};

    std::shared_ptr<logger::Logger> mLg;

    // frontend
    FrontendPtr mFrontendPtr;

    // factor graph
    std::unique_ptr<gtsam::ISAM2> isam2;
    gtsam::NonlinearFactorGraph factorGraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;

    // optimize thread
    std::atomic_bool mRunning;
    std::unique_ptr<std::thread> mOptimThread;

    // noise
    V6d priorNoise, odomNoise;

    // current pose
    trd::AtomicVar<Pose6d> mRTPose;

protected:

    void addOdomFactor();

    // TODO: loop-detect maybe another module
    void addLoopFactor();

public:

    Backend() = delete;
    explicit Backend(const FrontendPtr&);

    void optimHandler();

    void setRTPose(const Pose6d& p) { mRTPose.store(p); }

    ~Backend();
};
    
} // namespace backend



