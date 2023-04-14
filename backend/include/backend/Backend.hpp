#pragma once

#include <utils/Atomic.hpp>
#include <utils/Logger.hpp>
#include <types/PCLTypes.hpp>
#include <pcl/kdtree/kdtree_flann.h>

// add a tmp macro to enable c++11 for tbb, https://github.com/oneapi-src/oneTBB/issues/22
// clang++-12 -v to see what glibcxx version you got
// #define TBB_USE_GLIBCXX_VERSION 70500

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <nanoflann/kfs_adaptor.hpp>

namespace backend
{

using namespace utils;
using namespace PCLTypes;
using namespace EigenTypes;

template <typename PointType>
class Backend
{
private:

    typename pcl::PointCloud<PointType>::Ptr mSubMap;
    typename pcl::KdTreeFLANN<PointType>::Ptr mSubMapKdtree;

    using KF = KeyFrame<PointType>;

    std::shared_ptr<logger::Logger> mLg;

    // factor graph
    std::unique_ptr<gtsam::ISAM2> isam2;
    gtsam::NonlinearFactorGraph factorGraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;

    std::deque<KF> keyframes;
    int mKFnums;
    std::mutex mKFlock;
    std::condition_variable mKFcv;

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

    Backend();

    // pcd mode
    Backend(std::string pcd_file);

    const typename pcl::KdTreeFLANN<PointType>::Ptr& getSubMapKdtree() const;
    const typename pcl::PointCloud<PointType>::Ptr& getSubMap() const;
    
    void optimHandler();

    void putKeyFrame(KF&&);

    void setRTPose(const Pose6d& p) { mRTPose.store(p); }

    ~Backend();
};
    
} // namespace backend



