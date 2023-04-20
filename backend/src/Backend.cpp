#include <geometry/trans.hpp>

#include <backend/Backend.hpp>
#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace backend
{

using namespace gtsam;
using namespace EigenTypes;
using namespace PCLTypes;

Backend::Backend(const FrontendPtr& ft, const MapManagerPtr& mp) : mRunning(true), mFrontendPtr(ft), mMapManagerPtr(mp)
{
    lg = logger::Logger::getInstance();

    lg->info("backend is constructing!!");

    priorNoise << 1e-2, 1e-2, M_PI / 72, 1e-1, 1e-1, 1e-1;
    odomNoise << 1e-4, 1e-4, 1e-4, 1e-1, 1e-1, 1e-1;

    // get keyframe obj to optimize
    mKFObjPtr = mMapManagerPtr->getKeyFrameObjPtr();

    gtsam::ISAM2Params param;
    param.relinearizeThreshold = 0.1;
    param.relinearizeSkip = 1;
    isam2 = std::make_unique<gtsam::ISAM2>(param);

    mOptimThread = std::make_unique<trd::ResidentThread>(&Backend::optimHandler, this);
}

void Backend::addOdomFactor()
{
    auto n = mKFObjPtr->mKFNums;
    auto& keyframes = mKFObjPtr->keyframes;

    if(n == 0){
        noiseModel::Diagonal::shared_ptr gtPriorNoise = noiseModel::Diagonal::Variances(priorNoise);
        auto pose = gtsam::Pose3(keyframes.front().pose.matrix());
        factorGraph.add(PriorFactor<gtsam::Pose3>(0, pose, gtPriorNoise));
        initialEstimate.insert(0, pose);
        n++;
    }

    noiseModel::Diagonal::shared_ptr gtOdomNoise = noiseModel::Diagonal::Variances(odomNoise);
    for(int i=n; i<keyframes.size(); i++){
        auto from = gtsam::Pose3(keyframes[i-1].pose.matrix());
        auto to = gtsam::Pose3(keyframes[i].pose.matrix());
        factorGraph.add(BetweenFactor<gtsam::Pose3>(i-1, i, from.between(to), gtOdomNoise));
        initialEstimate.insert(i, to);
    }
}

void Backend::optimHandler()
{
    std::unique_lock<std::mutex> lk(mKFObjPtr->mLockKF);
    mKFObjPtr->mKFcv.wait(lk, [&](){ return (mKFObjPtr->keyframes.size() > mKFObjPtr->mKFNums || lg->isProgramExit()); });
    lg->info("backend start to optimize!!");

    if(lg->isProgramExit()){
        lg->info("program is about exit, give up this optim!");
        return;
    }

    // new kf is put
    addOdomFactor();
    mKFObjPtr->mKFNums = mKFObjPtr->keyframes.size();

    lk.unlock();    // ------------------------------------

    //  maybe time comsuming
    isam2->update(factorGraph, initialEstimate);
    isam2->update();

    factorGraph.resize(0);
    initialEstimate.clear();
    optimizedEstimate = isam2->calculateEstimate();

    lk.lock();     // ------------------------------------  
    
    auto n = mKFObjPtr->mKFNums;
    auto& keyframes = mKFObjPtr->keyframes;
    // save newest kf
    pose_t latest_pose = keyframes.back().pose; 
    // update kfs
    for(int i=0; i<n; i++){
        const auto& p = optimizedEstimate.at<gtsam::Pose3>(i);
        keyframes[i].pose.matrix() = p.matrix();
    }

    // update frontend, make it se3
    pose_t delta = keyframes.back().pose * latest_pose.inverse();
    geometry::trans::T2SE3(delta.matrix());

    const auto& gb = mFrontendPtr->getGlobal();

    {
        std::lock_guard<std::mutex> _lk(*gb->getLock());
        auto gbq = gb->getDequeInThreadUnsafeWay();
        // will pose become non-se3??
        for(auto& e : *gbq) e->odom = delta * e->odom;
    }

    pose_t p = mFrontendPtr->get().load();
    mFrontendPtr->get().store(p);
    
    // now update map immediately
    mMapManagerPtr->updateMap();
}

Backend::~Backend()
{
    mKFObjPtr->mKFcv.notify_all();
}

}