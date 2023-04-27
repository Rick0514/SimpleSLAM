#include <geometry/trans.hpp>

#include <backend/Backend.hpp>
#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <config/params.hpp>
#include <utils/File.hpp>

namespace backend
{

using namespace gtsam;
using namespace EigenTypes;
using namespace PCLTypes;

Backend::Backend(const FrontendPtr& ft, const MapManagerPtr& mp) : mRunning(true), mFrontendPtr(ft), mMapManagerPtr(mp)
{
    lg = logger::Logger::getInstance();
    lg->info("backend is constructing!!");

    auto cfg = config::Params::getInstance();
    mSaveMapDir = cfg["saveMapDir"];

    priorNoise << 1e-2, 1e-2, M_PI / 72, 1e-1, 1e-1, 1e-1;
    odomNoise << 1e-4, 1e-4, 1e-4, 1e-1, 1e-1, 1e-1;

    // get keyframe obj to optimize
    mKFObjPtr = mMapManagerPtr->getKeyFrameObjPtr();

    gtsam::ISAM2Params param;
    param.relinearizeThreshold = 0.1;
    param.relinearizeSkip = 1;
    isam2 = std::make_unique<gtsam::ISAM2>(param);

    loadFactorGraph();

    mOptimThread = std::make_unique<trd::ResidentThread>(&Backend::optimHandler, this);
}

void Backend::loadFactorGraph()
{
    std::string fn = fmt::format("{}/fg.g2o", mSaveMapDir);
    if(!utils::file::isFileExist(fn)){
        lg->info("no factor graph file, maybe first build or rebuild!!");
        return;
    }
    gtsam::NonlinearFactorGraph::shared_ptr fg;
    gtsam::Values::shared_ptr ie;
    std::tie(fg, ie) = gtsam::readG2o(fn, true);

    factorGraph = *fg;
    recordFactorGraph = *fg;
    recordEstimate = *ie;
    // isam2->update(factorGraph, initialEstimate);
    // isam2->update();
    // factorGraph.resize(0);
    // initialEstimate.clear();

    lg->info("load factor graph done!");
}

void Backend::addOdomFactor()
{
    auto n = mKFObjPtr->mKFNums;
    auto& keyframes = mKFObjPtr->keyframes;

    if(n == 0){
        noiseModel::Diagonal::shared_ptr gtPriorNoise = noiseModel::Diagonal::Variances(priorNoise);
        
        // turn to 2d
        pose_t kfp = keyframes.front().pose;
        // kfp = geometry::trans::SixDof2Mobile(kfp);
        auto pose = gtsam::Pose3(kfp.matrix().cast<double>());
        factorGraph.add(PriorFactor<gtsam::Pose3>(0, pose, gtPriorNoise));
        recordFactorGraph.add(PriorFactor<gtsam::Pose3>(0, pose, gtPriorNoise));
        initialEstimate.insert(0, pose);
        n++;
    }

    noiseModel::Diagonal::shared_ptr gtOdomNoise = noiseModel::Diagonal::Variances(odomNoise);
    int cidx = 0;
    for(int i=n; i<keyframes.size(); i++){
        auto from_idx = mKFObjPtr->mClosestKfIdx[cidx++];
        auto from = gtsam::Pose3(keyframes[from_idx].pose.matrix().cast<double>());
        auto to = gtsam::Pose3(keyframes[i].pose.matrix().cast<double>());
        lg->info("factor graph add edge from {} to {}", from_idx, i);
        factorGraph.add(BetweenFactor<gtsam::Pose3>(from_idx, i, from.between(to), gtOdomNoise));
        recordFactorGraph.add(BetweenFactor<gtsam::Pose3>(from_idx, i, from.between(to), gtOdomNoise));
        // insert pose that project to 2d
        // pose_t kf_to = keyframes[i].pose;
        // kf_to = geometry::trans::SixDof2Mobile(kf_to);
        // to = gtsam::Pose3(kf_to.matrix().cast<double>());
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

    mMapManagerPtr->saveKfs();
    // new kf is put
    addOdomFactor();
    mKFObjPtr->mKFNums = mKFObjPtr->keyframes.size();
    mKFObjPtr->mClosestKfIdx.clear();

    lk.unlock();    // ------------------------------------

    //  maybe time comsuming
    isam2->update(factorGraph, initialEstimate);
    isam2->update();

    factorGraph.resize(0);
    initialEstimate.clear();
    optimizedEstimate = isam2->calculateEstimate();
    recordEstimate = optimizedEstimate;

    lk.lock();     // ------------------------------------  
    
    auto n = mKFObjPtr->mKFNums;
    auto& keyframes = mKFObjPtr->keyframes;
    // save newest kf
    pose_t latest_pose = keyframes.back().pose; 
    // update kfs
    for(int i=0; i<n; i++){
        const auto& p = optimizedEstimate.at<gtsam::Pose3>(i);
        keyframes[i].pose.matrix() = p.matrix().cast<scalar_t>();
    }

    // update frontend, make it se3
    pose_t delta = keyframes.back().pose * latest_pose.inverse();

    lk.unlock();

    {
        std::stringstream ss;
        ss << delta.translation().transpose();
        lg->info("backend delta: {}", ss.str());
    }

    geometry::trans::T2SE3(delta.matrix());

    const auto& gb = mFrontendPtr->getGlobal();

    {
        lg->info("update globalodom queue!!");

        std::lock_guard<std::mutex> _lk(gb->getLock());
        auto gbq = gb->getDequeInThreadUnsafeWay();
        // will pose become non-se3??
        for(auto& e : gbq) e->odom = delta * e->odom;
    }

    // update odom2map
    pose_t p = delta * mFrontendPtr->get().load();
    mFrontendPtr->get().store(p);
    
    // now update map immediately
    mMapManagerPtr->notifyUpdateMap();
}

Backend::~Backend()
{
    mKFObjPtr->mKFcv.notify_all();
    lg->info("exit backend, save kfs and factor graph...");
    mOptimThread->Stop();
    // save tum
    std::lock_guard<std::mutex> lk(mKFObjPtr->mLockKF);
    utils::file::writeAsTum(mSaveMapDir, mKFObjPtr->keyframes);
    gtsam::writeG2o(recordFactorGraph, recordEstimate, fmt::format("{}/fg.g2o", mSaveMapDir));
}

}