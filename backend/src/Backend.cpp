#include <backend/Backend.hpp>

#include <macro/templates.hpp>
#include <pcl/io/pcd_io.h>
#include <pcp/pcp.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace backend
{

using namespace gtsam;
using namespace PCLTypes;

template<typename PointType>
Backend<PointType>::Backend() : mRunning(true), mKFnums(0)
{
    priorNoise << 1e-2, 1e-2, M_PI / 72, 1e-1, 1e-1, 1e-1;
    odomNoise << 1e-4, 1e-4, 1e-4, 1e-1, 1e-1, 1e-1;

    Pose6d p;
    p.setIdentity();
    mRTPose.store(p);

    gtsam::ISAM2Params param;
    param.relinearizeThreshold = 0.1;
    param.relinearizeSkip = 1;
    isam2 = std::make_unique<gtsam::ISAM2>(param);
}


template<typename PointType>
Backend<PointType>::Backend(std::string pcd_file)
{
    mLg = logger::Logger::getInstance();
    // load global map mode
    mSubMap = pcl::make_shared<PC<PointType>>();
    
    if(pcl::io::loadPCDFile<PointType>(pcd_file, *mSubMap) == -1)
    {
        auto msg = fmt::format("can't load globalmap from: {}", pcd_file);
        mLg->error(msg);
        throw std::runtime_error(msg);
    }

    mLg->info("load map success!!");

    // downsample global pc
    pcp::voxelDownSample<PointType>(mSubMap, 0.7f);
    mLg->info("submap size: {}", mSubMap->size());
}

template<typename PointType>
const typename pcl::PointCloud<PointType>::Ptr& Backend<PointType>::getSubMap() const
{
    return mSubMap;
}

template<typename PointType>
void Backend<PointType>::addOdomFactor()
{
    if(mKFnums == 0){
        noiseModel::Diagonal::shared_ptr gtPriorNoise = noiseModel::Diagonal::Variances(priorNoise);
        auto pose = gtsam::Pose3(keyframes.front().pose.matrix());
        factorGraph.add(PriorFactor<gtsam::Pose3>(0, pose, gtPriorNoise));
        initialEstimate.insert(0, pose);
    }

    noiseModel::Diagonal::shared_ptr gtOdomNoise = noiseModel::Diagonal::Variances(odomNoise);
    for(int i=mKFnums+1; i<keyframes.size(); i++){
        auto from = gtsam::Pose3(keyframes[i-1].pose.matrix());
        auto to = gtsam::Pose3(keyframes[i].pose.matrix());
        factorGraph.add(BetweenFactor<gtsam::Pose3>(i-1, i, from.between(to), gtOdomNoise));
        initialEstimate.insert(i, to);
    }
}


template<typename PointType>
void Backend<PointType>::putKeyFrame(KF&& kf)
{
    std::lock_guard<std::mutex> lk(mKFlock);
    keyframes.emplace_back(kf);
    mKFcv.notify_one();
}

template<typename PointType>
void Backend<PointType>::optimHandler()
{
    while(mRunning){

        std::unique_lock<std::mutex> lk(mKFlock);
        mKFcv.wait(lk, [&](){ return keyframes.size() > mKFnums; });
        // new kf is put
        addOdomFactor();
        mKFnums = keyframes.size();

        lk.unlock();    // ------------------------------------

        //  maybe time comsuming
        isam2->update(factorGraph, initialEstimate);
        isam2->update();

        factorGraph.resize(0);
        initialEstimate.clear();
        optimizedEstimate = isam2->calculateEstimate();

        lk.lock();     // ------------------------------------   
        // 1. update kfs
        for(int i=0; i<mKFnums; i++){
            auto p = optimizedEstimate.at<gtsam::Pose3>(i);
            keyframes[i].pose.matrix() = p.matrix();
        }
        // 2. update submap
        
        // 3. info frontend something update, this step maybe tricky if backend dont own reference of frontend!!        
    }    
}

template<typename PointType>
Backend<PointType>::~Backend()
{
    mRunning.store(false);
    if(mOptimThread->joinable())    mOptimThread->join();    
}


PCTemplateInstantiateExplicitly(Backend);
}