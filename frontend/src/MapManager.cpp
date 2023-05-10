#include <dataproxy/LidarDataProxy.hpp>
#include <frontend/MapManager.hpp>
#include <nanoflann/kfs_adaptor.hpp>

#include <pcl/io/pcd_io.h>
#include <pcp/pcp.hpp>

#include <config/params.hpp>
#include <utils/File.hpp>

#include <time/tictoc.hpp>

namespace frontend {

MapManager::MapManager(LidarDataProxyPtr ldp) : mKFObjPtr(std::make_shared<KeyFramesObj>()),
    mSubmap(pcl::make_shared<pc_t>()), mLidarDataProxyPtr(ldp)
{
    lg = logger::Logger::getInstance();

    auto cfg = config::Params::getInstance();
    mSaveMapDir = cfg["saveMapDir"];
    mGridSize = cfg["downSampleVoxelGridSize"].get<float>();

    // important!! init pose decide what submap is like!!
    pose_t p;
    p.setIdentity();
    mLastPose = p;
    mCurPose.store(p);

    mUpdateMapThreadPtr = std::make_unique<trd::ResidentThread>(&MapManager::updateMap, this);

    // load from tum
    auto t2p = utils::file::loadFromTum<scalar_t>(mSaveMapDir);
    if(t2p.empty()){
        lg->warn("tum file maybe empty or not even exist!!");
        return;
    }

    for(int i=0; i<t2p.size(); i++){
        kf_t kf;
        kf.pose = t2p[i].second;
        auto fn = fmt::format("{}/{}.pcd", mSaveMapDir, i);
        pcp::loadPCDFile<pt_t>(fn, kf.pc);
        kf.pc->header.stamp = (size_t)(t2p[i].first * 1e6); // add time stamp
        pcp::voxelDownSample<pt_t>(kf.pc, mGridSize);
        mKFObjPtr->keyframes.emplace_back(kf);
    }
    mKFObjPtr->mKFNums = mKFObjPtr->keyframes.size();

    notifyUpdateMap();
}

// deprecated
MapManager::MapManager(std::string pcd_file)
{
    lg = logger::Logger::getInstance();

    auto cfg = config::Params::getInstance();
    mGridSize = cfg["downSampleVoxelGridSize"].get<float>();

    // important!! init pose decide what submap is like!!
    pose_t p;
    p.setIdentity();
    mLastPose = p;
    mCurPose.store(p);

    isMapping = false;
    mSubmap = pcl::make_shared<pc_t>();

    mKFObjPtr = std::make_shared<KeyFramesObj>();
    mKFObjPtr->mSubmapIdx.insert(0);
    // load global map mode
    if(pcl::io::loadPCDFile<pt_t>(pcd_file, *mSubmap) == -1)
    {
        auto msg = fmt::format("can't load globalmap from: {}", pcd_file);
        lg->error(msg);
        throw std::runtime_error(msg);
    }

    lg->info("load map success!!");

    common::time::tictoc tt;
    pcp::voxelDownSample<pt_t>(mSubmap, mGridSize);
    // pcp::VoxelDownSampleV3 vds(mGridSize);
    // vds.filter<pt_t>(mSubmap, mSubmap);

    lg->info("submap ds cost: {:.3f}", tt);
    lg->info("submap size: {}", mSubmap->size());
    // lg->info("submap mm: {}", vds.getMaxMin());
}

void MapManager::setCurPose(const pose_t &p)
{
    mCurPose.store(p);

    V3<scalar_t> ot = mLastPose.translation();
    V3<scalar_t> ct = p.translation();
    if((ot - ct).norm() > minKFGap){
        mLastPose = p;
        notifyUpdateMap();
    }
}

// carefully check kf
void MapManager::putKeyFrame(const KeyFrame& kf)
{
    if(!isMapping)  return;

    auto& keyframes = mKFObjPtr->keyframes;

    // no any kfs， it must be at first
    if(keyframes.empty()){
        lg->warn("no any keyframes, start mapping at the very first time!!");
        std::lock_guard<std::mutex> lk(mKFObjPtr->mLockKF);
        keyframes.emplace_back(std::move(kf));
        mKFObjPtr->newKFIsComing();
        return;
    }

    std::vector<index_t> k_indices;
    std::vector<scalar_t> k_sqr_distances;
    nanoflann::KeyFramesKdtree<kfs_t, scalar_t, 3> kf_kdtree(keyframes);
    kf_kdtree.nearestKSearch(kf.pose.translation(), 1, k_indices, k_sqr_distances);

    // if keyframes size greater than some gap then notify
    if(k_sqr_distances[0] > minKFGap){
        std::lock_guard<std::mutex> lk(mKFObjPtr->mLockKF);
        keyframes.emplace_back(std::move(kf));
        mKFObjPtr->mClosestKfIdx.emplace_back(k_indices[0]);
        mKFObjPtr->newKFIsComing();
    }
}

void MapManager::updateMap()
{
    std::unique_lock<std::mutex> lk(mLockMap);
    mSubmapCv.wait(lk, [&](){ return mSetUpdateMap.load() || lg->isProgramExit(); });
    mSetUpdateMap.store(false);

    lk.unlock();

    if(lg->isProgramExit()){ 
        lg->warn("program exit, give up update map!"); 
        return;
    }

    lg->info("update map...");

    auto& keyframes = mKFObjPtr->keyframes;
    if(keyframes.empty()){
        lg->warn("no any keyframes to update!!");
        return;
    }

    std::vector<index_t> k_indices;
    std::vector<scalar_t> k_sqr_distances;    
    
    std::unique_lock<std::mutex> kf_lock(mKFObjPtr->mLockKF);
    nanoflann::KeyFramesKdtree<kfs_t, scalar_t, 3> kf_kdtree(keyframes);
    kf_kdtree.radiusSearch(mCurPose.load().translation(), mSurroundingKeyframeSearchRadius, k_indices, k_sqr_distances);

    lk.lock();
    mKFObjPtr->mSubmapIdx.clear();
    mSubmap->clear();
    for(auto i : k_indices){
        const auto& src = keyframes[i].pc;
        pc_t pc;
        pcp::transformPointCloud<pt_t, scalar_t>(src, pc, keyframes[i].pose);
        *mSubmap += pc;
        mKFObjPtr->mSubmapIdx.insert(i);
    }
    
    kf_lock.unlock();

    pcp::voxelDownSample<pt_t>(mSubmap, mGridSize);

    lg->info("kf size: {}", keyframes.size());
    lg->info("kf dist: {}", k_sqr_distances);
    lg->info("submap idx: {}", k_indices);
    lg->info("submap pts: {}", mSubmap->size());

    // vis
    if(mLidarDataProxyPtr)    mLidarDataProxyPtr->setVisGlobalMap(mSubmap);
}

void MapManager::saveKfs()
{
    int n = mKFObjPtr->mKFNums; // last nums
    auto& kfs = mKFObjPtr->keyframes;
    for(int i=n; i<kfs.size(); i++){
        auto fn = fmt::format("{}/{}.pcd", mSaveMapDir, i);
        pcp::savePCDFile<pt_t>(fn, kfs[i].pc);
        // now you can downsample the pc
        pcp::voxelDownSample<pt_t>(kfs[i].pc, mGridSize);
    }
}

void MapManager::notifyUpdateMap()
{
    mSetUpdateMap.store(true);
    mSubmapCv.notify_one();
}

MapManager::~MapManager()
{
    lg->info("exit MapManager!");
    if(mUpdateMapThreadPtr) mUpdateMapThreadPtr->Stop();
    notifyUpdateMap();
}

}