#include <dataproxy/LidarDataProxy.hpp>
#include <frontend/MapManager.hpp>
#include <nanoflann/kfs_adaptor.hpp>

#include <pcl/io/pcd_io.h>
#include <pcp/pcp.hpp>

#include <config/params.hpp>
#include <utils/File.hpp>

namespace frontend {

MapManager::MapManager() : mKFObjPtr(std::make_shared<KeyFramesObj>()),
    mSubmap(pcl::make_shared<pc_t>())
{
    lg = logger::Logger::getInstance();

    auto cfg = config::Params::getInstance();
    mSaveMapDir = cfg["saveMapDir"];
    mGridSize = cfg["downSampleVoxelGridSize"].get<float>();

    // important!! init pose decide what submap is like!!
    pose_t p;
    p.setIdentity();
    mCurPose.store(p);

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
        mKFObjPtr->keyframes.emplace_back(kf);
    }
    updateMap();    // init submap and submapIdx
}

// deprecated
MapManager::MapManager(std::string pcd_file) : MapManager()
{
    isMapping = false;

    mKFObjPtr->mSubmapIdx.insert(0);
    // load global map mode
    if(pcl::io::loadPCDFile<pt_t>(pcd_file, *mSubmap) == -1)
    {
        auto msg = fmt::format("can't load globalmap from: {}", pcd_file);
        lg->error(msg);
        throw std::runtime_error(msg);
    }

    lg->info("load map success!!");

    pcp::voxelDownSample<pt_t>(mSubmap, mGridSize);
    lg->info("submap size: {}", mSubmap->size());
}

// carefully check kf
void MapManager::putKeyFrame(const KeyFrame& kf)
{
    if(!isMapping)  return;

    auto& keyframes = mKFObjPtr->keyframes;

    std::vector<index_t> k_indices;
    std::vector<scalar_t> k_sqr_distances;
    nanoflann::KeyFramesKdtree<kfs_t, scalar_t, 3> kf_kdtree(keyframes);
    if(kf_kdtree.radiusSearch(kf.pose.translation(), minKFGap, k_indices, k_sqr_distances))
        return;     // kf is not distant from other

    std::lock_guard<std::mutex> lk(mKFObjPtr->mLockKF);
    keyframes.emplace_back(std::move(kf));
    // if keyframes size greater than some gap than notify
    mKFObjPtr->mKFcv.notify_one();
}

// keyframe should be locked before invoke
void MapManager::updateMap()
{
    lg->info("update map...");
    // it is said that nano-kdtree is thread-safe
    auto& keyframes = mKFObjPtr->keyframes;
    std::vector<index_t> k_indices;
    std::vector<scalar_t> k_sqr_distances;
    nanoflann::KeyFramesKdtree<kfs_t, scalar_t, 3> kf_kdtree(keyframes);
    kf_kdtree.radiusSearch(mCurPose.load().translation(), mSurroundingKeyframeSearchRadius, k_indices, k_sqr_distances);

    std::lock_guard<std::mutex> mlk(mLockMap);
    mKFObjPtr->mSubmapIdx.clear();
    mSubmap->points.clear();
    for(auto i : k_indices){
        const auto& src = keyframes[i].pc;
        *mSubmap += *pcp::transformPointCloud<pt_t, scalar_t>(src, keyframes[i].pose);
        mKFObjPtr->mSubmapIdx.insert(i);
    }
    pcp::voxelDownSample<pt_t>(mSubmap, mGridSize);

    lg->info("kf size: {}", keyframes.size());
    lg->info("kf dist: {}", k_sqr_distances);
    lg->info("submap idx: {}", k_indices);
    lg->info("submap pts: {}", mSubmap->points.size());

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

MapManager::~MapManager()
{
    lg->info("exit MapManager!");
    // save tum
    std::lock_guard<std::mutex> lk(mKFObjPtr->mLockKF);
    utils::file::writeAsTum(mSaveMapDir, mKFObjPtr->keyframes);
}

}