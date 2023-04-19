#include <frontend/MapManager.hpp>
#include <nanoflann/kfs_adaptor.hpp>

#include <pcl/io/pcd_io.h>
#include <pcp/pcp.hpp>

namespace frontend {

MapManager::MapManager() : mKFObjPtr(std::make_shared<KeyFramesObj>()),
    mSubmap(pcl::make_shared<pc_t>())
{
    lg = logger::Logger::getInstance();

    pose_t p;
    p.setIdentity();
    mCurPose.store(p);

    lg->info("construct MapManager!!");
    mapHandlerThd = std::make_unique<trd::ResidentThread>(&MapManager::updateMap, this);
}

MapManager::MapManager(std::string pcd_file) : MapManager()
{
    // load global map mode
    if(pcl::io::loadPCDFile<pt_t>(pcd_file, *mSubmap) == -1)
    {
        auto msg = fmt::format("can't load globalmap from: {}", pcd_file);
        lg->error(msg);
        throw std::runtime_error(msg);
    }

    lg->info("load map success!!");

    pcp::voxelDownSample<pt_t>(mSubmap, 0.7f);
    lg->info("submap size: {}", mSubmap->size());
}

// carefully check kf
void MapManager::putKeyFrame(const KeyFrame& kf)
{
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

void MapManager::updateMap()
{
    std::unique_lock<std::mutex> lk(mLockMap);
    mMapCv.wait(lk, [=](){ return (mReadyUpdateMap.load() || lg->isProgramExit()); });
    mReadyUpdateMap.store(false);

    if(lg->isProgramExit()){
        lg->info("program is exiting, give up this update map!");
        return;
    }

    mLockMap.unlock();
    // it is said that nano-kdtree is thread-safe
    auto& keyframes = mKFObjPtr->keyframes;
    std::vector<index_t> k_indices;
    std::vector<scalar_t> k_sqr_distances;
    nanoflann::KeyFramesKdtree<kfs_t, scalar_t, 3> kf_kdtree(keyframes);
    kf_kdtree.radiusSearch(mCurPose.load().translation(), mSurroundingKeyframeSearchRadius, k_indices, k_sqr_distances);

    mLockMap.lock();
    std::lock_guard<std::mutex> kf_lk(mKFObjPtr->mLockKF);
    mKFObjPtr->mSubmapIdx.clear();
    for(auto i : k_indices){
        const auto& src = keyframes[i].pc;
        *mSubmap += *pcp::transformPointCloud<pt_t, scalar_t>(src, keyframes[i].pose);
        mKFObjPtr->mSubmapIdx.insert(i);
    }
    pcp::voxelDownSample<pt_t>(mSubmap, 0.7f);
}

void MapManager::setReadyUpdateMap()
{
    mReadyUpdateMap.store(true); 
    mMapCv.notify_one();
}

MapManager::~MapManager()
{
    lg->info("exit MapManager!");
    mMapCv.notify_all();
}

}