#include <PCR/VgicpRegister.hpp>
#include <frontend/MapManager.hpp>
#include <backend/ScanContext.hpp>
#include <backend/LoopClosureManager.hpp>

#include <pcp/pcp.hpp>
#include <config/params.hpp>

namespace backend {

LoopClosureManager::LoopClosureManager(const MapManagerPtr& mmp) : mmp_(mmp), lc_size_(0), lcq_(10),
    lc_scan_(pcl::make_shared<pc_t>()), lc_map_(pcl::make_shared<pc_t>())
{
    lg = logger::Logger::getInstance();

    auto cfg = config::Params::getInstance()["backend"]["lc"];
    context_pc_ds_ = cfg["contextDownSampleGridSize"].get<float>();
    history_submap_range_ = cfg["historySubmapRange"].get<int>();
    fitness_score_ = cfg["fitnessThreshold"].get<float>();

    ctb_ = std::make_unique<context::ScanContext>();
    pcr_ = std::make_unique<PCR::VgicpRegister>();    
    pcr_->initForLC();

    lc_thd_ = std::make_unique<trd::ResidentThread>(&LoopClosureManager::lcHandler, this);
}

// not thread-safe, kf should be guard
void LoopClosureManager::addContext()
{
    const auto& kfs = mmp_->getKeyFrameObjPtr();
    for(int i=kfs->mKFNums; i<kfs->keyframes.size(); i++){
        pc_t pc_ds;
        pcp::voxelDownSample<pt_t>(kfs->keyframes[i].pc, pc_ds, context_pc_ds_);
        ctb_->addContext(pc_ds);
    }
    lc_cv_.notify_one();
}

// not thread-safe, kf should be guard
void LoopClosureManager::loopFindNearKeyframes(pc_t::Ptr& nearKeyframes, int key, int searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    const auto& kfs = mmp_->getKeyFrameObjPtr();
    int cloudSize = kfs->keyframes.size();
    for(int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear >= 0 && keyNear < cloudSize)
        {
            const auto& src = kfs->keyframes[keyNear].pc;
            pc_t dst;
            pcp::transformPointCloud(src, dst, kfs->keyframes[keyNear].pose);
            *nearKeyframes += dst;
        }
    }

    if(nearKeyframes->empty())  return;

    // downsample near keyframes
    pcp::voxelDownSample<pt_t>(nearKeyframes, context_pc_ds_);
}

void LoopClosureManager::lcHandler()
{
    std::unique_lock<std::mutex> lk(lc_lock_);
    lc_cv_.wait(lk, [&](){ return lc_size_ < ctb_->size() || lg->isProgramExit(); });

    lg->info("enter lc handler...");

    const auto& kfo = mmp_->getKeyFrameObjPtr();

    for(int i=lc_size_; i<ctb_->size(); i++){
        auto q = ctb_->query(i);

        if(q.first >= 0){
            int curKey = i;
            int oldKey = q.first;

            pose_t old_pose, cur_pose;
            {
                std::lock_guard<std::mutex> lk(kfo->mLockKF);
                old_pose = kfo->keyframes[oldKey].pose;
                cur_pose = kfo->keyframes[curKey].pose;
                loopFindNearKeyframes(lc_scan_, curKey, 0);
                loopFindNearKeyframes(lc_map_, oldKey, history_submap_range_);
            }
        
            // pcr
            bool conv = pcr_->scan2Map(lc_scan_, lc_map_, cur_pose);
            auto fs = pcr_->getFitnessScore();
            lg->debug("{} to {} fitness score: {}", oldKey, curKey, fs);
            if(conv && fs < fitness_score_){
                auto r = std::make_shared<LCResult_t>(oldKey, curKey, old_pose.inverse() * cur_pose);
                lcq_.push_back<true>(std::move(r));
            }
        }
    }

    // it means lc_size_ pc is turned into context !!
    lc_size_ = ctb_->size();

    if(!lcq_.empty()){
        kfo->LCIsHappening();
    }
}

LoopClosureManager::~LoopClosureManager()
{
    lc_cv_.notify_one();
    lc_thd_->Stop();
    lcq_.abort();
}

}