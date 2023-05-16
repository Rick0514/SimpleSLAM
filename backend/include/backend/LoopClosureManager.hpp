#pragma once
#include <queue>

#include <types/basic.hpp>
#include <utils/Thread.hpp>
#include <utils/Logger.hpp>
#include <utils/SafeDeque.hpp>

namespace PCR { class VgicpRegister; }
namespace frontend { class MapManager; }

namespace backend {

using namespace utils;
namespace context { class ContextBase; }

class LoopClosureManager
{

protected:

    using MapManagerPtr = std::shared_ptr<frontend::MapManager>;

    struct LCResult{
        int from, to;
        pose_t between;  
        LCResult(int f, int t, const pose_t& p) : from(f), to(t), between(p) {}
    };
    using LCResult_t = struct LCResult;
    using LCQ_t = concurrency::SafeDeque<LCResult_t>;

    float context_pc_ds_;   
    int history_submap_range_; 
    float fitness_score_;

    std::unique_ptr<context::ContextBase> ctb_;
    std::unique_ptr<PCR::VgicpRegister> pcr_;
    MapManagerPtr mmp_;

    std::mutex lc_lock_;
    std::condition_variable lc_cv_;
    size_t lc_size_;
    std::unique_ptr<trd::ResidentThread> lc_thd_;

    pc_t::Ptr lc_scan_;
    pc_t::Ptr lc_map_;

    std::shared_ptr<logger::Logger> lg;

    LCQ_t lcq_;

    // for debug
    using visfunc_t = std::function<void(const pc_t::ConstPtr&, const pc_t::ConstPtr&, const pose_t&)>;
    visfunc_t vis_func_;

public:
    
    LoopClosureManager(const MapManagerPtr& mmp);

    void loopFindNearKeyframes(pc_t::Ptr& nearKeyframes, int key, int searchNum);

    void addContext();
    void lcHandler();

    LCQ_t& getLCQ() { return lcq_; }

    void registerVis(const visfunc_t& vf) { vis_func_ = vf; }

    ~LoopClosureManager();

};

}
