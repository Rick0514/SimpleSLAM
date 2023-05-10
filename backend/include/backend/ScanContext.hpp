#pragma once

#include <types/basic.hpp>

#include <nanoflann/vov_adaptor.h>
#include <backend/ContextBase.hpp>

namespace backend {

// adapt from https://github.com/irapkaist/scancontext/blob/master/cpp/module/Scancontext/Scancontext.h
class ScanContext : public ContextBase<pc_t, Eigen::MatrixXd>
{

private:
    static constexpr int PC_NUM_RING = 20;      // 20 in the original paper (IROS 18)

public:
    using SourceType = pc_t;
    using Context = Eigen::MatrixXd;
    using KeyMat = std::vector<std::vector<double>>;
    using InvKeyTree = nanoflann::VectorOfVectorsKdTree<KeyMat, double, PC_NUM_RING>;

protected:
    const float LIDAR_HEIGHT = 2.0;    // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    
    const int PC_NUM_SECTOR = 60;    // 60 in the original paper (IROS 18)
    const float PC_MAX_RADIUS = 80.0f;  // 80 meter max in the original paper (IROS 18)
    const float PC_UNIT_SECTORANGLE = 360.0f / float(PC_NUM_SECTOR);
    const float PC_UNIT_RINGGAP = PC_MAX_RADIUS / float(PC_NUM_RING);

    // tree
    const int NUM_EXCLUDE_RECENT = 50; // simply just keyframe gap, but node position distance-based exclusion is ok. 
    const int NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    const float SEARCH_RATIO = 0.1f; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    const float SC_DIST_THRES = 0.13f; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    // const double SC_DIST_THRES = 0.5; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config 
    const int    TREE_MAKING_PERIOD_ = 50; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
    int          tree_making_period_conter = 0;

    // data 
    std::vector<Context> polarcontexts_;
    std::vector<Context> polarcontext_invkeys_;
    std::vector<Context> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    // ------------- func -------------
    static float xy2theta(const float & _x, const float & _y);
    static Context circshift(const Context &_mat, int _num_shift);
    static std::vector<double> eig2stdvec(const Context& _eigmat);

    int fastAlignUsingVkey(Context & _vkey1, Context & _vkey2); 
    std::pair<double, int> distanceBtnScanContext (const Context &_sc1, const Context &_sc2); // "D" (eq 6) in the original paper (IROS 18)
    int fastAlignUsingVkey(const Context& _vkey1, const Context& _vkey2);

    Context makeRingkeyFromScancontext(const Context& _desc);
    Context makeSectorkeyFromScancontext(const Context& _desc);

public:

    virtual Context makeContext(const SourceType& input) override;
    virtual double computeSimularity(size_t from, size_t to) override;
    virtual double computeSimularity(const Context& from, const Context& to) override;

};

}