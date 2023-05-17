#pragma once

#include <types/basic.hpp>

#include <nanoflann/vov_adaptor.h>
#include <backend/ContextBase.hpp>

namespace backend {

namespace context {

// adapt from https://github.com/irapkaist/scancontext/blob/master/cpp/module/Scancontext/Scancontext.h
class ScanContext : public ContextBase
{

private:
    scpr_t int PC_NUM_RING = 20;      // 20 in the original paper (IROS 18)
    scpr_t int PC_NUM_SECTOR = 60;    // 60 in the original paper (IROS 18)
    scpr_t float PC_MAX_RADIUS = 80.0f;  // 80 meter max in the original paper (IROS 18)

    // tree
    int NUM_EXCLUDE_RECENT; // simply just keyframe gap, but node position distance-based exclusion is ok. 
    int BUILD_TREE_GAP;
    int NUM_CANDIDATES_FROM_TREE; // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    float SEARCH_RATIO;     // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    float SC_DIST_THRES;    // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    // const double SC_DIST_THRES = 0.5; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

public:    
    using VContext = Eigen::VectorXd;
    using VcContainer = std::vector<VContext>;
    using RingKdtree = nanoflann::VectorOfVectorsKdTree<VcContainer, double, PC_NUM_RING>;

protected:
    float LIDAR_HEIGHT;    // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    const float PC_UNIT_SECTORANGLE = 360.0f / float(PC_NUM_SECTOR);
    const float PC_UNIT_RINGGAP = PC_MAX_RADIUS / float(PC_NUM_RING);

    // data 
    std::vector<Context> polarcontexts_;
    VcContainer ringcontexts_;
    VcContainer sectorcontexts_;

    VcContainer ring_sub_;
    RingKdtree ring_kdtree_;

    // ------------- func -------------
    static float xy2theta(const float & _x, const float & _y);
    static Context circshift(const Context &_mat, int _num_shift);

    std::pair<double, int> distanceBtnScanContext (int idx1, int idx2); // "D" (eq 6) in the original paper (IROS 18)
    int fastAlignUsingVkey(const Context& _vkey1, const Context& _vkey2);

    Context makeScanContext(const SourceType& input);
    VContext makeRingkeyFromScancontext(const Context& _desc);
    VContext makeSectorkeyFromScancontext(const Context& _desc);

public:

    ScanContext();

    virtual size_t size() const override { return polarcontexts_.size(); }

    virtual void addContext(const SourceType& input) override;
    virtual QueryResult query(int id) override;
    // virtual double computeSimularity(size_t from, size_t to) override;
    virtual double computeSimularity(const Context& from, const Context& to) override;

};

}

}