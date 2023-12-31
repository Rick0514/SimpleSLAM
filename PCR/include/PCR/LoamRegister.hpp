#pragma once

#include <PCR/PointCloudRegister.hpp>

#if defined(DEBUG_DIR)
#include <fstream>
#define DEBUG(f, msg)   f << msg << std::endl;
#else
#define DEBUG(...) (void)0
#endif                      

namespace PCR
{ 

class LoamRegister : public PointCloudRegister
{

public:
    using typename PointCloudRegister::PC_Ptr;
    using typename PointCloudRegister::PC_cPtr;
    using V3 = EigenTypes::V3<scalar_t>;
    using M3 = EigenTypes::M3<scalar_t>;
    using V4 = Eigen::Matrix<scalar_t, 4, 1>;
    using V6 = Eigen::Matrix<scalar_t, 6, 1>;
    using M4 = Eigen::Matrix<scalar_t, 4, 4>;
    using M6 = Eigen::Matrix<scalar_t, 6, 6>;

private:
    
    static constexpr int mPlanePtsNum{5};
    const float mKdtreeMaxSearchDist{1.0};
    const float mPlaneValidThresh{0.2};
    const float mPointValidThresh{0.1};

    const float mDegenerateThresh{100};

    const float mPosConverge{5e-3};
    const float mRotConverge{5e-3};

    int iters{8};

    bool isDegenerate;
    bool degenerateProjSet;
    M6 degenerateProj;

    struct Kdtree;
    std::unique_ptr<Kdtree> mKdtree;

#ifdef DEBUG_DIR
    std::ofstream debug_file;
#endif

private:

    static void _pointAssociateToMap(pt_t const * const pi, pt_t * const po, const pose_t& p)
    {
        const M3& rot = p.rotation();
        const V3& t = p.translation();
        po->x = rot(0,0) * pi->x + rot(0,1) * pi->y + rot(0,2) * pi->z + t(0);
        po->y = rot(1,0) * pi->x + rot(1,1) * pi->y + rot(1,2) * pi->z + t(1);
        po->z = rot(2,0) * pi->x + rot(2,1) * pi->y + rot(2,2) * pi->z + t(2);
    }

    template<unsigned int N>
    bool _extractPlaneCoeffs(const Eigen::Matrix<scalar_t, N, 3>& A, V3& hx);

    bool _extractPlaneMatrix(const pt_t& pointInMap, const PC_cPtr& dst, Eigen::Matrix<scalar_t, mPlanePtsNum, 3>& A);

    // independent of x
    static V3 _J_e_wrt_x(const V3& coff){
        // cn is larger than 1 absolutely
        return coff / coff.norm();
    }

    static inline scalar_t _dist(const V3& coff, const V3& pInMap){
        return (pInMap.dot(coff) + 1.0) / coff.norm();
    }

    void _removeDegeneratePart(const M6& JtJ, V6& x);

public:
    LoamRegister();

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) override;

    ~LoamRegister();
};

} // namespace PCR

