#pragma once

#include <PCR/PointCloudRegister.hpp>
#include <nanoflann/pcl_adaptor.hpp>

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
    const float mRotConverge{1e-2};

    int iters{5};

    bool isDegenerate;
    bool degenerateProjSet;
    M6 degenerateProj;

    nanoflann::PointCloudKdtree<pt_t, scalar_t> mKdtree;

#ifdef DEBUG_DIR
    std::ofstream debug_file;
#endif

private:

    static inline void _pointAssociateToMap(pt_t const * const pi, pt_t * const po, const pose_t& p)
    {
        const M3& rot = p.rotation();
        const V3& t = p.translation();
        po->x = rot(0,0) * pi->x + rot(0,1) * pi->y + rot(0,2) * pi->z + t(0);
        po->y = rot(1,0) * pi->x + rot(1,1) * pi->y + rot(1,2) * pi->z + t(1);
        po->z = rot(2,0) * pi->x + rot(2,1) * pi->y + rot(2,2) * pi->z + t(2);
        // c++17
        if constexpr (std::is_same_v<pt_t, Pxyzi>){
            po->intensity = pi->intensity;
        }
    }

    template<unsigned int N>
    bool _extractPlaneCoeffs(const Eigen::Matrix<scalar_t, N, 3>& A, V4& hx);

    bool _extractPlaneMatrix(const pt_t& pointInMap, const PC_cPtr& dst, Eigen::Matrix<scalar_t, mPlanePtsNum, 3>& A);

    // independent of x
    static inline V3 _J_e_wrt_x(const V4& coff){
        // cn is larger than 1 absolutely
        scalar_t cn = coff.norm();
        V3 v = coff.head<3>();
        return v / coff.norm();
    }
    
    static inline scalar_t _error(const V4& coff, const V3& pInMap){
        return pInMap.homogeneous().dot(coff) / coff.norm();
    }

    void _removeDegeneratePart(const M6& JtJ, V6& x);

public:
    LoamRegister();

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, pose_t& res) override;

};

} // namespace PCR

