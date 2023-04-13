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
using namespace EigenTypes;

template<typename PointType>
class LoamRegister : public PointCloudRegister<PointType>
{

public:
    using typename PointCloudRegister<PointType>::PC_Ptr;
    using typename PointCloudRegister<PointType>::PC_cPtr;
    using M6d = Eigen::Matrix<double, 6, 6>;

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
    M6d degenerateProj;

    nanoflann::PointCloudKdtree<PointType, float> mKdtree;

#ifdef DEBUG_DIR
    std::ofstream debug_file;
#endif

private:

    void _pointAssociateToMap(PointType const * const pi, PointType * const po, const Pose6d& p)
    {
        const M3d& rot = p.rotation();
        const V3d& t = p.translation();
        po->x = rot(0,0) * pi->x + rot(0,1) * pi->y + rot(0,2) * pi->z + t(0);
        po->y = rot(1,0) * pi->x + rot(1,1) * pi->y + rot(1,2) * pi->z + t(1);
        po->z = rot(2,0) * pi->x + rot(2,1) * pi->y + rot(2,2) * pi->z + t(2);
        if constexpr (std::is_same_v<PointType, pcl::PointXYZI>){
            po->intensity = pi->intensity;
        }
    }

    template<unsigned int N>
    bool _extractPlaneCoeffs(const Eigen::Matrix<double, N, 3>& A, V4d& hx);

    bool _extractPlaneMatrix(const PointType& pointInMap, const PC_cPtr& dst, Eigen::Matrix<double, mPlanePtsNum, 3>& A);

    // independent of x
    static inline V3d _J_e_wrt_x(const V4d& coff){
        // cn is larger than 1 absolutely
        auto cn = coff.norm();
        return coff.head<3>(0) / coff.norm();
    }
    
    static inline double _error(const V4d& coff, const V3d& pInMap){
        return pInMap.homogeneous().dot(coff) / coff.norm();
    }

    void _removeDegeneratePart(const M6d& JtJ, V6d& x);

public:
    LoamRegister();

    virtual bool scan2Map(const PC_cPtr& src, const PC_cPtr& dst, Pose6d& res) override;

};

} // namespace PCR

