#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <PCR/PointCloudRegister.hpp>


namespace PCR
{ 
using namespace EigenTypes;

template<typename PointType>
class LoamRegister : public PointCloudRegister<PointType>
{

public:
    using typename PointCloudRegister<PointType>::PC_Ptr;
    using M6d = Eigen::Matrix<double, 6, 6>;
    using PointCloudRegister<PointType>::PointCloudRegister;

private:
    
    static constexpr int mPlanePtsNum{5};
    const float mKdtreeMaxSearchDist{1.0};
    const float mPlaneValidThresh{0.2};
    const float mPointValidThresh{0.1};

    const float mDegenerateThresh{100};

    int iters{20};

    bool isDegenerate;
    bool degenerateProjSet;
    M6d degenerateProj;
    pcl::shared_ptr<pcl::KdTreeFLANN<PointType>> mKdtree;

private:

    static inline void _pointAssociateToMap(PointType const * const pi, PointType * const po, const Pose6d& p)
    {
        const M3d& rot = p.rotation();
        const V3d& t = p.translation();
        po->x = rot(0,0) * pi->x + rot(0,1) * pi->y + rot(0,2) * pi->z + t(0);
        po->y = rot(1,0) * pi->x + rot(1,1) * pi->y + rot(1,2) * pi->z + t(1);
        po->z = rot(2,0) * pi->x + rot(2,1) * pi->y + rot(2,2) * pi->z + t(2);
        po->intensity = pi->intensity;
    }

    template<unsigned int N>
    bool _extractPlaneCoeffs(const Eigen::Matrix<double, N, 3>& A, V4d& hx);

    bool _extractPlaneMatrix(const V3d& pointInMap, PC_Ptr dst, Eigen::Matrix<double, mPlanePtsNum, 3>& A);

    // independ of x
    static inline V3d _J_e_wrt_x(const V4d& coff){
        auto cn = coff.norm();
        if(cn < 1e-6)   return V3d::Zero();
        return coff.head<3>(0) / coff.norm();
    }
    
    static inline double _error(const V4d& coff, const V3d& pInMap){
        return pInMap.homogeneous().dot(coff) / coff.norm();
    }

    void _removeDegeneratePart(const M6d& JtJ, V6d& x);

public:
    LoamRegister();
    virtual bool scan2Map(PC_Ptr src, PC_Ptr dst, Pose6d& res) override;

};

} // namespace PCR

