#pragma once

#include <types/EigenTypes.hpp>

namespace geometry {
namespace trans {

using namespace EigenTypes;

template <typename T>
inline T rad2deg(T rad)
{
    return rad * 180.0 / M_PI;
}

template <typename T>
inline T deg2rad(T deg)
{
    return deg * M_PI / 180.0;
}

// below are rotate by static axis implementation!!
template<typename Scalar>
inline V3<Scalar> q2ypr(const Eigen::Quaternion<Scalar>& q)
{
    M3<Scalar> rot = q.toRotationMatrix();
    V3<Scalar> ypr;
    if(std::abs(rot(2, 0)) >= 1.0){
        // singularity
        ypr(0) = 0;
        ypr(1) = std::copysign(M_PI/2, -rot(2, 0));
        ypr(2) = std::atan2(rot(0, 1), rot(0, 2));
    }else{
        // output the first solution, actually 2 solutions exist!!
        ypr(0) = std::atan2(rot(1, 0), rot(0, 0));
        ypr(1) = -std::asin(rot(2, 0));
        ypr(2) = std::atan2(rot(2, 1), rot(2, 2));
    }

    return ypr;
}

template<typename Scalar>
inline Qt<Scalar> ypr2q(const V3<Scalar>& ypr)
{
    Qt<Scalar> q = Eigen::AngleAxis<Scalar>(ypr(0), V3<Scalar>::UnitZ()) *
                   Eigen::AngleAxis<Scalar>(ypr(1), V3<Scalar>::UnitY()) *
                   Eigen::AngleAxis<Scalar>(ypr(2), V3<Scalar>::UnitX());
    return q;
}

// quaternion q = [qx, qy, qz, qw](Eigen convention)
template<typename Scalar>
inline Qt<Scalar> rot2q(const Eigen::Ref<const M3<Scalar>> R)
{
    Qt<Scalar> q(R);
    return q.normalized();
}

template<typename Scalar>
inline void T2SE3(M4<Scalar>& T)
{
    M3<Scalar> SO3 = T.template topLeftCorner<3, 3>();
    T.template topLeftCorner<3, 3>() = rot2q<Scalar>(SO3).toRotationMatrix();
}

template<typename Scalar>
inline Pose6<Scalar> SixDof2Mobile(const Pose6<Scalar>& p)
{
    // simply get yaw and x, y, recal the pose
    Eigen::AngleAxis<Scalar> r(p.rotation());
    V3<Scalar> t = p.translation();

    Pose6<Scalar> res;
    res.setIdentity();
    t(2) = 0.0;
    res.translate(t);
    
    Scalar tmp = r.axis().dot(V3<Scalar>::UnitZ()); 
    if(std::abs(tmp) > 0.95){
        Eigen::AngleAxis<Scalar> axis(r.angle(), std::copysign(1.0, tmp) * V3<Scalar>::UnitZ());
        res.rotate(axis.matrix());
    }

    return res;
}

}
}


