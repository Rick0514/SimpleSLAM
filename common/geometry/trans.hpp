#pragma once

#include <types/EigenTypes.hpp>

namespace geometry {
namespace trans {

using namespace EigenTypes;

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

}
}


