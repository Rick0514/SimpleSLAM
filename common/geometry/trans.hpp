#pragma once

#include <Eigen/src/Geometry/Quaternion.h>
#include <types/EigenTypes.hpp>

namespace geometry {
namespace trans {

using namespace EigenTypes;

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> q2rpy(const Eigen::Quaternion<Scalar>& q)
{
    return q.toRotationMatrix().eulerAngles(0, 1, 2);  
}

}
}


