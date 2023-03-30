#pragma once

#include <types/EigenTypes.hpp>

namespace geometry {
namespace trans {

using namespace EigenTypes;

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> q2ypr(const Eigen::Quaternion<Scalar>& q)
{
    return q.toRotationMatrix().eulerAngles(2, 1, 0);  
}

}
}


