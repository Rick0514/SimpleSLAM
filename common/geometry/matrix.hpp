#pragma once

#include <types/EigenTypes.hpp>

namespace geometry
{
namespace matrix
{
    
using namespace EigenTypes;

template<typename Scalar>
inline void skew(const V3<Scalar>& t, M3<Scalar>& t_hat)
{
    t_hat << 0, -t(2), t(1),
            t(2), 0, -t(0),
            -t(1), t(0), 0;
}
 
} // namespace matrix
}
