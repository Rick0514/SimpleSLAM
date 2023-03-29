#pragma once

#include <types/EigenTypes.hpp>

namespace PCR
{
namespace matrix
{
    
using namespace EigenTypes;

inline void skew(const V3d& t, M3d& t_hat)
{
    t_hat << 0, -t(2), t(1),
            t(2), 0, -t(0),
            -t(1), t(0), 0;
}
 
} // namespace matrix
}
