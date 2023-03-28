#pragma once
#include <cmath>

namespace utils {
namespace math {

template<typename T>
inline bool relClose(const T& a, const T& b, double rel_tol=1e-9)
{
    return std::abs(a - b) <= rel_tol * std::max(std::abs(a), std::abs(b));
}

template<typename T>
inline bool absClose(const T& a, const T& b, double abs_tol=0.0)
{
    return std::abs(a - b) <= abs_tol;
}

}
}