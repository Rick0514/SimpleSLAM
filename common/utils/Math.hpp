#pragma once
#include <cmath>

namespace utils {
namespace math {

template<typename T>
inline T to_rad(const T& a){ return M_PI / 180 * a; }

template<typename T>
inline T to_deg(const T& a){ return 180.0 / M_PI * a; }

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

template<typename T>
inline void correctAngles(T& a, T ref)
{
    while ((a-ref) >  M_PI) a -= 2*M_PI;
    while ((a-ref) < -M_PI) a += 2*M_PI;
}

}
}