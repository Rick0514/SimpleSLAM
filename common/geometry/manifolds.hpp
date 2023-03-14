#pragma once

#include <types/EigenTypes.hpp>
#include <geometry/matrix.hpp>
using namespace EigenTypes;

namespace PCR
{
namespace manifolds
{
    inline void exp(const V3d& w, M3d& SO3)
    {
        double t = w.norm();
        double t2 = t * t;
        M3d omega;
        matrix::skew(w, omega);
        M3d omega2 = omega * omega;
        SO3 = M3d::Identity() + (sin(t) / t) * omega + ((1 - cos(t)) / t2) * omega2;        
    }

    inline void exp(const V6d& k, M4d& SE3)
    {
        V3d p = k.head<3>();
        V3d w = k.tail<3>();

        double t = w.norm();
        double t2 = t * t;
        double t3 = t * t2;
        M3d omega;
        matrix::skew(w, omega);
        M3d omega2 = omega * omega;

        M3d R = M3d::Identity() + (sin(t) / t) * omega + ((1 - cos(t)) / t2) * omega2;
        M3d V = M3d::Identity() + ((1 - cos(t)) / t2) * omega + ((t - sin(t)) / t3) * omega2;

        SE3.block<3, 3>(0, 0) = R;
        SE3.block<3, 1>(0, 3) = V * p;
    }

    inline void J_SE3(const V3d& p, Eigen::Matrix<double, 3, 6>& J){
        J.block<3, 3>(0, 0).setIdentity();
        M3d s;
        matrix::skew(p, s);
        J.block<3, 3>(0, 3) = -s;
    }

} // namespace common

}
