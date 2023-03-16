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
        if(t < 1e-6){
            SO3 = M3d::Identity();
            return;
        }

        V3d a = w / t;
        double ct = cos(t);

        M3d a_hat;
        matrix::skew(a, a_hat);
        SO3 = ct * M3d::Identity() + (1.0 - ct) * a * a.transpose() + sin(t) * a_hat;        
    }

    inline void exp(const V6d& k, M4d& SE3)
    {
        V3d p = k.head<3>();
        V3d w = k.tail<3>();

        double t = w.norm();
        SE3 = M4d::Identity();

        if(t < 1e-6){
            SE3.block<3, 1>(0, 3) = p;
            return;
        }

        V3d a = w / t;
        double ct = cos(t);
        double st = sin(t);

        M3d a_hat;
        matrix::skew(a, a_hat);

        M3d aa = a * a.transpose();

        M3d R = ct * M3d::Identity() + (1.0 - ct) * aa + sin(t) * a_hat;
        M3d V = st / t * M3d::Identity() + (1.0 - st / t) * aa + ((1 - ct) / t) * a_hat;

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
