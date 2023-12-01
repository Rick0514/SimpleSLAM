#pragma once

#include <types/EigenTypes.hpp>
#include <geometry/matrix.hpp>

namespace geometry
{
namespace manifolds
{
    using namespace EigenTypes;
    
    template <typename Scalar>
    using V6 = Eigen::Matrix<Scalar, 6, 1>;

    template <typename Scalar>
    inline void exp(const V3<Scalar>& w, M3<Scalar>& SO3)
    {
        Scalar t = w.norm();
        if(t < 1e-6){
            SO3 = M3<Scalar>::Identity();
            return;
        }

        V3<Scalar> a = w / t;
        Scalar ct = cos(t);

        M3<Scalar> a_hat;
        matrix::skew(a, a_hat);
        SO3 = ct * M3<Scalar>::Identity() + (1.0 - ct) * a * a.transpose() + sin(t) * a_hat;        
    }

    template <typename Scalar>
    inline void exp(const V6<Scalar>& k, M4<Scalar>& SE3)
    {
        V3<Scalar>p = k.template head<3>();
        V3<Scalar> w = k.template tail<3>();

        Scalar t = w.norm();
        SE3 = M4<Scalar>::Identity();

        if(t < 1e-6){
            SE3.template block<3, 1>(0, 3) = p;
            return;
        }

        V3<Scalar> a = w / t;
        Scalar ct = cos(t);
        Scalar st = sin(t);

        M3<Scalar> a_hat;
        matrix::skew(a, a_hat);

        M3<Scalar> aa = a * a.transpose();

        M3<Scalar> R = ct * M3<Scalar>::Identity() + (1.0 - ct) * aa + sin(t) * a_hat;
        M3<Scalar> V = st / t * M3<Scalar>::Identity() + (1.0 - st / t) * aa + ((1 - ct) / t) * a_hat;

        SE3.template block<3, 3>(0, 0) = R;
        SE3.template block<3, 1>(0, 3) = V * p;
    }

    template <typename Scalar>
    inline void J_SE3(const V3<Scalar>& p, Eigen::Matrix<Scalar, 3, 6>& J){
        J.template block<3, 3>(0, 0).setIdentity();
        M3<Scalar> s;
        matrix::skew(p, s);
        J.template block<3, 3>(0, 3) = -s;
    }

    inline M3d Jr_SO3_inv(const V3d& v)
    {
        M3d J;
        J.setIdentity();
        double theta = v.norm();

        if (theta < 1e-6) {
            return J;
        }
        double ht = theta / 2;
        double a = ht * cos(ht) / sin(ht);
        M3d sv = matrix::skewd(v);
        J = M3d::Identity() - 0.5 * sv + (1.0 - a) * sv * sv / theta / theta;
        return J;
    }

    inline V3d log_SO3(const M3d &R) 
    {
        // note switch to base 1
        double R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
        double R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
        double R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

        // Get trace(R)
        const double tr = R.trace();
        Eigen::Vector3d omega;

        // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
        // we do something special
        if (tr + 1.0 < 1e-10) 
        {
            if (std::abs(R33 + 1.0) > 1e-5)
                omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * Eigen::Vector3d(R13, R23, 1.0 + R33);
            else if (std::abs(R22 + 1.0) > 1e-5)
                omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * Eigen::Vector3d(R12, 1.0 + R22, R32);
            else
                // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
                omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * Eigen::Vector3d(1.0 + R11, R21, R31);
        } else {
            double magnitude;
            const double tr_3 = tr - 3.0; // always negative
            if (tr_3 < -1e-7) {
                double theta = acos((tr - 1.0) / 2.0);
                magnitude = theta / (2.0 * sin(theta));
            }else{
                // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
                // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
                // see https://github.com/borglab/gtsam/issues/746 for details
                magnitude = 0.5 - tr_3 / 12.0;
            }
            omega = magnitude * Eigen::Vector3d(R32 - R23, R13 - R31, R21 - R12);
        }
        return omega;
    }
} // namespace common

}

