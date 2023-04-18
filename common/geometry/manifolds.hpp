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

} // namespace common

}
