#include <iostream>
#include <types/EigenTypes.hpp>
#include <geometry/trans.hpp>
#include <Eigen/Geometry>
#include <geometry/manifolds.hpp>

using namespace std;
using namespace EigenTypes;
using namespace geometry;
using namespace PCR;

struct A{
    Pose6d p;
};

void test_setPose6d()
{    
    Pose6d p;
    p.setIdentity();
    // p.pretranslate(V3d(1, 2, 3));
    // cout << p.matrix() << endl;

    V3<double> ypr(0.3, 0, 0);
    auto q = trans::ypr2q(ypr);
    cout << trans::q2ypr(q).transpose() << endl;

    V3<double> t(1, 2, 3);
    p.translate(t);
    p.rotate(q);
    
    cout << p.matrix() << endl;
}

void test_diagonal()
{
    Eigen::Matrix<float, 3, 1> v;
    v << 1, 2, 3;
    Eigen::Matrix3f m = v.asDiagonal();

    cout << m << endl;
}

void test_cwise()
{
    Eigen::Matrix<float, 3, 1> v;
    v << 2, 2, 2;
    Eigen::Matrix3f m = v.asDiagonal();

    m = m.array().square();

    cout << m << endl;
}

void test_euler()
{
    // should be 0.3, 0.2, 0.1
    Eigen::Quaternionf q(0.98334744, 0.0342708, 0.10602051, 0.14357218);
    
    // Eigen::Quaternionf q(0.65447885476098189, -0.00075549903893106993, 0.0061940238151494556, 0.75605455620671513);

    V3f ypr = trans::q2ypr(q);

    cout << ypr.transpose() << endl;

    cout << trans::ypr2q(ypr).coeffs().transpose() << endl;
}

void test_manifolds()
{
    V6d x;
    x << -0.00373127 ,  0.00599259 ,  0.00010917, -0.000599459,  0.000276421,  2.11126e-05;
    M4d SE3;
    manifolds::exp(x, SE3);
    cout << SE3 << endl;
}

int main(int argc, char const *argv[])
{
    // test_setPose6d();
    // test_diagonal();
    // test_cwise();
    // test_euler();
    
    test_manifolds();
    return 0;
}

