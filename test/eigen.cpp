#include <iostream>
#include <geometry/trans.hpp>
#include <types/PCLTypes.hpp>
#include <Eigen/Geometry>
#include <geometry/manifolds.hpp>

using namespace std;
using namespace EigenTypes;
using namespace PCLTypes;
using namespace geometry;

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

void test_map()
{
    double a[] = {1,2,3,4,5,6,7,8,9};
    
    // Eigen::Map<M3d> m(a);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> m(a);

    // cout << m << endl;

    int n = 4;
    vector<V3d> vs;
    V3d p(1,2,3);

    for(int i=0; i<n; i++)
    {
        V3d v = (double)(i+1) * p;
        vs.emplace_back(v);
    }

    using mat_t = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
    Eigen::Map<mat_t> J(vs.data()->data(), n, 3);
    cout << J << endl;
}

void test_map_pcl()
{
    Pxyzi p;
    auto ep = p.getVector3fMap();
    ep << 1, 2, 3;
    cout << "x: " << p.x << endl;
    cout << "y: " << p.y << endl;

    Pose6<float> tr;
    tr.setIdentity();
    tr.translate(V3f(1, 1, 1));

    ep = tr * ep;

    cout << "after trans : " << ep.transpose() << endl;
}

int main(int argc, char const *argv[])
{
    // test_setPose6d();
    // test_diagonal();
    // test_cwise();
    // test_euler();
    
    // test_manifolds();
    test_map();
    // test_map_pcl();

    return 0;
}

