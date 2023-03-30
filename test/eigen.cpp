#include <iostream>
#include <types/EigenTypes.hpp>
#include <geometry/trans.hpp>
#include <Eigen/Geometry>

using namespace std;
using namespace EigenTypes;
using namespace geometry;

void test_setPose6d()
{    
    Pose6d p;
    p.setIdentity();
    p.pretranslate(V3d(1, 2, 3));

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
    Eigen::Quaternionf q(0.98334744, 0.0342708, 0.10602051, 0.14357218);

    V3f ypr = trans::q2ypr(q);

    // should be 0.3, 0.2, 0.1
    cout << ypr.transpose() << endl;
}

int main(int argc, char const *argv[])
{
    test_diagonal();
    test_cwise();
    test_euler();
    return 0;
}
