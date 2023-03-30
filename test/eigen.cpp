#include <iostream>
#include <types/EigenTypes.hpp>

using namespace std;
using namespace EigenTypes;

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

int main(int argc, char const *argv[])
{
    test_diagonal();
    test_cwise();
    return 0;
}

