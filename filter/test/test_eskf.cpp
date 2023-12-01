#include <filter/Eskf.hpp>
#include <iostream>

using namespace std;
using namespace filter;

double deg2rad = M_PI / 180.0;

void test_dv_dx()
{
    Eigen::Matrix<double, 1, 3> e1, H;
    e1 << 1, 0, 0;
    double r = 1;

    M3d Rw = Eigen::AngleAxisd(30 * deg2rad, V3d(1, 1, 1)).toRotationMatrix();
    M3d Ri = Eigen::AngleAxisd(20 * deg2rad, V3d(1, 2, -2)).toRotationMatrix();

    V3d v(1, 2, 3);

    V3d dx(1e-4, 1e-3, 1e-4);

    double b1 = e1.dot(Rw.transpose() * Ri.transpose() * v);
    M3d R;
    manifolds::exp(dx, R);

    double b2 = e1.dot(Rw.transpose() * (Ri * R).transpose() * v);

    H = e1 * Rw.transpose() * matrix::skewd(Ri.transpose() * v);

    cout << "b2 - b1: " << b2 - b1 << endl;
    cout << "J * dx: " << H * dx << endl;

    b2 = e1.dot((Rw * R).transpose() * Ri.transpose() * v);
    H = e1 * matrix::skewd(Rw.transpose() * Ri.transpose() * v);
    cout << "b2 - b1: " << b2 - b1 << endl;
    cout << "J * dx: " << H * dx << endl;

    double dr = 1e-3;
    b2 = e1.dot(Rw.transpose() * Ri.transpose() * v) / (r + dr);
    cout << "b2 - b1: " << b2 - b1 << endl;
    cout << "J * dx: " << -b1 * dr << endl;
}


int main()
{
    // State eskf(1, 1);

    // V3d am;
    // am.setRandom();
    // double dt = 0;

    // eskf.predict(am, dt);

    test_dv_dx();

    return 0;
}