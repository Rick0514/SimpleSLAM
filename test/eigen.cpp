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

int main(int argc, char const *argv[])
{

    return 0;
}

