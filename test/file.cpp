#include <utils/File.hpp>
#include <chrono>
#include <iostream>

using namespace std;
using namespace chrono;
using namespace utils;

double toSec(const time_point<system_clock>& clk)
{
    auto now = time_point_cast<milliseconds>(clk);
    return (double)now.time_since_epoch().count() / 1000;
}

void test_save_tum()
{
    cout << "now: " << fmt::format("{:.3f}s", toSec(system_clock::now())) << endl;    

    EigenTypes::Pose6<float> p;
    p.setIdentity();

    string dir;
#ifdef DATA_DIR
    dir = DATA_DIR;
#endif

    srand((unsigned int) time(0));
    for(int i=0; i<10; i++){
        p.translate(EigenTypes::V3f::Random());
        p.rotate(Eigen::AngleAxisf(0.1, EigenTypes::V3f::Random()));
        sleep(1);
        file::writeAsTum(dir, toSec(system_clock::now()), p);
        if(i == 5){
            cout << "rebuild it!" << endl;
            file::removeTum(dir);
        }
    }
}

int main()
{
    test_save_tum();

    return 0;
}