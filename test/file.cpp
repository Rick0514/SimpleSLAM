#include <utils/File.hpp>
#include <config/params.hpp>
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

void test_load_tum()
{
    auto cfg = config::Params::getInstance();
    std::string dir = cfg["saveMapDir"];
    auto t2p = file::loadFromTum<float>(dir);

    for(auto&& e : t2p){
        cout << "time: " << e.first << endl;
        cout << "pose: " << e.second.translation().transpose() << endl;
    }
}

int main()
{
    // test_save_tum();
    test_load_tum();

    return 0;
}