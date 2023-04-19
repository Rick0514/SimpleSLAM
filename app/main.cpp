#include <frontend/Frontend.hpp>
#include <backend/Backend.hpp>
#include <frontend/MapManager.hpp>
#include <frontend/LidarOdometry.hpp>

#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

using namespace std;
using namespace frontend;
using namespace backend;

int main(int argc, char* argv[])
{
    // set log first
    auto lg = utils::logger::Logger::getInstance();
#ifdef LOG_FILE
    lg->setLogFile(LOG_FILE, spdlog::level::debug);
    // lg->setLogFile(LOG_FILE);
#endif
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;

    // lidar data proxy
    auto ldp = std::make_shared<LidarDataProxy>(nh, 10);   
    // reloc data proxy
    auto rdp = make_shared<RelocDataProxy>(nh);
    // frontend
    auto ftd = std::make_shared<Frontend>(100, 10);
    // mapmanager
    auto mmp = std::make_shared<MapManager>();
    // construct LO
    auto lo = std::make_unique<LidarOdometry>(ldp, ftd, rdp, mmp);

    ftd->run(std::move(lo));

    ros::spin();
    
    lg->exitProgram();

    return 0;
}