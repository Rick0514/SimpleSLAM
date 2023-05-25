#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>
#include <frontend/LidarOdometry.hpp>

#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>
#include <dataproxy/Vis.hpp>

#include <utils/Logger.hpp>
#include <config/params.hpp>

using namespace std;
using namespace frontend;

int main(int argc, char* argv[])
{
    // get params
    auto cfg = config::Params::getInstance();
    auto lidar_size = cfg["dataproxy"]["lidar_size"].get<int>();
    auto local_size = cfg["frontend"]["local_size"].get<int>();
    auto global_size = cfg["frontend"]["global_size"].get<int>();

    // set log first
    auto lg = utils::logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);
#ifdef LOG_FILE
    lg->setLogFile(LOG_FILE, spdlog::level::debug);
    // lg->setLogFile(LOG_FILE);
#endif

    string pcd_file = cfg["pcd_file"];
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;

    bool enable_vis = cfg["vis"]["enable"].get<bool>();
    auto vis = std::make_shared<Vis>(nh);

    // lidar data proxy
    auto ldp = std::make_shared<LidarDataProxy>(nh, lidar_size);   
    // reloc data proxy
    auto rdp = std::make_shared<RelocDataProxy>(nh);
    // frontend
    auto ftd = std::make_shared<Frontend>(local_size, global_size);
    // mapmanager
    auto mmp = std::make_shared<MapManager>(pcd_file);
    // construct LO
    auto lo = std::make_unique<LidarOdometry>(ldp, ftd, rdp, mmp);

    // vis or not
    if(enable_vis){
        mmp->registerVis(vis);
        lo->registerVis(vis);
    }

    mmp->showSubmap();
    // run lo, no dependency of lo and ftd, because lo alreay own a copy of
    // ftd, if ftd own lo, circular reference will happen!!
    trd::ResidentThread lo_thread([&](){
        lo->generateOdom();
    });

    ros::spin();

    lg->exitProgram();

    return 0;
}
