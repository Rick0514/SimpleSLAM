#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>
#include <frontend/LidarOdometry.hpp>

#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

#include <utils/Logger.hpp>

using namespace std;
using namespace frontend;

int main(int argc, char* argv[])
{
    // set log first
    auto lg = utils::logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);
#ifdef LOG_FILE
    lg->setLogFile(LOG_FILE, spdlog::level::debug);
    // lg->setLogFile(LOG_FILE);
#endif

    string pcd_file = "/home/hgy/.robot/data/maps/hqc/hqc.pcd";  
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;

    // lidar data proxy
    auto ldp = std::make_shared<LidarDataProxy>(nh, 10);   
    // reloc data proxy
    auto rdp = make_shared<RelocDataProxy>(nh);
    // frontend
    auto ftd = std::make_shared<Frontend>(100, 10);
    // mapmanager
    auto mmp = std::make_shared<MapManager>(pcd_file);
    // construct LO
    auto lo = std::make_unique<LidarOdometry>(ldp, ftd, rdp, mmp);

    // show global pc
    ros::Publisher gpc_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);
    sensor_msgs::PointCloud2 rospc;
    pcl::toROSMsg(*(mmp->getSubmap()), rospc);
    rospc.header.frame_id = "map";
    gpc_pub.publish(rospc);

    ftd->run(std::move(lo));

    ros::spin();

    lg->exitProgram();

    return 0;
}