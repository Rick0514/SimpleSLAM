#include <string>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <frontend/Frontend.hpp>
// #include <backend/Backend.hpp>
#include <frontend/LidarOdometry.hpp>
#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

using namespace std;
using namespace PCLTypes;
using namespace dataproxy;
using PointType = Pxyzi;
using PCType = PC<PointType>;

int main(int argc, char* argv[])
{
    // set log first
    auto lg = utils::logger::Logger::getInstance();
#ifdef LOG_FILE
    lg->setLogFile(LOG_FILE, spdlog::level::debug);
    // lg->setLogFile(LOG_FILE);
#endif

    // string pcd_file = "/home/gy/.robot/data/maps/hqc/hqc.pcd";  
    // auto bkd = std::make_shared<backend::Backend<PointType>>(pcd_file);
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;

    // lidar data proxy
    shared_ptr<DataProxy<PC<PointType>>> ldp = std::make_shared<LidarDataProxy<PC<PointType>>>(nh, 10);   
    // reloc data proxy
    auto rdp = make_shared<RelocDataProxy>(nh);
    // frontend
    auto ftd = std::make_shared<frontend::Frontend>(100, 10);
    // construct LO
    auto lo = new frontend::LidarOdometry<PointType>(ldp, ftd, rdp);

    // show global pc
    ros::Publisher gpc_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);
    sensor_msgs::PointCloud2 rospc;
    pcl::toROSMsg(*(lo->getSubmap()), rospc);
    rospc.header.frame_id = "map";
    gpc_pub.publish(rospc);

    ftd->run(lo);

    ros::spin();

    return 0;
}