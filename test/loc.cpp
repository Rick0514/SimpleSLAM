#include <string>

#include <ros/ros.h>

#include <types/PCLTypes.hpp>

#include <frontend/Frontend.hpp>
#include <backend/Backend.hpp>
#include <frontend/LidarOdometry.hpp>
#include <dataproxy/LidarDataProxy.hpp>

using namespace std;
using namespace PCLTypes;
using namespace dataproxy;
using PointType = Pxyzi;

int main(int argc, char* argv[])
{
    string pcd_file = "/home/hgy/.robot/data/maps/lvisam/lvisam.pcd";  
    auto bkd = std::make_shared<backend::Backend<PointType>>(pcd_file);
    auto ftd = std::make_shared<frontend::Frontend>();

    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;
    shared_ptr<DataProxy<PC<PointType>>> ldp = std::make_shared<LidarDataProxy<PC<PointType>>>(nh, 10);   

    ftd->initLO<PointType>(ldp, bkd);

    ros::spin();

    return 0;
}