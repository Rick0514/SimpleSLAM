#include <string>

#include <ros/ros.h>

#include <types/PCLTypes.hpp>

#include <frontend/Frontend.hpp>
#include <backend/Backend.hpp>
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
    string pcd_file = "/home/gy/.robot/data/maps/hqc/hqc.pcd";  
    auto bkd = std::make_shared<backend::Backend<PointType>>(pcd_file);
    auto ftd = std::make_shared<frontend::Frontend>(100, 10);

    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;
    shared_ptr<DataProxy<PC<PointType>>> ldp = std::make_shared<LidarDataProxy<PC<PointType>>>(nh, 10);   
    RelocDataProxy rdp(nh);

    ftd->initLO<PointType>(ldp, bkd);
    ftd->initReloc<PointType>(rdp);

    ros::spin();

    return 0;
}