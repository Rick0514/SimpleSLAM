#include <pcp/pcp.hpp>

#include <types/basic.hpp>

#include <utils/Logger.hpp>
#include <time/tictoc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

using std::string;

static constexpr float gs = 0.7f;
static pc_t::Ptr sm;

void test_pcl_vgf(ros::Publisher& pub)
{
    auto lg = utils::logger::Logger::getInstance();
    pc_t pc;

    common::time::tictoc tt;

    pcl::VoxelGrid<pt_t> voxelgrid;
    voxelgrid.setLeafSize(gs, gs, gs);
    voxelgrid.setInputCloud(sm);
    voxelgrid.filter(pc);

    lg->info("map pts: {}, ds pts: {}, ds cost: {:.3f}s", sm->size(), pc.size(), tt);

    sensor_msgs::PointCloud2 rospc;
    pcl::toROSMsg(pc, rospc);
    rospc.header.frame_id = "map";
    pub.publish(rospc);
}

void test_my_vgf(ros::Publisher& pub)
{
    auto lg = utils::logger::Logger::getInstance();

    pc_t::Ptr pc = pcl::make_shared<pc_t>();

    common::time::tictoc tt;

    // pcp::VoxelDownSampleV2 vds(gs);
    // pc = vds.filter<pt_t>(sm);

    pcp::VoxelDownSampleV3 vds(gs);
    vds.filter<pt_t>(sm, pc);

    lg->info("map pts: {}, ds pts: {}, ds cost: {:.3f}s", sm->size(), pc->size(), tt);

    sensor_msgs::PointCloud2 rospc;
    pcl::toROSMsg(*pc, rospc);
    rospc.header.frame_id = "map";
    pub.publish(rospc);
}

int main(int argc, char** argv)
{
    const string pcd_file = "/home/gy/.robot/data/maps/hqc/hqc.pcd";
    sm = pcl::make_shared<pc_t>();
    pcl::io::loadPCDFile<pt_t>(pcd_file, *sm);

    ros::init(argc, argv, "vgf");
    ros::NodeHandle nh;
    auto pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/pcl", 1, true);
    auto pub_my = nh.advertise<sensor_msgs::PointCloud2>("/my", 1, true);

    test_pcl_vgf(pub_pcl);
    test_my_vgf(pub_my);

    ros::spin();

    return 0;
}