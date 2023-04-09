#include <benchmark/benchmark.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// compare ros publish at once or copy pointcloud
using PT = pcl::PointXYZI;

static ros::Publisher* pub;
static pcl::PointCloud<PT>::Ptr pc;

static void publishPC(benchmark::State& s)
{
    sensor_msgs::PointCloud2 p;
    pcl::toROSMsg(*pc, p);
    p.header.frame_id = "map";
    
    for(auto _ : s){
        pub->publish(p);
        ros::spinOnce();
    }
}
BENCHMARK(publishPC);

static void copyPC(benchmark::State& s)
{
    for(auto _ : s){
        pcl::PointCloud<PT>::Ptr newpc(new pcl::PointCloud<PT>);
        pcl::copyPointCloud(*pc, *newpc);
    }
}
BENCHMARK(copyPC);


int main(int argc, char** argv) {              
    char arg0_default[] = "benchmark";           
    char* args_default = arg0_default;           
    if (!argv) {                                 
        argc = 1;                                  
        argv = &args_default;                      
    }

    ros::init(argc, argv, "rospub_bm");
    ros::NodeHandle nh;
    ros::Publisher tmp_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan", 1);
    pub = &tmp_pub;

    // load pc
    std::string pcd_file = "/home/gy/.robot/data/maps/hqc/hqc.pcd";
    pc.reset(new pcl::PointCloud<PT>());
    pcl::io::loadPCDFile<PT>(pcd_file, *pc);
    
    std::cout << "before pc size: " << pc->size() << std::endl;
    pcl::VoxelGrid<PT> voxelgrid;
    float grid_size = 1.0;
    voxelgrid.setLeafSize(grid_size, grid_size, grid_size);
    voxelgrid.setInputCloud(pc);
    voxelgrid.filter(*pc);
    std::cout << "after pc size: " << pc->size() << std::endl;    

    ::benchmark::Initialize(&argc, argv);        
    if (::benchmark::ReportUnrecognizedArguments(argc, argv))   return 1;
    ::benchmark::RunSpecifiedBenchmarks();       
    ::benchmark::Shutdown();                     
    return 0;                                    
}   