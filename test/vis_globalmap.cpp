#include <iostream>
#include <fstream>

#include <config/params.hpp>
#include <utils/Logger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

using pt_t = pcl::PointXYZI;
using pc_t = pcl::PointCloud<pt_t>;

int main()
{
    auto cfg = config::Params::getInstance();

    string map_dir = cfg["saveMapDir"];

    auto tum = fmt::format("{}/tum.txt", map_dir);

    ifstream inf(tum);
    string line;
    int idx = 0;

    pc_t::Ptr globalmap = pcl::make_shared<pc_t>();

    while(std::getline(inf, line)){
        stringstream ss(line);
        
        string t;
        ss >> t;
        
        Eigen::Vector3f p;
        for(int i=0; i<3; i++)  ss >> p(i);

        Eigen::Quaternionf q;
        ss >> q.x() >> q.y() >> q.z() >> q.w();

        Eigen::Isometry3f af;
        af.setIdentity();
        af.translate(p);
        af.rotate(q);

        auto pcd_file = fmt::format("{}/{}.pcd", map_dir, idx++);
        pc_t pc;
        pcl::io::loadPCDFile<pt_t>(pcd_file, pc);
        pcl::transformPointCloud(pc, pc, af);   
        *globalmap += pc;   

    }
    inf.close();

    // downsample
    pcl::VoxelGrid<pt_t> voxelgrid;
    float grid_size = 0.5f;
    voxelgrid.setLeafSize(grid_size, grid_size, grid_size);
    voxelgrid.setInputCloud(globalmap);
    voxelgrid.filter(*globalmap);

    // vis
    pcl::visualization::CloudViewer vis("viewer");
    vis.showCloud(globalmap);
    cout << "points size: " << globalmap->size() << endl;

    while(!vis.wasStopped()){}

    return 0;    
    
}