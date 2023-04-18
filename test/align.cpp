#include <PCR/NdtRegister.hpp>
#include <PCR/LoamRegister.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <types/basic.hpp>
#include <utils/Logger.hpp>
#include <time/tictoc.hpp>

using namespace PCR;
static std::string data_dir;

template<typename PointType>
void voxelDownSample(pcl::shared_ptr<pcl::PointCloud<PointType>> cloud, float grid_size){
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(grid_size, grid_size, grid_size);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*cloud);
}

int main(int argc, char const *argv[])
{
    if(argc != 3) {
        std::cerr << "usage: align target.pcd source.pcd" << std::endl;
        return 0;
    }

#ifdef DATA_DIR
    data_dir = DATA_DIR;
#endif

    auto lg = utils::logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);

    auto target_pcd = fmt::format("{}/{}.pcd", data_dir, argv[1]);
    auto source_pcd = fmt::format("{}/{}.pcd", data_dir, argv[2]);

    pc_t::Ptr target_cloud(new pc_t());
    pc_t::Ptr source_cloud(new pc_t());

    if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
        lg->error("failed to load: {}", target_pcd);
        return 0;
    }

    if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
        lg->error("failed to load: {}", source_pcd);
        return 0;
    }

    // PointCloudRegister::Ptr pcr = std::make_shared<NdtRegister>();
    PointCloudRegister::Ptr pcr = std::make_shared<LoamRegister>();
    
    pose_t init_pose;
    init_pose.setIdentity();
    
    lg->info("target cloud size: {}", target_cloud->points.size());
    lg->info("source cloud size: {}", source_cloud->points.size());

    common::time::tictoc tt;
    lg->info("start to downsample pc!");
    voxelDownSample(target_cloud, 0.1f);
    voxelDownSample(source_cloud, 0.1f);
    lg->info("downsample pc elapsed {:.6f}s", tt);

    lg->info("--------- after downsample ---------");
    lg->info("target cloud size: {}", target_cloud->points.size());
    lg->info("source cloud size: {}", source_cloud->points.size());

    lg->info("start to scan2map!");
    tt.tic();
    pcr->scan2Map(source_cloud, target_cloud, init_pose);
    lg->info("scan to map elapsed {:.3f}s", tt);

    std::stringstream ss;
    ss << init_pose.matrix();
    lg->info("trans: \n{}", ss.str());

    // visualize
    pc_t::Ptr aligned(new pc_t());
    pcl::transformPointCloud(*source_cloud, *aligned, init_pose.matrix().cast<scalar_t>());
    pcl::visualization::PCLVisualizer vis("vis");
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> target_handler(target_cloud, 255.0, 0.0, 0.0);  // r
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> source_handler(source_cloud, 0.0, 255.0, 0.0);  // g
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> aligned_handler(aligned, 0.0, 0.0, 255.0);      // b
    vis.addPointCloud(target_cloud, target_handler, "target");
    vis.addPointCloud(source_cloud, source_handler, "source");
    vis.addPointCloud(aligned, aligned_handler, "aligned");
    vis.spin();
    
    return 0;
}
