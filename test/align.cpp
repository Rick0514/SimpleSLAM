#include <PCR/NdtRegister.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <spdlog/spdlog.h>
#include <spdlog/version.h>
#include <spdlog/fmt/fmt.h>
#include <time/tictoc.hpp>

using namespace PCR;
using PointType = pcl::PointXYZ;
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
        std::cout << "usage: align target.pcd source.pcd" << std::endl;
        return 0;
    }

#ifdef DATA_DIR
    data_dir = DATA_DIR;
#endif

    auto target_pcd = fmt::format("{}/{}.pcd", data_dir, argv[1]);
    auto source_pcd = fmt::format("{}/{}.pcd", data_dir, argv[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
        SPDLOG_ERROR("failed to load: {}", target_pcd);
        return 0;
    }

    if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
        SPDLOG_ERROR("failed to load: {}", source_pcd);
        return 0;
    }

    PointCloudRegister<PointType>::Ptr pcr = std::make_shared<NdtRegister<PointType>>();
    
    Pose6d init_pose = Eigen::Isometry3d::Identity();
    
    time::tictoc tt;
    SPDLOG_INFO("start to downsample pc!");
    voxelDownSample(target_cloud, 0.1f);
    voxelDownSample(source_cloud, 0.1f);
    SPDLOG_INFO("downsample pc elapsed {:.6f}s", tt);

    SPDLOG_INFO("start to scan2map!");
    tt.tic();
    pcr->scan2Map(source_cloud, target_cloud, init_pose);
    SPDLOG_INFO("scan to map elapsed {:.3f}s", tt);
    
    return 0;
}
