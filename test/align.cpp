#include <PCR/NdtRegister.hpp>
#include <PCR/LoamRegister.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <pcp/pcp.hpp>

#include <nanoflann/pcl_adaptor.hpp>

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

double getFitnessScore(const pc_t::ConstPtr& src, const pc_t::ConstPtr& dst, const pose_t& p)
{
    double fitness_score = 0.0;
    // Transform the input dataset using the final transformation
    pc_t input_transformed;
    pcl::transformPointCloud(*src, input_transformed, p.matrix().cast<float>());

    std::vector<size_t> nn_indices(1);
    std::vector<float> nn_dists(1);

    nanoflann::PointCloudKdtree<pt_t, float> nkdtree;
    nkdtree.setInputCloud(dst);

    // nkdtree.nearestKSearch(p, k, sk_indices, k_sqr_distances);
    // For each point in the source dataset
    int nr = 0;
    for (auto& point : input_transformed) {
        // Find its nearest neighbor in the target
        nkdtree.nearestKSearch(point.data, 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= 1.0f) {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0)
        return (fitness_score / nr);

    return -1;
}


int main(int argc, char const *argv[])
{
    if(argc < 4) {
        std::cerr << "usage: align target.pcd source.pcd method" << std::endl;
        return 0;
    }

#ifdef DATA_DIR
    data_dir = DATA_DIR;
#endif

    auto lg = utils::logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);

    auto target_pcd = fmt::format("{}/{}.pcd", data_dir, argv[1]);
    auto source_pcd = fmt::format("{}/{}.pcd", data_dir, argv[2]);
    std::string method = argv[3];

    pose_t before_pose, init_pose;
    before_pose.setIdentity();
    
    if(argc > 4){
        // read init_pose
        std::ifstream inf(fmt::format("{}/init_pose.txt", data_dir));
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                inf >> before_pose.matrix()(i, j);
            }
        }
        cout << before_pose.matrix() << endl;
    }

    init_pose = before_pose;

    pc_t::Ptr target_cloud(new pc_t());
    pc_t::Ptr source_cloud(new pc_t());

    if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
        lg->error("failed to load: {}", target_pcd);
        return -1;
    }

    if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
        lg->error("failed to load: {}", source_pcd);
        return -1;
    }

    PointCloudRegister::Ptr pcr;
    if(method == "loam"){
        pcr = std::make_shared<LoamRegister>();
    }else if(method == "ndt"){
        pcr = std::make_shared<NdtRegister>();
    }else{
        lg->error("no such method!!");
        return -1;
    }
    
    lg->info("target cloud size: {}", target_cloud->points.size());
    lg->info("source cloud size: {}", source_cloud->points.size());

    common::time::tictoc tt;
    lg->info("start to downsample pc!");
    voxelDownSample(target_cloud, 0.1f);
    voxelDownSample(source_cloud, 0.1f);

    // voxel downsample version 2
    // pcp::VoxelDownSampleV3 vds(0.1f);
    // source_cloud = vds.filter<pt_t>(source_cloud);
    // target_cloud = vds.filter<pt_t>(target_cloud);

    lg->info("downsample pc elapsed {:.6f}s", tt);

    lg->info("--------- after downsample ---------");
    lg->info("target cloud size: {}", target_cloud->points.size());
    lg->info("source cloud size: {}", source_cloud->points.size());
    
    lg->info("start to scan2map!");
    tt.tic();
    if(!pcr->scan2Map(source_cloud, target_cloud, init_pose))
        lg->warn("not converge!!");
    lg->info("scan to map elapsed {:.3f}s", tt);

    lg->warn("get fitness score: {}", getFitnessScore(source_cloud, target_cloud, init_pose));

    std::stringstream ss;
    ss << init_pose.matrix();
    lg->info("trans: \n{}", ss.str());

    // visualize
    pc_t::Ptr aligned(new pc_t());
    pc_t::Ptr vis_source(new pc_t());
    pcl::transformPointCloud(*source_cloud, *aligned, init_pose.matrix().cast<scalar_t>());
    pcl::transformPointCloud(*source_cloud, *vis_source, before_pose.matrix().cast<scalar_t>());
    pcl::visualization::PCLVisualizer vis("vis");
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> target_handler(target_cloud, 255.0, 0.0, 0.0);  // r
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> source_handler(vis_source, 0.0, 255.0, 0.0);  // g
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> aligned_handler(aligned, 0.0, 0.0, 255.0);      // b
    vis.addPointCloud(target_cloud, target_handler, "target");
    vis.addPointCloud(vis_source, source_handler, "source");
    vis.addPointCloud(aligned, aligned_handler, "aligned");
    vis.spin();
    
    return 0;
}
