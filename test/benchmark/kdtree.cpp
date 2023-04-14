#include <benchmark/benchmark.h>

#include <types/PCLTypes.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <utils/Logger.hpp>
#include <nanoflann/pcl_adaptor.hpp>
#include <nanoflann/kfs_adaptor.hpp>

#include <deque>

using namespace PCLTypes;
using PointType = Pxyzi;

static typename PC<PointType>::Ptr map;

void checkKNN()
{
    auto lg = utils::logger::Logger::getInstance();

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(map);
    
    int k = 10;
    PointType p;
    bzero(p.data, 4);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    kdtree.nearestKSearch(p, k, k_indices, k_sqr_distances);

    lg->info("pcl kdtree get idx: {}", k_indices);
    lg->info("pcl kdtree get dist: {}", k_sqr_distances);

    k_indices.clear();
    std::vector<size_t> sk_indices;

    nanoflann::PointCloudKdtree<PointType, float> nkdtree;
    nkdtree.setInputCloud(map);
    nkdtree.nearestKSearch(p, k, sk_indices, k_sqr_distances);

    lg->info("nano kdtree get idx: {}", sk_indices);
    lg->info("nano kdtree get dist: {}", k_sqr_distances);
}

static void pclKdtree(benchmark::State& s)
{
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(map);

    int k = 5;
    PointType p;
    bzero(p.data, 4);

    for(auto _ : s)
    {
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        kdtree.nearestKSearch(p, k, k_indices, k_sqr_distances);
    }
}
BENCHMARK(pclKdtree);

static void nanoKdtree(benchmark::State& s)
{
    nanoflann::PointCloudKdtree<PointType, float> nkdtree;
    nkdtree.setInputCloud(map);

    int k = 5;
    PointType p;
    bzero(p.data, 4);

    for(auto _ : s){
        std::vector<size_t> k_indices;
        std::vector<float> k_sqr_distances;
        nkdtree.nearestKSearch(p, k, k_indices, k_sqr_distances);
    }
}
BENCHMARK(nanoKdtree);

// void checkKFS()
// {

//     auto lg = utils::logger::Logger::getInstance();

//     constexpr int nums = 100;
//     using matrix_t = Eigen::Matrix<float, nums, 3>;

//     srand((unsigned int) time(0));
//     matrix_t mat;
//     mat.setRandom();

//     int k = 5;
//     float q[3];
//     bzero(q, 3);
//     std::vector<size_t> k_indices(k);
//     std::vector<float> k_sqr_distances(k);

//     using metric_t = nanoflann::metric_L2_Simple::traits<float, matrix_t>;
//     using kdtree_t = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t, 3, metric_t>;
    
//     kdtree_t kdtree(3, mat);
//     nanoflann::KNNResultSet<float> resultSet(k);
//     resultSet.init(k_indices.data(), k_sqr_distances.data());
//     kdtree.index_->findNeighbors(resultSet, q);

//     lg->info("before idx: {}", k_indices);
//     lg->info("before dist: {}", k_sqr_distances);

//     // make kfs
//     using kfs_t = std::deque<KeyFrame<PointType, float>>;
//     kfs_t kfs(nums);
//     for(int i=0; i<nums; i++){
//         KeyFrame<PointType, float> kf;
//         kf.pose.setIdentity();
//         kf.pose.translation() = mat.row(i);
//     }

//     // nanoflann::KeyFramesKdtree<kfs_t, float> kf_kdtree(kfs);
//     // k_indices.clear()
//     // k_sqr_distances.clear();

// }


int main(int argc, char** argv) {              
    char arg0_default[] = "benchmark";           
    char* args_default = arg0_default;           
    if (!argv) {                                 
        argc = 1;                                  
        argv = &args_default;                      
    }

    auto lg = utils::logger::Logger::getInstance();

    // load a pc
    const std::string pcf = "/home/gy/.robot/data/maps/hqc/hqc.pcd";
    map = pcl::make_shared<PC<PointType>>();
    pcl::io::loadPCDFile<PointType>(pcf, *map);

    lg->info("load map size: {}", map->points.size());    

    checkKNN();
    // checkKFS();

    ::benchmark::Initialize(&argc, argv);        
    if (::benchmark::ReportUnrecognizedArguments(argc, argv))   return 1;
    ::benchmark::RunSpecifiedBenchmarks();       
    ::benchmark::Shutdown();                     

    return 0;
}
