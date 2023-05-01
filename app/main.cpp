#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>
#include <frontend/LidarOdometry.hpp>
#include <backend/Backend.hpp>

#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/EkfOdomProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

#include <config/params.hpp>
#include <boost/filesystem.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <time/tictoc.hpp>

using namespace std;
using namespace frontend;
using namespace backend;
namespace bfs = boost::filesystem;

void progressBar(stamp_t perc)
{
    #define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
    #define PBWIDTH 60
    int val = (int) (perc * 100);
    int lpad = (int) (perc * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

int main(int argc, char* argv[])
{
    // set log first
    auto lg = utils::logger::Logger::getInstance();
#ifdef LOG_FILE
    // lg->setLogFile(LOG_FILE, spdlog::level::debug);
    lg->setLogFile(LOG_FILE);
#endif

    // get params
    auto cfg = config::Params::getInstance();
    auto save_map = cfg["saveMapDir"].get<std::string>();
    auto b_save_map = bfs::path(save_map);
    if(!bfs::exists(b_save_map)){
        lg->error("your mapdir [ {} ] is not exist, please check!!", b_save_map.string());
        return -1;
    }

    auto mode = cfg["mode"].get<std::string>();
    auto lidar_size = cfg["dataproxy"]["lidar_size"].get<int>();
    auto local_size = cfg["frontend"]["local_size"].get<int>();
    auto global_size = cfg["frontend"]["global_size"].get<int>();

    ros::init(argc, argv, "app");
    ros::NodeHandle nh;

    // lidar data proxy
    auto ldp = std::make_shared<LidarDataProxy>(nh, lidar_size);   
    // ekf data proxy
    auto edp = std::make_shared<EkfOdomProxy>(nh, local_size);

    // when slam mode, no reloc data proxy
    std::shared_ptr<RelocDataProxy> rdp;
    // frontend
    std::shared_ptr<Frontend> ftd;
    if(mode == "lo")    ftd = std::make_shared<Frontend>(local_size, global_size);
    if(mode == "lio")   ftd = std::make_shared<Frontend>(edp->get(), global_size);

    // mapmanager
    auto mmp = std::make_shared<MapManager>(ldp);

    // construct LO
    auto lo = std::make_unique<LidarOdometry>(ldp, ftd, rdp, mmp);
    // backend
    auto bkd = std::make_unique<Backend>(ftd, mmp);

    // run lo, no dependency of lo and ftd, because lo alreay own a copy of
    // ftd, if ftd own lo, circular reference will happen!!
    trd::ResidentThread lo_thread([&](){
        lo->generateOdom();
    });

    if constexpr (constant::usebag)
    {
        std::string fn = cfg["rosbag"];
        rosbag::Bag bag;
        bag.open(fn, rosbag::bagmode::Read);

        // get total time of bag
        stamp_t total_time;
        ros::Time start_time, end_time;
        {
            rosbag::View view(bag);
            start_time = view.getBeginTime();
            end_time = view.getEndTime();
            total_time = (end_time - start_time).toSec();
        }
        lg->info("rosbag total time: {:.3f}s", total_time);

        std::string lidar_topic = cfg["dataproxy"]["lidar"];
        std::string imu_topic = cfg["dataproxy"]["imu"];
        std::string wheel_topic = cfg["dataproxy"]["wheel"];
        std::vector<std::string> topics{imu_topic, wheel_topic, lidar_topic};

        common::time::tictoc tt;
        for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topics)))
        {   
            std::string tp = m.getTopic();
            auto cur_time =  m.getTime();

            if(tp == imu_topic){
                sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
                edp->imuHandler(imu);
            }else if(tp == wheel_topic){
                nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
                edp->wheelHandler(odom);
            }else if(tp == lidar_topic){
                sensor_msgs::PointCloud2ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
                ldp->subscribe(pc);  
            }

            if(!nh.ok())  break;

            auto perc = (cur_time - start_time).toSec() / total_time;
            progressBar(perc);
            
        }

        bag.close();

        std::cout << std::endl;
        std::cout << fmt::format("run time: {:.3f}s", tt.elapsed().count()) << std::endl;
        std::cout << fmt::format("bag time: {:.3f}s", total_time) << std::endl;

    }else{
        ros::spin();
    }

    // for valgrind test
    // ros::spinOnce();
    // sleep(2);
    lg->exitProgram();

    return 0;
}