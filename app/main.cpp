#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/EkfOdomProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>
#include <dataproxy/Vis.hpp>

#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>
#include <frontend/LidarOdometry.hpp>

#include <backend/Backend.hpp>
#include <backend/LoopClosureManager.hpp>

#include <config/params.hpp>
#include <boost/filesystem.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <time/tictoc.hpp>

using namespace std;
using namespace frontend;
using namespace backend;
namespace bfs = boost::filesystem;
using Odom_t = EkfOdomProxy::Odom;

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

Odom_t imuFromROS(const sensor_msgs::ImuConstPtr& imu){
    auto q = imu->orientation;
    Qt<scalar_t> eq(q.w, q.x, q.y, q.z);

    Odom_t o;
    o.stamp = imu->header.stamp.toSec();
    o.q = eq;
    return o;
}

Odom_t wheelFromROS(const nav_msgs::OdometryConstPtr& wheel)
{
    auto p = wheel->pose.pose.position;
    auto q = wheel->pose.pose.orientation;
    Qt<scalar_t> eq(q.w, q.x, q.y, q.z);

    Odom_t o;
    o.stamp = wheel->header.stamp.toSec();
    o.t << p.x, p.y, p.z;
    o.q = eq;
    return o;
}

std::shared_ptr<pc_t> pcFromROS(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    auto cloud = std::make_shared<pc_t>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
    return cloud;
}

int main(int argc, char* argv[])
{
    bool memcheck = false;
    if(argc == 2 && strcmp(argv[1], "memcheck") == 0)   memcheck = true;

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
    auto enable_vis = cfg["vis"]["enable"].get<bool>();
    auto enable_lc = cfg["backend"]["lc"]["enable"].get<bool>();
    auto lidar_size = cfg["dataproxy"]["lidar_size"].get<int>();
    auto local_size = cfg["frontend"]["local_size"].get<int>();
    auto global_size = cfg["frontend"]["global_size"].get<int>();

    ros::init(argc, argv, "app");
    ros::NodeHandle nh;

    // vis
    auto vis = std::make_shared<Vis>(nh);
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
    auto mmp = std::make_shared<MapManager>();
    // construct LO
    auto lo = std::make_unique<LidarOdometry>(ldp, ftd, rdp, mmp);
    // construct LC
    std::shared_ptr<LoopClosureManager> lc;
    // backend
    auto bkd = std::make_unique<Backend>(ftd, mmp, lc);
    // vis or not
    if(enable_vis){
        mmp->registerVis(vis);
        lo->registerVis(vis);
    }  

    mmp->initAndRunUpdateMap();
    // run lo, no dependency of lo and ftd, because lo alreay own a copy of
    // ftd, if ftd own lo, circular reference will happen!!
    trd::ResidentThread lo_thread([&](){
        lo->generateOdom();
    });

    if (!constant::usebag){

        if(memcheck){
            // check 10s
            int checkCnt = 10 * 100;
            while (checkCnt) {
                ros::spinOnce();
                --checkCnt;
                ros::Duration(0.01).sleep();
            }
        }else{
            ros::spin();
        }

    }else{
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
                edp->imuHandler(imuFromROS(imu));
            }else if(tp == wheel_topic){
                nav_msgs::OdometryConstPtr wheel = m.instantiate<nav_msgs::Odometry>();
                edp->wheelHandler(wheelFromROS(wheel));
            }else if(tp == lidar_topic){
                sensor_msgs::PointCloud2ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
                ldp->subscribe(pcFromROS(pc));  
            }

            if(!nh.ok())  break;

            auto perc = (cur_time - start_time).toSec() / total_time;
            progressBar(perc);
            
            // memcheck for 5%
            if(memcheck && perc > 5.0 / 100)    break;
        }

        bag.close();

        std::cout << std::endl;
        std::cout << fmt::format("run time: {:.3f}s", tt.elapsed().count()) << std::endl;
        std::cout << fmt::format("bag time: {:.3f}s", total_time) << std::endl;
    }
    
    // for valgrind test
    lg->exitProgram();

    return 0;
}
