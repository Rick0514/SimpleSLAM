#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>
#include <frontend/LidarOdometry.hpp>
#include <backend/Backend.hpp>

#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/EkfOdomProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

#include <config/params.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace frontend;
using namespace backend;
namespace bfs = boost::filesystem;

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

    ros::init(argc, argv, "loc");
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
    auto mmp = std::make_shared<MapManager>();
    mmp->registerVis(ldp);
    // construct LO
    auto lo = std::make_unique<LidarOdometry>(ldp, ftd, rdp, mmp);
    // backend
    auto bkd = std::make_unique<Backend>(ftd, mmp);

    // run lo, no dependency of lo and ftd, because lo alreay own a copy of
    // ftd, if ftd own lo, circular reference will happen!!
    trd::ResidentThread lo_thread([&](){
        lo->generateOdom();
    });

    ros::spin();

    // for valgrind test
    // ros::spinOnce();
    // sleep(2);
    lg->exitProgram();

    return 0;
}