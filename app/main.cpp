#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>
#include <frontend/LidarOdometry.hpp>
#include <backend/Backend.hpp>

#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/EkfOdomProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

using namespace std;
using namespace frontend;
using namespace backend;

int main(int argc, char* argv[])
{
    // set log first
    auto lg = utils::logger::Logger::getInstance();
#ifdef LOG_FILE
    // lg->setLogFile(LOG_FILE, spdlog::level::debug);
    lg->setLogFile(LOG_FILE);
#endif

    if(argc != 2){
        lg->error("please input: app [mode], mode : lo, lio");
        return -1;
    }
    auto mode = std::string(argv[1]);
    lg->info("slam mode: {}", mode);
    if(mode != "lo" && mode != "lio"){
        lg->error("no such mode: {}", mode);
        return -1;
    }

    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;

    // lidar data proxy
    auto ldp = std::make_shared<LidarDataProxy>(nh, 10);   
    // ekf data proxy
    auto edp = std::make_shared<EkfOdomProxy>(nh, 100);

    // when slam mode, no reloc data proxy
    std::shared_ptr<RelocDataProxy> rdp;
    // frontend
    std::shared_ptr<Frontend> ftd;
    if(mode == "lo")    ftd = std::make_shared<Frontend>(100, 10);
    if(mode == "lio")   ftd = std::make_shared<Frontend>(edp->get(), 10);

    // mapmanager
    auto mmp = std::make_shared<MapManager>();
    mmp->registerVis(ldp);
    // construct LO
    auto lo = std::make_unique<LidarOdometry>(ldp, ftd, rdp, mmp);
    // backend
    auto bkd = std::make_unique<Backend>(ftd, mmp);

    ftd->run(std::move(lo));
    ros::spin();
    
    lg->exitProgram();

    return 0;
}