/**
 * @file lcm.cpp
 * @author hgy_rick (gy_rick@foxmail.com)
 * @brief use for test loop closure manager and backend by the way
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <config/params.hpp>
#include <utils/Logger.hpp>
#include <utils/Thread.hpp>

#include <frontend/MapManager.hpp>
#include <backend/LoopClosureManager.hpp>

using namespace utils;
using namespace frontend;
using namespace backend;

void visHandler(const pc_t::ConstPtr& scan, const pc_t::ConstPtr& map, const pose_t& guess)
{

    pc_t::Ptr aligned = pcl::make_shared<pc_t>();
    pcl::transformPointCloud(*scan, *aligned, guess.matrix().cast<scalar_t>());
    pcl::visualization::PCLVisualizer vis("vis");
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> target_handler(map, 255.0, 0.0, 0.0);        // r
    pcl::visualization::PointCloudColorHandlerCustom<pt_t> aligned_handler(aligned, 0.0, 0.0, 255.0);   // b
    vis.addPointCloud(map, target_handler, "target");
    vis.addPointCloud(aligned, aligned_handler, "aligned");

    vis.spin();
    vis.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcm");
    ros::NodeHandle nh;

    auto lg = logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);
    lg->debug("lg set to debug...");

    auto cfg = config::Params::getInstance();

    // without vis
    auto mmp = std::make_shared<MapManager>();

    auto lcm = std::make_unique<LoopClosureManager>(mmp);
    lcm->registerVis(visHandler);

    // lcq should be enlarged or blocked !!!!!!!!
    auto& lcq = lcm->getLCQ();
    lcq.resize(100);

    // here need to unset last kf num
    const auto& kfo = mmp->getKeyFrameObjPtr();
    kfo->mKFNums = 0;

    lg->info("start to make context...");
    lcm->addContext();

    auto backFunc = [&](){
        if(!nh.ok() && !lg->isProgramExit()){
            lg->exitProgram();
            kfo->LCIsHappening();
        }
        std::this_thread::yield();
    };
    trd::ResidentThread backThd(backFunc);
    
    lg->info("context matching...");
    
    std::unique_lock<std::mutex> lk(kfo->mLockKF);
    kfo->mKFcv.wait(lk, [&](){ return kfo->getEvent() == KeyFramesObj::Event::LC; });

    lg->info("get lcq size: {}", lcq.size());

    return 0;
}

