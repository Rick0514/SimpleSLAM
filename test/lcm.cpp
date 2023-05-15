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

#include <config/params.hpp>
#include <utils/Logger.hpp>
#include <utils/Thread.hpp>

#include <frontend/MapManager.hpp>
#include <backend/LoopClosureManager.hpp>

using namespace utils;
using namespace frontend;
using namespace backend;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcm");
    ros::NodeHandle nh;

    auto lg = logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);

    auto cfg = config::Params::getInstance();

    // without vis
    auto mmp = std::make_shared<MapManager>();

    auto lcm = std::make_unique<LoopClosureManager>(mmp);
    // lcq should be enlarged or blocked !!!!!!!!
    auto& lcq = lcm->getLCQ();
    lcq.resize(100);

    // here need to unset last kf num
    const auto& kfo = mmp->getKeyFrameObjPtr();
    kfo->mKFNums = 0;

    lg->info("start to make context...");
    lcm->addContext();

    std::atomic_bool exit{false};
    auto backFunc = [&](){
        if(!nh.ok()){
            exit.store(true);
            kfo->LCIsHappening();
        }
        std::this_thread::yield();
    };
    trd::ResidentThread backThd(backFunc);
    
    lg->info("context matching...");
    
    std::unique_lock<std::mutex> lk(kfo->mLockKF);
    kfo->mKFcv.wait(lk, [&](){ return kfo->getEvent() == KeyFramesObj::Event::LC || exit.load(); });

    lg->info("get lcq size: {}", lcq.size());

    return 0;
}

