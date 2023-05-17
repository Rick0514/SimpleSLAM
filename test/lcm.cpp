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
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <config/params.hpp>
#include <utils/Logger.hpp>
#include <utils/Thread.hpp>

#include <frontend/MapManager.hpp>
#include <backend/LoopClosureManager.hpp>

using namespace EigenTypes;
using namespace utils;
using namespace frontend;
using namespace backend;

namespace vms = visualization_msgs;
namespace ph = std::placeholders;

class Vis final
{
    
private:
    ros::Publisher p_scan_, p_map_, p_lc_;
    vms::Marker mn_, me_;

public:

    Vis(ros::NodeHandle& nh)
    {
        p_scan_ = nh.advertise<sensor_msgs::PointCloud2>("/scan", 1, true);
        p_map_ = nh.advertise<sensor_msgs::PointCloud2>("/map", 1, true);
        p_lc_ = nh.advertise<vms::MarkerArray>("/ma", 1, true);

        mn_.header.frame_id = me_.header.frame_id = "map";
        mn_.action = me_.action = vms::Marker::ADD;

        mn_.type = vms::Marker::SPHERE_LIST;
        me_.type = vms::Marker::LINE_LIST;

        mn_.ns = "loop_nodes";
        me_.ns = "loop_edges";

        mn_.pose.orientation.w = me_.pose.orientation.w = 1.0;

        mn_.id = 0;
        me_.id = 1;

        mn_.scale.x = 0.3;
        mn_.scale.y = 0.3;
        mn_.scale.z = 0.3;

        mn_.color.r = 0;
        mn_.color.g = 0.8;
        mn_.color.b = 1;
        mn_.color.a = 1;

        me_.scale.x = 0.1;

        me_.color.r = 0.9;
        me_.color.g = 0.9;
        me_.color.b = 0;
        me_.color.a = 1;
    }

    void visLC(const pose_t& from, const pose_t& to){

        mn_.header.stamp = me_.header.stamp = ros::Time::now();

        V3d vf = from.translation().cast<double>();
        V3d vt = to.translation().cast<double>();

        auto pf = Eigen::toMsg(vf);
        auto pt = Eigen::toMsg(vt);

        mn_.points.clear();
        me_.points.clear();

        mn_.points.push_back(pf);
        mn_.points.push_back(pt);

        me_.points.push_back(pf);
        me_.points.push_back(pt);

        vms::MarkerArray ma;
        ma.markers.push_back(mn_);
        ma.markers.push_back(me_);
        p_lc_.publish(ma);
    }

    void visPC(const pc_t::ConstPtr& scan, const pc_t::ConstPtr& map, const pose_t& guess)
    {
        sensor_msgs::PointCloud2 r_scan, r_map;
        pc_t aligned;
        pcl::transformPointCloud(*scan, aligned, guess.matrix().cast<scalar_t>());
        
        pcl::toROSMsg(aligned, r_scan);
        pcl::toROSMsg(*map, r_map);
        r_map.header.frame_id = r_scan.header.frame_id = "map";
        
        p_scan_.publish(r_scan);
        p_map_.publish(r_map);
        
        // std::cin.get();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
};

// void visHandler(const pc_t::ConstPtr& scan, const pc_t::ConstPtr& map, const pose_t& guess)
// {

//     pc_t::Ptr aligned = pcl::make_shared<pc_t>();
//     pcl::transformPointCloud(*scan, *aligned, guess.matrix().cast<scalar_t>());
//     pcl::visualization::PCLVisualizer vis("vis");
//     pcl::visualization::PointCloudColorHandlerCustom<pt_t> target_handler(map, 255.0, 0.0, 0.0);        // r
//     pcl::visualization::PointCloudColorHandlerCustom<pt_t> aligned_handler(aligned, 0.0, 0.0, 255.0);   // b
//     vis.addPointCloud(map, target_handler, "target");
//     vis.addPointCloud(aligned, aligned_handler, "aligned");

//     vis.spin();
//     vis.close();
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcm");
    ros::NodeHandle nh;

    auto lg = logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);
    lg->debug("lg set to debug...");

    auto cfg = config::Params::getInstance();

    Vis vis(nh);

    // without vis
    auto mmp = std::make_shared<MapManager>();

    auto lcm = std::make_unique<LoopClosureManager>(mmp);
    lcm->registerVis(std::bind(&Vis::visPC, &vis, ph::_1, ph::_2, ph::_3), std::bind(&Vis::visLC, &vis, ph::_1, ph::_2));

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

    lg->exitProgram();

    return 0;
}

