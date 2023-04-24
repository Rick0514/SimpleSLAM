// ekf dataproxy test
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>

#include <utils/Logger.hpp>

#include <dataproxy/EkfOdomProxy.hpp>

using namespace utils;
using namespace dataproxy;

int main(int argc, char** argv)
{

    auto lg = logger::Logger::getInstance();
    lg->setLogLevel(spdlog::level::debug);

    ros::init(argc, argv, "edp");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/edp", 1);

    auto edp = std::make_unique<EkfOdomProxy>(nh, 100);

    while(nh.ok())
    {
        auto p = edp->get()->consume_back();
        if(p){
            auto gp = tf2::toMsg(p->odom.cast<double>());
            nav_msgs::Odometry odom;
            odom.header.frame_id = "map";
            odom.header.stamp = ros::Time::now();
            odom.pose.pose = gp;
            pub.publish(odom);
        }

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    return 0;
}

