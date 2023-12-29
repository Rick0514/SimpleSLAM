#include <dataproxy/EskfProxy.hpp>
#include <config/params.hpp>
#include <geometry/trans.hpp>

namespace dataproxy {

using std::string;
using namespace EigenTypes;

EskfProxy::EskfProxy()
{
    auto params = config::Params::getInstance()["dataproxy"];

    lg = utils::logger::Logger::getInstance();

    eskf = new filter::State();

    auto imu_topic = params["imu"].get<string>();
    auto wheel_topic = params["wheel"].get<string>();

    mImuSub = nh.subscribe(imu_topic, 100, &EskfProxy::onImu, this);
    mWheelSub = nh.subscribe(wheel_topic, 100, &EskfProxy::onWheel, this);

    odom_path.header.frame_id = "map";
    gt_path.header.frame_id = "map";

    vis_R = nh.advertise<geometry_msgs::Vector3>("/eskf/R", 10);
    vis_p = nh.advertise<geometry_msgs::Vector3>("/eskf/p", 10);
    vis_v = nh.advertise<geometry_msgs::Vector3>("/eskf/v", 10);
    vis_w = nh.advertise<geometry_msgs::Vector3>("/eskf/w", 10);
    vis_RW = nh.advertise<geometry_msgs::Vector3>("/eskf/RW", 10);
    vis_hr = nh.advertise<geometry_msgs::Vector3>("/eskf/hr", 10);
    vis_odom = nh.advertise<nav_msgs::Odometry>("/eskf/odom", 10);
    vis_odom_path = nh.advertise<nav_msgs::Path>("/eskf/odom_path", 10);
    vis_gt_path = nh.advertise<nav_msgs::Path>("/eskf/gt_path", 10);

    timer = nh.createTimer(ros::Duration(0.1), &EskfProxy::pubEskfState, this);
}

geometry_msgs::Vector3 toGeoVec(const V3d& v){
    geometry_msgs::Vector3 gv;
    gv.x = v(0);
    gv.y = v(1);
    gv.z = v(2);
    return gv;
}

void EskfProxy::pubEskfState(const ros::TimerEvent& e)
{
    // pub eskf state
    std::unique_lock<std::mutex> lk(eskf->getLock());

    Qd gq(eskf->R_.matrix());
    auto gp = toGeoVec(eskf->p_);
    lk.unlock();
    // auto gv = toGeoVec(eskf->v_);
    // auto gw = toGeoVec(eskf->w_);

    // Qd grw(eskf->R_W_I);
    // auto grw_euler = toGeoVec(geometry::trans::q2ypr<double>(grw));

    // geometry_msgs::Vector3 hr;
    // hr.x = eskf->h_;
    // hr.y = eskf->r_;
    // hr.z = 0;

    // g_euler.x *= 180.0 / M_PI;
    // g_euler.y *= 180.0 / M_PI;
    // g_euler.z *= 180.0 / M_PI;
    // grw_euler.x *= 180.0 / M_PI;
    // grw_euler.y *= 180.0 / M_PI;
    // grw_euler.z *= 180.0 / M_PI;
    // vis_R.publish(g_euler);
    // vis_p.publish(gp);
    // vis_v.publish(gv);
    // vis_w.publish(gw);
    // vis_RW.publish(grw_euler);
    // vis_hr.publish(hr);

    nav_msgs::Odometry odom;
    odom.header.stamp = e.current_real;
    odom.header.frame_id = "map";
    odom.child_frame_id = "eskf";
    odom.pose.pose.position.x = gp.x;
    odom.pose.pose.position.y = gp.y;
    odom.pose.pose.position.z = gp.z;
    odom.pose.pose.orientation.w = gq.w();
    odom.pose.pose.orientation.x = gq.x();
    odom.pose.pose.orientation.y = gq.y();
    odom.pose.pose.orientation.z = gq.z();
    vis_odom.publish(odom);

    geometry_msgs::PoseStamped psd;
    psd.header.frame_id = "map";
    odom_path.header.stamp = psd.header.stamp = e.current_real;
    psd.pose = odom.pose.pose;
    odom_path.poses.push_back(psd);
    vis_odom_path.publish(odom_path);
}

void EskfProxy::pubGtPath(const nav_msgs::OdometryConstPtr& msg)
{
    static int cnt = 0;
    if(cnt % 10 == 0){
        geometry_msgs::PoseStamped psd;
        psd.header.frame_id = "map";
        gt_path.header.stamp = psd.header.stamp = msg->header.stamp;
        psd.pose = msg->pose.pose;
        gt_path.poses.push_back(psd);
        vis_gt_path.publish(gt_path);
    }
    cnt++;
}

void EskfProxy::onImu(const sensor_msgs::ImuConstPtr &msg)
{
    static double last_time = msg->header.stamp.toSec();

    auto gwm = msg->angular_velocity;
    V3d wm(gwm.x, gwm.y, gwm.z);

    auto gam = msg->linear_acceleration;
    V3d am(gam.x, gam.y, gam.z);

    double now = msg->header.stamp.toSec();

    lg->info("am: {}, {}, {}, now: {}", am(0), am(1), am(2), now);
    eskf->predict(wm, am, now - last_time);
    last_time = now;
    // auto gwm = msg->orientation;
    // V3d wm(gwm.x, gwm.y, gwm.z);
    // eskf->get_imu(wm);
}

void EskfProxy::onWheel(const nav_msgs::OdometryConstPtr &msg)
{
    double alpha = msg->twist.covariance[0];
    double beta = msg->twist.twist.linear.x;
    lg->info("alpha: {}, beta: {}, now: {}", alpha, beta, msg->header.stamp.toSec());
    eskf->get_wheel(alpha, beta, msg->header.stamp.toSec());
}

}
