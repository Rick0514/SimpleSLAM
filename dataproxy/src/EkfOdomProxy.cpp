// ---------- ros ----------
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <dataproxy/EkfOdomProxy.hpp>

#include <utils/Math.hpp>
#include <geometry/trans.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <config/params.hpp>

// ---------- filter -----------
#include <filter/SystemModel.hpp>
#include <filter/ImuMeasModel.hpp>
#include <filter/WheelMeasModel.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>

namespace dataproxy
{
using namespace filter;
using namespace geometry;

class EkfOdomProxy::Ros
{
public:
    ros::Subscriber mEkfSub;
    ros::Subscriber mImuSub;
    ros::Subscriber mWheelSub;

    EkfOdomProxy* eop;

    Ros(ros::NodeHandle& nh);
    // void ekfHandler(const nav_msgs::OdometryConstPtr& msg);
    void onImu(const sensor_msgs::ImuConstPtr& msg);
    void onWheel(const nav_msgs::OdometryConstPtr& msg);
    // void chassisHandler(const mobile_platform_msgs::ChassisConstPtr& msg);
};

class EkfOdomProxy::Filter
{
private:

    SystemModel<scalar_t> sys;
    ImuMeasModel<scalar_t> imu;
    WheelMeasModel<scalar_t> wheel;
    Kalman::ExtendedKalmanFilter<State<scalar_t>> ekf;

public:
    State<scalar_t> x;

    Filter();
    void initX() { ekf.init(x); }
    void predict(double dt) { x = ekf.predict(sys, dt); }
    void updateImu(const filter::ImuMeas<scalar_t>& ims, double dt) { x = ekf.update(imu, ims, dt); }
    void updateWheel(const filter::WheelMeas<scalar_t>& wms, double dt) { x = ekf.update(wheel, wms, dt); }
};

EkfOdomProxy::Ros::Ros(ros::NodeHandle& nh)
{
    auto params = config::Params::getInstance()["dataproxy"];

    std::string imu_topic = params["imu"];
    std::string wheel_topic = params["wheel"];

    // mEkfSub = nh.subscribe("/ekf_odom", 50, &EkfOdomProxy::ekfHandler, this);
    mImuSub = nh.subscribe(imu_topic, 100, &Ros::onImu, this);
    mWheelSub = nh.subscribe(wheel_topic, 50, &Ros::onWheel, this);    
}

EkfOdomProxy::Filter::Filter()
{
    x.setZero();

    // prior noise
    NoiseVector<filter::State<float>> priorNoise;
    priorNoise << 1e-4, 1e-4, 1e-4;     // assume prior noise is very small
    ekf.setCovariance(priorNoise.array().square().matrix().asDiagonal());

    // sys noise
    NoiseVector<filter::State<float>> sysNoise;
    sysNoise << 1.0, 1.0, utils::math::to_rad(5.0);     // assume system model has relatively large noise
    sys.setCovariance(sysNoise.array().square().matrix().asDiagonal());

    // imu yaw noise
    NoiseVector<filter::ImuMeas<float>> imuNoise;
    imuNoise << utils::math::to_rad(0.1);   // assume 0.5deg/s
    imu.setCovariance(imuNoise.array().square().matrix().asDiagonal());

    // odom xy noise
    NoiseVector<filter::WheelMeas<float>> wheelNoise;
    wheelNoise << 0.1, 0.1; // assume 0.1m/s
    wheel.setCovariance(wheelNoise.array().square().matrix().asDiagonal());
}

EkfOdomProxy::EkfOdomProxy(ros::NodeHandle& nh, int size) 
    : mUpdateImuFlag(false), DataProxy<Odometry>(size), mRosImpl(std::make_unique<Ros>(nh)),
    mFilterImpl(std::make_unique<Filter>())
{
    mRosImpl->eop = this;
}

// void EkfOdomProxy::Ros::ekfHandler(const nav_msgs::OdometryConstPtr& msg)
// {
//     auto p = msg->pose.pose.position;
//     auto q = msg->pose.pose.orientation;

//     Pose6<double> pd;
//     tf2::fromMsg(msg->pose.pose, pd);

//     auto odom = std::make_shared<Odometry>();

//     odom->stamp = msg->header.stamp.toSec();
//     odom->odom = pd.cast<scalar_t>();
//     eop->mDataPtr->template push_back<constant::usebag>(std::move(odom));
// }

void EkfOdomProxy::Ros::onImu(const sensor_msgs::ImuConstPtr &msg)
{
    auto q = msg->orientation;
    Qt<scalar_t> eq(q.w, q.x, q.y, q.z);

    Odom o;
    o.q = eq;
    o.stamp = msg->header.stamp.toSec();
    eop->imuHandler(o);
}

void EkfOdomProxy::imuHandler(const Odom& msg)
{
    static stamp_t lastTime = -1;
    static Odom lastImu;

    State<scalar_t>& x = mFilterImpl->x;

    if(lastTime < 0){
        // init
        lastTime = msg.stamp;
        lastImu = msg;
        V3<scalar_t> ypr = trans::q2ypr(msg.q);

        x.yaw() = ypr(0);
        mFilterImpl->initX();
        mLg->info("imu init x done: ({}, {}, {})", x.x(), x.y(), x.yaw());
        return;
    }

    if(mUpdateImuFlag){
        // unset immediately
        mUpdateImuFlag = false;

        stamp_t now = msg.stamp;
        auto dt = now - lastTime;

        // get relative meas, avoid singularity
        Qt<scalar_t> dq = lastImu.q.inverse() * msg.q;
        V3<scalar_t> ypr = trans::q2ypr(dq);

        // update!!
        filter::ImuMeas<scalar_t> ims;

        ims.yaw() = x.yaw() + ypr(0);
        utils::math::correctAngles(ims.yaw(), x.yaw());
        mFilterImpl->updateImu(ims, dt);
        
        lastTime = now;
        lastImu = msg;
    }
}

void EkfOdomProxy::Ros::onWheel(const nav_msgs::OdometryConstPtr &msg)
{
    auto p = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;
    Qt<scalar_t> eq(q.w, q.x, q.y, q.z);

    Odom o;
    o.stamp = msg->header.stamp.toSec();
    o.t << p.x, p.y, p.z;
    o.q = eq;
    eop->wheelHandler(o);
}

void EkfOdomProxy::wheelHandler(const Odom& msg)
{
    static stamp_t lastTime = -1;
    static Odom lastWheel;

    stamp_t now = msg.stamp;
    
    State<scalar_t>& x = mFilterImpl->x;

    if(lastTime < 0){
        lastTime = now;
        lastWheel = msg;
        
        x.x() = msg.t(0);
        x.y() = msg.t(1);
        mFilterImpl->initX();

        mLg->info("wheel init x done: ({}, {}, {})", x.x(), x.y(), x.yaw());
        return;
    }

    // the pred rate is the same of wheel rate!!
    stamp_t dt = now - lastWheel.stamp;
    mFilterImpl->predict(dt);
    mUpdateImuFlag = true;

    Pose6<double> pc, pl, po;
    pc.setIdentity();
    pc.translate(msg.t);
    pc.rotate(msg.q);

    pl.setIdentity();
    pl.translate(lastWheel.t);
    pl.rotate(lastWheel.q);

    po.setIdentity();
    po.translate(V3d(x.x(), x.y(), 0.0));
    po.rotate(trans::ypr2q(V3d(x.yaw(), 0.0, 0.0)));

    Pose6<double> delta = po * (pl.inverse() * pc);
    filter::WheelMeas<scalar_t> wms;
    wms.x() = delta.translation().x();
    wms.y() = delta.translation().y();
    mFilterImpl->updateWheel(wms, dt);

    lastTime = now;
    lastWheel = msg;

    // push to local odom
    Pose6<scalar_t> pd;
    pd.setIdentity();
    pd.translate(V3<scalar_t>(x.x(), x.y(), 0.0));
    pd.rotate(trans::ypr2q(V3<scalar_t>(x.yaw(), 0.0, 0.0)));
    auto odom = std::make_shared<Odometry>();
    odom->stamp = now;
    odom->odom = pd;

    std::stringstream ss;
    ss << pd.translation().transpose();
    // mLg->debug("push trans: {}", ss.str());

    mDataPtr->template push_back<constant::usebag>(std::move(odom));
    // mLg->debug("ekf size: {}", mDataPtr->size());
}

EkfOdomProxy::~EkfOdomProxy() = default;

}
