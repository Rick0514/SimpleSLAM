#include <dataproxy/EkfOdomProxy.hpp>

#include <utils/Math.hpp>
#include <geometry/trans.hpp>

#include <tf2_eigen/tf2_eigen.h>

namespace dataproxy
{

using namespace geometry;

EkfOdomProxy::EkfOdomProxy(ros::NodeHandle& nh, int size) 
    : DataProxy<Odometry>(size),
      mUpdateImuFlag(false)
{
    initFilter();

    mEkfSub = nh.subscribe("/ekf_odom", 50, &EkfOdomProxy::ekfHandler, this);
    mImuSub = nh.subscribe("/imu/data", 100, &EkfOdomProxy::imuHandler, this);
    mWheelSub = nh.subscribe("/odom/raw", 50, &EkfOdomProxy::wheelHandler, this);
}

void EkfOdomProxy::initFilter()
{
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

void EkfOdomProxy::ekfHandler(const nav_msgs::OdometryConstPtr& msg)
{
    auto p = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;

    Pose6<double> pd;
    tf2::fromMsg(msg->pose.pose, pd);

    auto odom = std::make_shared<Odometry>();

    odom->stamp = msg->header.stamp.toSec();
    odom->odom = pd.cast<scalar_t>();
    this->mDataPtr->template push_back<constant::usebag>(std::move(odom));
}

void EkfOdomProxy::imuHandler(const sensor_msgs::ImuConstPtr &msg)
{
    static stamp_t lastTime = -1;
    static sensor_msgs::Imu lastImu;

    auto q = msg->orientation;
    Qt<scalar_t> eq(q.w, q.x, q.y, q.z);

    if(lastTime < 0){
        // init
        lastTime = msg->header.stamp.toSec();
        lastImu = *msg;
        V3<scalar_t> ypr = trans::q2ypr(eq);
        x.yaw() = ypr(0);
        ekf.init(x);
        return;
    }

    if(mUpdateImuFlag){
        // unset immediately
        mUpdateImuFlag = false;

        stamp_t now = msg->header.stamp.toSec();
        auto dt = now - lastTime;

        // get relative meas
        auto lq = lastImu.orientation;
        Qt<scalar_t> elq(lq.w, lq.x, lq.y, lq.z);
        
        // avoid singularity
        Qt<scalar_t> dq = elq.inverse() * eq;
        V3<scalar_t> ypr = trans::q2ypr(dq);

        // update!!
        filter::ImuMeas<scalar_t> ims;
        ims.yaw() = x.yaw() + ypr(0);
        utils::math::correctAngles(ims.yaw(), x.yaw());
        x = ekf.update(imu, ims, dt);
        
        lastTime = now;
        lastImu = *msg;
    }
}

void EkfOdomProxy::wheelHandler(const nav_msgs::OdometryConstPtr &msg)
{
    static stamp_t lastTime = -1;
    static nav_msgs::Odometry lastWheel;

    stamp_t now = msg->header.stamp.toSec();
    
    if(lastTime < 0){
        lastTime = now;
        lastWheel = *msg;
        auto p = msg->pose.pose.position;
        x.x() = p.x;
        x.y() = p.y;
        ekf.init(x);
        return;
    }

    // the pred rate is the same of wheel rate!!
    stamp_t dt = now - lastWheel.header.stamp.toSec();
    x = ekf.predict(sys, dt);
    mUpdateImuFlag = true;

    Pose6<double> pc, pl, po;
    tf2::fromMsg(msg->pose.pose, pc);
    tf2::fromMsg(lastWheel.pose.pose, pl);

    po.setIdentity();
    po.translate(V3d(x.x(), x.y(), 0.0));
    po.rotate(trans::ypr2q(V3d(x.yaw(), 0.0, 0.0)));

    Pose6<double> delta = po * (pl.inverse() * pc);
    filter::WheelMeas<scalar_t> wms;
    wms.x() = delta.translation().x();
    wms.y() = delta.translation().y();
    x = ekf.update(wheel, wms, dt);

    lastTime = now;
    lastWheel = *msg;

    // push to local odom
    Pose6<scalar_t> pd;
    pd.translate(V3<scalar_t>(x.x(), x.y(), 0.0));
    pd.rotate(trans::ypr2q(V3<scalar_t>(x.yaw(), 0.0, 0.0)));
    auto odom = std::make_shared<Odometry>();
    odom->stamp = now;
    odom->odom = pd;
    this->mDataPtr->template push_back<constant::usebag>(std::move(odom));
}

}
