#pragma once

#include <ros/ros.h>
#include <types/basic.hpp>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <dataproxy/DataProxy.hpp>

#include <filter/SystemModel.hpp>
#include <filter/ImuMeasModel.hpp>
#include <filter/WheelMeasModel.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>

namespace dataproxy
{
using namespace filter;

class EkfOdomProxy : public DataProxy<Odometry>
{
protected:
    ros::Subscriber mEkfSub;
    ros::Subscriber mImuSub;
    ros::Subscriber mWheelSub;

    State<scalar_t> x;
    SystemModel<scalar_t> sys;
    ImuMeasModel<scalar_t> imu;
    WheelMeasModel<scalar_t> wheel;
    Kalman::ExtendedKalmanFilter<State<scalar_t>> ekf;

    bool mUpdateImuFlag;

    template <typename T>
    using NoiseVector = Eigen::Matrix<scalar_t, T::RowsAtCompileTime, 1>;

public:
    explicit EkfOdomProxy(ros::NodeHandle& nh, int size);

    void initFilter();

    void ekfHandler(const nav_msgs::OdometryConstPtr& msg);

    void imuHandler(const sensor_msgs::ImuConstPtr& msg);
    void wheelHandler(const nav_msgs::OdometryConstPtr& msg);
    // void chassisHandler(const mobile_platform_msgs::ChassisConstPtr& msg);

    ~EkfOdomProxy() {};
};
    
} // namespace dataproxy




