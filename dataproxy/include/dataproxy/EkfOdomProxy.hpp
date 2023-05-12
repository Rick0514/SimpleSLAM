#pragma once
#include <types/basic.hpp>

#include <dataproxy/DataProxy.hpp>

// ---------- ros ----------
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

namespace dataproxy
{

class EkfOdomProxy : public DataProxy<Odometry>
{
protected:
    ros::Subscriber mEkfSub;
    ros::Subscriber mImuSub;
    ros::Subscriber mWheelSub;

    struct Filter;
    std::unique_ptr<Filter> mFilter;

    bool mUpdateImuFlag;

    template <typename T>
    using NoiseVector = Eigen::Matrix<scalar_t, T::RowsAtCompileTime, 1>;

public:
    explicit EkfOdomProxy(ros::NodeHandle& nh, int size);

    void ekfHandler(const nav_msgs::OdometryConstPtr& msg);

    void imuHandler(const sensor_msgs::ImuConstPtr& msg);
    void wheelHandler(const nav_msgs::OdometryConstPtr& msg);
    // void chassisHandler(const mobile_platform_msgs::ChassisConstPtr& msg);

    ~EkfOdomProxy();
};
    
} // namespace dataproxy




