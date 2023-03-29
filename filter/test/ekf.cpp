
#include "ros/duration.h"
#include "ros/forwards.h"
#include "ros/timer.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <geometry/trans.hpp>

#include <filter/SystemModel.hpp>
#include <filter/ImuMeasModel.hpp>
#include <filter/WheemMeasModel.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>

using namespace geometry;
using namespace EigenTypes;

class TestEKF
{
private:

    ros::NodeHandle nh;

    ros::Timer pred_timer;   
    float pred_duration{0.01};

    ros::Subscriber imu_sub;
    ros::Subscriber wheel_sub;

    filter::State<float> x;
    filter::SystemModel<float> sys;
    filter::ImuMeasModel<float> imu;
    filter::WheelMeasModel<float> wheel;
    kalman::ExtendedKalmanFilter<filter::State<float>> ekf;

    bool init{false};


public:

    TestEKF(){
        imu_sub = nh.subscribe("/imu/data", 100, &TestEKF::imuHandler, this);
        wheel_sub = nh.subscribe("/odom/raw", 100, &TestEKF::wheelHandler, this);
    
        pred_timer = nh.createTimer(ros::Duration(pred_duration), &TestEKF::predHandler, this);
    }

    void imuHandler(const sensor_msgs::ImuConstPtr& msg)
    {
        if(init){
            auto q = msg->orientation;
            Qf eq(q.w, q.x, q.y, q.z);
            V3f rpy = trans::q2rpy(eq);
            
            filter::ImuMeas<float> ims;
            ims.yaw() = rpy(2);   
            x = ekf.update(imu, ims);
        }
    }

    void wheelHandler(const nav_msgs::OdometryConstPtr& msg)
    {
        if(init){
            filter::WheelMeas<float> wms;
            wms.x() = msg->pose.pose.position.x;
            wms.y() = msg->pose.pose.position.y;
            x = ekf.updata(wheel, wms);
        }        
    }

    void predHandler(const ros::TimerEvent& e)
    {
        if(init)
        {
            x = ekf.predict(sys);
        }
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_ekf");

    return 0;
}

