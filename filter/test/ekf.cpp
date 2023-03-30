#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <utils/Logger.hpp>
#include <utils/Math.hpp>
#include <geometry/trans.hpp>

#include <filter/SystemModel.hpp>
#include <filter/ImuMeasModel.hpp>
#include <filter/WheelMeasModel.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <vector>

using namespace geometry;
using namespace EigenTypes;

template<typename T>
using NoiseVector = Eigen::Matrix<typename T::Scalar, T::RowsAtCompileTime, 1>;

class TestEKF
{
private:

    // ros::NodeHandle nh;

    // ros::Timer pred_timer;   
    // float pred_duration{0.02};
    // ros::Subscriber imu_sub;
    // ros::Subscriber wheel_sub;

    filter::State<float> x;
    filter::SystemModel<float> sys;
    filter::ImuMeasModel<float> imu;
    filter::WheelMeasModel<float> wheel;
    Kalman::ExtendedKalmanFilter<filter::State<float>> ekf;

    bool init{false};

    nav_msgs::Odometry last_wheel;

    double pre_last_time;
    double imu_last_time;
    double wheel_last_time;

    std::unique_ptr<std::ofstream> tum;

public:

    TestEKF() : pre_last_time(-1), imu_last_time(-1), wheel_last_time(-1){

    #ifdef DATA_FILE
        tum = std::make_unique<std::ofstream>(DATA_FILE "/mekf.txt");
    #endif
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
        imuNoise << utils::math::to_rad(0.5);   // assume 0.5deg/s
        imu.setCovarianceSquareRoot(imuNoise.array().square().matrix().asDiagonal());

        // odom xy noise
        NoiseVector<filter::WheelMeas<float>> wheelNoise;
        wheelNoise << 0.01, 0.01; // assume 0.1m/s
        wheel.setCovarianceSquareRoot(wheelNoise.array().square().matrix().asDiagonal());

        // imu_sub = nh.subscribe("/imu/data", 100, &TestEKF::imuHandler, this);
        // wheel_sub = nh.subscribe("/odom/raw", 100, &TestEKF::wheelHandler, this);
        // pred_timer = nh.createTimer(ros::Duration(pred_duration), &TestEKF::predHandler, this);
    }

    void imuHandler(const sensor_msgs::ImuConstPtr& msg)
    {
        if(!init && utils::math::absClose(msg->header.stamp.toSec(), last_wheel.header.stamp.toSec(), 0.01))
        {
            x.x() = last_wheel.pose.pose.position.x;
            x.y() = last_wheel.pose.pose.position.y;
            auto q = msg->orientation;
            Qf eq(q.w, q.x, q.y, q.z);
            V3f ypr = trans::q2ypr(eq);
            x.yaw() = ypr(0);
            ekf.init(x);
            init = true;

            pre_last_time = msg->header.stamp.toSec();
            imu_last_time = pre_last_time;
            wheel_last_time = pre_last_time;

            return;
        }
        
        double now = msg->header.stamp.toSec(); 
        double dt = now - imu_last_time;

        auto q = msg->orientation;
        Qf eq(q.w, q.x, q.y, q.z);
        V3f ypr = trans::q2ypr(eq);
        // important!!
        utils::math::correctAngles(ypr(0), x.yaw());

        filter::ImuMeas<float> ims;
        ims.yaw() = ypr(0);
        x = ekf.update(imu, ims, dt);

        imu_last_time = now;
    }

    void wheelHandler(const nav_msgs::OdometryConstPtr& msg)
    {
        if(!init){
            last_wheel = *msg;
            return;
        }
        double now = msg->header.stamp.toSec();
        double dt = now - wheel_last_time;

        filter::WheelMeas<float> wms;
        wms.x() = msg->pose.pose.position.x;
        wms.y() = msg->pose.pose.position.y;
        x = ekf.update(wheel, wms, dt);

        wheel_last_time = now;
    }

    void predHandler(const ros::TimerEvent& e)
    {
        if(init){
            double now = e.current_real.toSec();
            auto q = tf::createQuaternionFromYaw(x.yaw());
            auto tum_str = fmt::format("{} {} {} {} {} {} {} {}", now, x.x(), x.y(), 0, q.x(), q.y(), q.z(), q.w());
            *tum << tum_str << std::endl;

            double dt = now - pre_last_time;
            x = ekf.predict(sys, dt);
            pre_last_time = now;
        }    
    }

    void run()
    {
        auto run_lg = utils::logger::Logger::getInstance();

        rosbag::Bag bag;
        std::string bag_fn;
    #ifdef DATA_FILE
        bag_fn = std::string(DATA_FILE "/arb.bag");
        run_lg->info("get bag file: {}", bag_fn);
    #endif
        bag.open(bag_fn, rosbag::bagmode::Read);
        std::string imu_topic = "/imu/data";
        std::string odom_topic = "/odom/raw";

        std::vector<std::string> topics{imu_topic, odom_topic};
        double start_time = -1;
        for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topics)))
        {
            if(start_time < 0)  start_time = m.getTime().toSec();
            if(m.getTime().toSec() - start_time > 50){
                run_lg->info("50s pass...");
                start_time = m.getTime().toSec();
            }

            if(m.getTopic() == odom_topic){
                nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
                // pred freq equal to wheel
                ros::TimerEvent e;
                e.current_real = odom->header.stamp;
                predHandler(e);
                wheelHandler(odom);
            }else if(m.getTopic() == imu_topic){
                sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
                imuHandler(imu);
            }
        }

        bag.close();
    }

};

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "test_ekf");

    auto lg = utils::logger::Logger::getInstance();
    lg->info("start ekf...");

    TestEKF _;
    _.run();

    // ros::spin();

    return 0;
}

