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

#include <time/tictoc.hpp>

using namespace geometry;
using namespace EigenTypes;
using namespace common;

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
    bool update_imu{false};

    nav_msgs::Odometry last_wheel;
    sensor_msgs::Imu last_imu;

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
        imuNoise << utils::math::to_rad(0.1);   // assume 0.5deg/s
        imu.setCovariance(imuNoise.array().square().matrix().asDiagonal());

        // odom xy noise
        NoiseVector<filter::WheelMeas<float>> wheelNoise;
        wheelNoise << 0.1, 0.1; // assume 0.1m/s
        wheel.setCovariance(wheelNoise.array().square().matrix().asDiagonal());

        // imu_sub = nh.subscribe("/imu/data", 100, &TestEKF::imuHandler, this);
        // wheel_sub = nh.subscribe("/odom/raw", 100, &TestEKF::wheelHandler, this);
        // pred_timer = nh.createTimer(ros::Duration(pred_duration), &TestEKF::predHandler, this);
    }

    void imuHandler(const sensor_msgs::ImuConstPtr& msg)
    {
        if(!init)
        {
            if(utils::math::absClose(msg->header.stamp.toSec(), last_wheel.header.stamp.toSec(), 0.01))
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
                last_imu = *msg;
            }
            return;
        }
        
        if(update_imu){
            update_imu = false;

            double now = msg->header.stamp.toSec(); 
            double dt = now - imu_last_time;

            // get relative meas
            auto q = msg->orientation;
            auto lq = last_imu.orientation;
            Qf eq(q.w, q.x, q.y, q.z);
            Qf elq(lq.w, lq.x, lq.y, lq.z);
            // avoid singularity
            Qf dq = elq.inverse() * eq;
            V3f ypr = trans::q2ypr(dq);
            // std::cout << ypr.transpose() << std::endl;
            // important!!
            filter::ImuMeas<float> ims;
            ims.yaw() = x.yaw() + ypr(0);
            utils::math::correctAngles(ims.yaw(), x.yaw());
            x = ekf.update(imu, ims, dt);
            
            imu_last_time = now;
            last_imu = *msg;
        }

    }

    void wheelHandler(const nav_msgs::OdometryConstPtr& msg)
    {
        double now = msg->header.stamp.toSec();
        if(init){
            double dt = now - wheel_last_time;

            // get relative meas
            Pose6<float> pcur, plast;
            auto p = msg->pose.pose.position;
            auto q = msg->pose.pose.orientation;
            pcur.setIdentity();
            pcur.translate(V3f(p.x, p.y, p.z));
            pcur.rotate(Qf(q.w, q.x, q.y, q.z));
            p = last_wheel.pose.pose.position;
            q = last_wheel.pose.pose.orientation;
            plast.setIdentity();
            plast.translate(V3f(p.x, p.y, p.z));
            plast.rotate(Qf(q.w, q.x, q.y, q.z));

            Pose6<float> pold;
            pold.setIdentity();
            pold.translate(V3f(x.x(), x.y(), 0.0f));
            pold.rotate(trans::ypr2q(V3f(x.yaw(), 0, 0)));

            Pose6<float> delta = pold * (plast.inverse() * pcur);            
            
            filter::WheelMeas<float> wms;
            wms.x() = delta.translation().x();
            wms.y() = delta.translation().y();
            x = ekf.update(wheel, wms, dt);
        }

        wheel_last_time = now;
        last_wheel = *msg;
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
            update_imu = true;
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
                // break;
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
    time::tictoc tt;
    _.run();
    lg->info("run elapsed {:.3f}s", tt);    

    // ros::spin();

    return 0;
}

