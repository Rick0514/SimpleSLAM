#pragma once
#include <geometry/manifolds.hpp>

#include <sophus/types.hpp>
#include <sophus/geometry.hpp>

#include <mutex>
#include <utils/Logger.hpp>

// eskf crash when eigen parallelize !!!!
// maybe not the case!!
// #define EIGEN_DONT_PARALLELIZE

namespace filter {

using namespace geometry;
using namespace EigenTypes;
using namespace Sophus;

using CovMat = Eigen::Matrix<double, 29, 29>;
using QMat = Eigen::Matrix<double, 12, 12>;
using ESVec = Eigen::Matrix<double, 29, 1>;
// error state ==> dim(29 = 3 x 9 + 2) vec

// in imu frame
struct State
{
    SO3d R_;
    V3d p_;
    V3d v_;
    V3d w_;
    V3d bg;
    V3d ba;
    SO3d R_L_I;
    V3d t_L_I;
    SO3d R_W_I;
    double r_, h_;
    V3d g_;

    CovMat cov, Fx;
    Eigen::Matrix<double, 29, 12> Fw;

    // process noise
    QMat Q;

    // meas noise
    M3d V_IMU, V_Wheel;
    Eigen::Matrix<double, 6, 6> V_Lidar;

    // error state
    ESVec es;

    // thread
    std::mutex lock;

    std::shared_ptr<utils::logger::Logger> lg;

    bool imu_init;

    State();

    void predict(const V3d& am, double dt);
    void predict(const V3d& wm, const V3d& am, double dt);

    void boxplus(const ESVec& es);

    void project(const ESVec& es);

    void get_imu(const V3d& wm);

    void get_wheel(double alpha, double beta, double timestamp);

    void get_lidar(const Pose6d& pos, double timestamp);

    Pose6d get_lidar_prior();

    std::mutex& getLock() { return lock; }

};

}