#pragma once

#include <geometry/manifolds.hpp>
#include <mutex>
#include <utils/Logger.hpp>

// eskf crash when eigen parallelize !!!!
// maybe not the case!!
// #define EIGEN_DONT_PARALLELIZE

namespace filter {

using namespace geometry;
using namespace EigenTypes;

using CovMat = Eigen::Matrix<double, 29, 29>;
using QMat = Eigen::Matrix<double, 12, 12>;
using ESVec = Eigen::Matrix<double, 29, 1>;
// error state ==> dim(29 = 3 x 9 + 2) vec

// in imu frame
struct State
{
    M3d R;
    V3d p;
    V3d v;
    V3d w;
    V3d bg;
    V3d ba;
    M3d R_L_I;
    V3d t_L_I;
    M3d R_W_I;
    double r, h;
    V3d g;

    CovMat cov, Fx;
    Eigen::Matrix<double, 29, 12> Fw;

    // process noise
    QMat Q;

    // meas noise
    M3d V_IMU;
    Eigen::Matrix<double, 6, 6> V_Wheel, V_Lidar;

    // error state
    ESVec es;

    // thread
    std::mutex lock;

    std::shared_ptr<utils::logger::Logger> lg;

    State(double r_, double h_);

    void reset();

    void predict(const V3d& am, double dt);

    void boxplus(const ESVec& es);

    void project(const ESVec& es);

    void get_imu(const V3d& wm);

    void get_wheel(double alpha, double beta);

    void get_lidar(const Pose6d& pos);

    Pose6d get_lidar_prior();

    std::mutex& getLock() { return lock; }

};

}