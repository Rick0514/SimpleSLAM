#include <filter/Eskf.hpp>
#include <geometry/trans.hpp>

#include <sstream>

namespace filter {

State::State(double r_, double h_) : r(r_), h(h_)
{
    reset();
    g << 0, 0, -9.8;
    lg = utils::logger::Logger::getInstance();   
}

void State::reset()
{
    R.setIdentity();
    p.setZero();
    v.setZero();
    w.setZero();
    bg.setZero();
    ba.setZero();
    R_L_I.setIdentity();
    t_L_I.setZero();
    R_W_I.setIdentity();

    // cov need to init maybe
    // Q need to init
    // V need to init
    // rotation matrix numerical issue!!

    // just like fastlio
    cov.setIdentity();
    cov.block<3, 3>(12, 12).diagonal() = V3d(1e-4, 1e-4, 1e-4); 
    cov.block<3, 3>(15, 15).diagonal() = V3d(1e-3, 1e-3, 1e-3);
    for(int i=0; i<9; i++)  cov(18+i, 18+i) = 1e-5; 
    cov(27, 27) = 1e-6;
    cov(28, 28) = 1e-6;

    Q.setZero();
    Q.block<3, 3>(0, 0).diagonal() = V3d(1e-1, 1e-1, 1e-1); 
    Q.block<3, 3>(3, 3).diagonal() = V3d(1e-2, 1e-2, 1e-2);
    for(int i=0; i<6; i++)  Q(6+i, 6+i) = 1e-4; 

    V_IMU.setIdentity();
    V_IMU *= 1e-3;

    V_Wheel.setIdentity();
    V_Wheel *= 1e-2;
    V_Wheel(2, 2) = 1e-3;

    V_Lidar.setZero();
    V_Lidar.block<3, 3>(0, 0).diagonal() = V3d(1e-2, 1e-2, 1e-2);
    V_Lidar.block<3, 3>(3, 3).diagonal() = V3d(1e-3, 1e-3, 1e-3);
}

void State::predict(const V3d& am, double dt)
{
    std::lock_guard<std::mutex> lk(lock);

    M3<double> R_add;
    manifolds::exp<double>(w * dt, R_add);
    p += (v * dt + 0.5 * (R * (am - ba) + g) * dt * dt);
    v += (R * (am - ba) + g) * dt;
    R = trans::rot2q<double>(R * R_add).toRotationMatrix();

    Fx.setIdentity();
    Fw.setZero();

    // dR
    M3<double> R_;
    manifolds::exp<double>(-w * dt, R_);
    Fx.block<3, 3>(0, 0) = R_;
    Fx.block<3, 3>(0, 9) = M3d::Identity() * dt;
    // dp
    Fx.block<3, 3>(3, 6) = M3d::Identity() * dt;
    // dv
    Fx.block<3, 3>(6, 0) = -R * matrix::skewd(am - ba) * dt;
    Fx.block<3, 3>(6, 12) = -R * dt;

    Fw.block<3, 3>(6, 0) = -R * dt;
    Fw.block<9, 9>(9, 3) = Eigen::Matrix<double, 9, 9>::Identity() * dt;

    cov = Fx * cov * Fx.transpose() + Fw * Q * Fw.transpose();
}

void State::boxplus(const ESVec& es){
    M3<double> R_add;
    manifolds::exp<double>(es.block<3, 1>(0, 0), R_add);
    R = trans::rot2q<double>(R * R_add).toRotationMatrix();

    p += es.block<3, 1>(3, 0);
    v += es.block<3, 1>(6, 0);
    w += es.block<3, 1>(9, 0);
    bg += es.block<3, 1>(12, 0);
    ba += es.block<3, 1>(15, 0);

    manifolds::exp<double>(es.block<3, 1>(18, 0), R_add);
    R_L_I = trans::rot2q<double>(R_L_I * R_add).toRotationMatrix();

    t_L_I += es.block<3, 1>(21, 0);

    manifolds::exp<double>(es.block<3, 1>(24, 0), R_add);
    R_W_I = trans::rot2q<double>(R_W_I * R_add).toRotationMatrix();

    r += es(27, 0);
    h += es(28, 0);
}

void State::project(const ESVec& es){
    CovMat J;
    J.setIdentity();
    J.block<3, 3>(0, 0) = manifolds::Jr_SO3_inv(es.block<3, 1>(0, 0)).transpose();
    J.block<3, 3>(18, 18) = manifolds::Jr_SO3_inv(es.block<3, 1>(18, 0)).transpose();
    J.block<3, 3>(24, 24) = manifolds::Jr_SO3_inv(es.block<3, 1>(24, 0)).transpose();

    cov = J * cov * J.transpose();
}

void State::get_imu(const V3d& wm){
    std::lock_guard<std::mutex> lk(lock);

    Eigen::Matrix<double, 3, 29> H;
    H.setZero();
    H.block<3, 3>(0, 9) = M3d::Identity();
    H.block<3, 3>(0, 12) = M3d::Identity();

    Eigen::Matrix<double, 29, 3> K = cov * H.transpose() * (H * cov * H.transpose() + V_IMU).inverse();
    es = K * (wm - w - bg);

    // update
    boxplus(es);
    cov = (CovMat::Identity() - K * H) * cov;
    project(es);
}

// void State::get_wheel(double alpha, double beta)
// {
//     std::lock_guard<std::mutex> lk(lock);

//     // for temp ronc.bag
//     beta /= 0.1;

//     Eigen::Matrix<double, 6, 29> H;
//     H.setZero();

//     double ri_ = 1.0 / r;

//     M3d RW = R_W_I.transpose();
//     V3d Ww = RW * w;
//     V3d Wv = RW * R.transpose() * v;

//     H.block<2, 3>(0, 9) = RW.block<2, 3>(0, 0);
//     H.block<2, 3>(0, 24) = matrix::skewd(Ww).block<2, 3>(0, 0);
    
//     H.block<1, 3>(2, 9) = h * ri_ * RW.row(2);
//     H.block<1, 3>(2, 24) = h * ri_ * matrix::skewd(Ww).row(2);
//     H(2, 27) = -h * ri_ * ri_ * Ww(2); 
//     H(2, 28) = ri_ * Ww(2);

//     H.block<1, 3>(3, 0) = ri_ * (RW * matrix::skewd(R.transpose() * v)).row(0);
//     H.block<1, 3>(3, 6) = ri_ * (RW * R.transpose()).row(0);
//     H.block<1, 3>(3, 24) = ri_ * matrix::skewd(Wv).row(0);
//     H(3, 27) = -ri_ * ri_ * Wv(0);

//     H.block<2, 3>(4, 0) = (RW * matrix::skewd(R.transpose() * v)).block<2, 3>(0, 0);
//     H.block<2, 3>(4, 6) = (RW * R.transpose()).block<2, 3>(0, 0);
//     H.block<2, 3>(4, 24) = matrix::skewd(Wv).block<2, 3>(0, 0);

//     Eigen::Matrix<double, 6, 6> JV;
//     JV.setIdentity();
//     JV(2, 2) = beta / cos(alpha) / cos(alpha);
//     JV(2, 3) = tan(alpha);

//     Eigen::Matrix<double, 29, 6> K = cov * H.transpose() * (H * cov * H.transpose() + JV * V_Wheel * JV.transpose()).inverse();
//     Eigen::Matrix<double, 6, 1> z, hx;
//     z << 0, 0, beta * tan(alpha), beta, 0, 0;
//     hx.head<2>() = Ww.head<2>();
//     hx(2) = h * ri_ * Ww(2);
//     hx(3) = ri_ * Wv(0);
//     hx.tail<2>() = Wv.tail<2>();
//     es = K * (z - hx);
    
//     // lg->info("w: {}, Ww: {}, v: {}, Wv: {}", w(2), Ww(2), v(0), Wv(0));
//     // lg->info("z: ({}, {}), hx: ({}, {})", z(0), z(1), hx(0), hx(1));

//     // {
//     //     std::stringstream ss;
//     //     ss << H;
//     //     lg->info("--- H ---\n{}", ss.str());
//     // }

//     // {
//     //     std::stringstream ss;
//     //     ss << K.transpose();
//     //     lg->info("--- K ---\n{}", ss.str());
//     // }

//     lg->info("Ww: ({}, {}, {}), Wv: ({}, {}, {})", Ww(0), Ww(1), Ww(2), Wv(0), Wv(1), Wv(2));
//     lg->info("z: ({}, {}), hx: ({}, {})", z(2), z(3), hx(2), hx(3));

//     // {
//     //     std::stringstream ss;
//     //     ss << es.transpose();
//     //     lg->info("--- es ---\n{}", ss.str());
//     // }

//     boxplus(es);
//     cov = (CovMat::Identity() - K * H) * cov;
//     project(es);
// }

void State::get_wheel(double alpha, double beta)
{
    std::lock_guard<std::mutex> lk(lock);

    // for temp ronc.bag
    beta /= 0.1;

    Eigen::Matrix<double, 1, 29> H;
    H.setZero();

    double ri_ = 1.0 / r;

    M3d RW = R_W_I.transpose();
    V3d Ww = RW * w;
    V3d Iv = R.transpose() * v;
    V3d Wv = RW * Iv;

    Eigen::Matrix<double, 1, 3> e1;
    e1 << 1, 0, 0;

    H.block<1, 3>(0, 0) = ri_ * e1 * RW * matrix::skewd(Iv);
    H.block<1, 3>(0, 6) = ri_ * e1 * RW * R.transpose();
    // H.block<1, 3>(0, 24) = ri_ * e1 * matrix::skewd(Wv);
    // H(0, 27) = -ri_ * ri_ * Wv(0);

//     Eigen::Matrix<double, 6, 6> JV;
//     JV.setIdentity();
//     JV(2, 2) = beta / cos(alpha) / cos(alpha);
//     JV(2, 3) = tan(alpha);
    Eigen::Matrix<double, 1, 1> V;
    V << 100;
    Eigen::Matrix<double, 29, 1> K = cov * H.transpose() * (H * cov * H.transpose() + V).inverse();
    
    // Eigen::Matrix<double, 6, 1> z, hx;
    // z << 0, 0, beta * tan(alpha), beta, 0, 0;
    // hx.head<2>() = Ww.head<2>();
    // hx(2) = h * ri_ * Ww(2);
    // hx(3) = ri_ * Wv(0);
    // hx.tail<2>() = Wv.tail<2>();

    lg->info("v: ({}, {}, {}), Wv: ({}, {}, {})", v(0), v(1), v(2), Wv(0), Wv(1), Wv(2));
    lg->info("z: {}, hx: {}", beta, ri_ * Wv(0));
    lg->info("r: {}", r);

    {
        std::stringstream ss;
        ss << H;
        lg->info("--- H ---\n{}", ss.str());
    }

    {
        std::stringstream ss;
        ss << K.transpose();
        lg->info("--- K ---\n{}", ss.str());
    }

    {
        std::stringstream ss;
        ss << es.transpose();
        lg->info("--- es ---\n{}", ss.str());
    }

    es = K * (beta - ri_ * Wv(0));
    boxplus(es);
    cov = (CovMat::Identity() - K * H) * cov;
    project(es);
}

void State::get_lidar(const Pose6d& pos)
{
    std::lock_guard<std::mutex> lk(lock);
    Eigen::Matrix<double, 6, 29> H;
    H.setZero();

    V3d v_R = manifolds::log_SO3(R); 
    
    M3d R_I_L = R_L_I.transpose();

    H.block<3, 3>(0, 0) = R_I_L * manifolds::Jr_SO3_inv(v_R);
    H.block<3, 3>(0, 18) = matrix::skewd(R_I_L * v_R);
    
    H.block<3, 3>(3, 0) = -R_I_L * R * matrix::skewd(t_L_I);
    H.block<3, 3>(3, 3) = R_I_L;
    H.block<3, 3>(3, 18) = R_I_L * matrix::skewd(p + (R - M3d::Identity()) * t_L_I);
    H.block<3, 3>(3, 21) = R_I_L * (R - M3d::Identity());

    Eigen::Matrix<double, 29, 6> K = cov * H.transpose() * (H * cov * H.transpose() + V_Lidar).inverse();
    Eigen::Matrix<double, 6, 1> r;
    r.block<3, 1>(0, 0) = manifolds::log_SO3(pos.rotation()) - manifolds::log_SO3(R_I_L * R * R_L_I);
    r.block<3, 1>(3, 0) = pos.translation() - R_I_L * (p + (R - M3d::Identity()) * t_L_I);
    
    es = K * r;
    boxplus(es);
    cov = (CovMat::Identity() - K * H) * cov;
    project(es);
}

Pose6d State::get_lidar_prior()
{
    std::lock_guard<std::mutex> lk(lock);
    Pose6d pri;
    pri.setIdentity();

    pri.matrix().block<3, 3>(0, 0) = R_L_I.transpose() * R * R_L_I;
    pri.matrix().block<3, 1>(0, 3) = R_L_I.transpose() * (p + (R - M3d::Identity()) * t_L_I);

    return pri;        
}

}