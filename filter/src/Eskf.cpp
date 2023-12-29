#include <filter/Eskf.hpp>
#include <geometry/trans.hpp>

#include <sstream>
#include <config/params.hpp>

#include <fstream>

namespace filter {

using std::endl;
using std::vector;
using std::ofstream;

ofstream f_hr;
ofstream f_RLI;
ofstream f_RWI;
ofstream f_tLI;
ofstream f_bgba;

const std::string log_dir = "/root/ws/src/SimpleSLAM/test/data/log/";

State::State() : imu_init(false)
{
    f_hr.open(log_dir + "hr.txt");
    f_RLI.open(log_dir + "RLI.txt");
    f_RWI.open(log_dir + "RWI.txt");
    f_tLI.open(log_dir + "tLI.txt");
    f_bgba.open(log_dir + "bgba.txt");

    lg = utils::logger::Logger::getInstance();   

    auto args = config::Params::getInstance()["eskf"];
    auto vg = args["grav"].get<vector<float>>();
    for(int i=0; i<3; i++)  g_(i) = vg[i];

    r_ = args["r"].get<double>();
    h_ = args["h"].get<double>();

    R_ = SO3d(M3d::Identity());
    p_.setZero();
    v_.setZero();
    w_.setZero();   // remove
    bg.setZero();
    ba.setZero();

    auto ve_RLI = args["RLI"].get<vector<float>>();
    auto ve_RWI = args["RWI"].get<vector<float>>();
    auto ve_tLI = args["tLI"].get<vector<float>>();

    V3d ypr_RLI(ve_RLI[2], ve_RLI[1], ve_RLI[0]);
    ypr_RLI *= M_PI / 180;
    V3d ypr_RWI(ve_RWI[2], ve_RWI[1], ve_RWI[0]);
    ypr_RWI *= M_PI / 180;
    R_L_I = SO3d(trans::ypr2q(ypr_RLI));
    R_W_I = SO3d(trans::ypr2q(ypr_RWI));
    
    for(int i=0; i<3; i++)  t_L_I(i) = ve_tLI[i];

    // cov need to init maybe
    // Q need to init
    // V need to init
    // rotation matrix numerical issue!!

    // just like fastlio
    cov.setIdentity();
    cov.block<3, 3>(12, 12).diagonal() = V3d(1e-4, 1e-4, 1e-4); 
    cov.block<3, 3>(15, 15).diagonal() = V3d(1e-4, 1e-4, 1e-4);
    for(int i=0; i<3; i++)  cov(18+i, 18+i) = 1e-5; 
    for(int i=0; i<3; i++)  cov(21+i, 21+i) = 1e-4; 
    for(int i=0; i<3; i++)  cov(24+i, 24+i) = 1e-5; 
    cov(27, 27) = 1e-5;
    cov(28, 28) = 1e-5;

    auto vQ = args["Q"].get<vector<float>>();
    Q.setZero();
    for(int i=0; i<3; i++)  Q(i, i) = vQ[0];
    for(int i=0; i<3; i++)  Q(3+i, 3+i) = vQ[1];
    for(int i=0; i<3; i++)  Q(6+i, 6+i) = vQ[2]; 
    for(int i=0; i<3; i++)  Q(9+i, 9+i) = vQ[3]; 

    V_IMU.setZero();
    auto vec_v_imu = args["v_imu"].get<vector<float>>();
    for(int i=0; i<3; i++)  V_IMU(i, i) = vec_v_imu[i];

    V_Wheel.setZero();
    auto vec_v_wheel = args["v_wheel"].get<vector<float>>();
    for(int i=0; i<3; i++)  V_Wheel(i, i) = vec_v_wheel[i];

    V_Lidar.setZero();
    auto vec_v_lidar = args["v_lidar"].get<vector<float>>();
    for(int i=0; i<6; i++)  V_Lidar(i, i) = vec_v_lidar[i];

}

void State::predict(const V3d& am, double dt)
{
    static int imu_cnt = 0;
    std::lock_guard<std::mutex> lk(lock);
    
    if(!imu_init && imu_cnt >= 10)   imu_init = true;
    imu_cnt++;

    p_ += (v_ * dt + 0.5 * (R_ * (am - ba) + g_) * dt * dt);
    v_ += (R_ * (am - ba) + g_) * dt;
    R_ = R_ * SO3d::exp(w_ * dt);

    Fx.setIdentity();
    Fw.setZero();

    // dR
    // M3<double> R_;
    // manifolds::exp<double>(-w_ * dt, R_);
    // Fx.block<3, 3>(0, 0) = R_;
    // Fx.block<3, 3>(0, 9) = M3d::Identity() * dt;
    // // dp
    // Fx.block<3, 3>(3, 6) = M3d::Identity() * dt;
    // // dv
    // Fx.block<3, 3>(6, 0) = -R_ * matrix::skewd(am - ba) * dt;
    // Fx.block<3, 3>(6, 12) = -R_ * dt;

    // Fw.block<3, 3>(6, 0) = -R_ * dt;
    // Fw.block<9, 9>(9, 3) = Eigen::Matrix<double, 9, 9>::Identity() * dt;

    // cov = Fx * cov * Fx.transpose() + Fw * Q * Fw.transpose();
}

void State::predict(const V3d& wm, const V3d& am, double dt)
{
    static int imu_cnt = 0;
    std::lock_guard<std::mutex> lk(lock);
    
    if(!imu_init && imu_cnt >= 10)   imu_init = true;
    imu_cnt++;

    w_ = wm - bg;

    p_ += (v_ * dt + 0.5 * (R_ * (am - ba) + g_) * dt * dt);
    v_ += (R_ * (am - ba) + g_) * dt;
    R_ = R_ * SO3d::exp(w_ * dt);

    Fx.setIdentity();
    Fw.setZero();

    // dR
    Fx.block<3, 3>(0, 0) = SO3d::exp(-(wm - bg) * dt).matrix();
    Fx.block<3, 3>(0, 12) = -SO3d::leftJacobianInverse(-(wm - bg) * dt) * dt;
    // dp
    Fx.block<3, 3>(3, 6) = M3d::Identity() * dt;
    // dv
    Fx.block<3, 3>(6, 0) = -R_.matrix() * matrix::skewd(am - ba) * dt;
    Fx.block<3, 3>(6, 12) = -R_.matrix() * dt;

    // nw na bg ba
    Fw.block<3, 3>(0, 0) = -SO3d::leftJacobianInverse(-(wm - bg) * dt) * dt;
    Fw.block<3, 3>(6, 3) = -R_.matrix() * dt;
    Fw.block<6, 6>(12, 6) = Eigen::Matrix<double, 6, 6>::Identity() * dt;

    cov = Fx * cov * Fx.transpose() + Fw * Q * Fw.transpose();
}

void State::boxplus(const ESVec& es){

    R_ = R_ * SO3d::exp(es.block<3, 1>(0, 0));
    
    p_ += es.block<3, 1>(3, 0);
    v_ += es.block<3, 1>(6, 0);
    w_ += es.block<3, 1>(9, 0);
    bg += es.block<3, 1>(12, 0);
    ba += es.block<3, 1>(15, 0);

    R_L_I = R_L_I * SO3d::exp(es.block<3, 1>(18, 0));

    t_L_I += es.block<3, 1>(21, 0);

    R_W_I = R_W_I * SO3d::exp(es.block<3, 1>(24, 0));

    r_ += es(27, 0);
    h_ += es(28, 0);
}

void State::project(const ESVec& es){
    CovMat J;
    J.setIdentity();
    J.block<3, 3>(0, 0) = SO3d::leftJacobianInverse(es.block<3, 1>(0, 0));
    J.block<3, 3>(18, 18) = SO3d::leftJacobianInverse(es.block<3, 1>(18, 0));
    J.block<3, 3>(24, 24) = SO3d::leftJacobianInverse(es.block<3, 1>(24, 0));

    cov = J * cov * J.transpose();
}

void State::get_imu(const V3d& wm){
    std::lock_guard<std::mutex> lk(lock);

    Eigen::Matrix<double, 3, 29> H;
    H.setZero();
    H.block<3, 3>(0, 9) = M3d::Identity();
    H.block<3, 3>(0, 12) = M3d::Identity();

    Eigen::Matrix<double, 29, 3> K = cov * H.transpose() * (H * cov * H.transpose() + V_IMU).inverse();
    es = K * (wm - w_ - bg);

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
//     V3d Ww = RW * w_;
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
    
//     // lg->info("w_: {}, Ww: {}, v: {}, Wv: {}", w_(2), Ww(2), v(0), Wv(0));
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

void State::get_wheel(double alpha, double beta, double timestamp)
{
    std::lock_guard<std::mutex> lk(lock);
 
    if(!imu_init)   return;

    Eigen::Matrix<double, 3, 29> H;
    H.setZero();

    double ri = 1.0 / r_;
    SO3d RW = R_W_I.inverse();
    V3d Iv = R_.inverse() * v_;
    V3d Ww = RW * w_;
    V3d Wv = RW * Iv;

    H.block<3, 3>(0, 0) = RW.matrix() * matrix::skewd(Iv);
    H.block<3, 3>(0, 6) = ri * (RW * R_.inverse()).matrix();
    H.block<3, 3>(0, 24) = ri * matrix::skewd(Wv);
    H.block<3, 1>(0, 27) = -ri * ri * Wv;

    Eigen::Matrix<double, 29, 3> K = cov * H.transpose() * (H * cov * H.transpose() + V_Wheel).inverse();
    
    V3d z(beta, 0, 0);
    es = K * (z - ri * Wv);
    boxplus(es);
    cov = (CovMat::Identity() - K * H) * cov;
    project(es);

    if(fabs(alpha) < 0.4 && fabs(alpha) > 0.1 && beta * r_ > 0.1){
        h_ = beta * r_ * tan(alpha) / Ww(2);
    }

    f_hr << timestamp << " " << h_ << " " << r_ << endl;
    f_RWI << timestamp << " "
          << R_W_I.angleX() * 180 / M_PI << " "
          << R_W_I.angleY() * 180 / M_PI << " "
          << R_W_I.angleZ() * 180 / M_PI << " " << endl;
}

void State::get_lidar(const Pose6d& pos, double timestamp)
{
    std::lock_guard<std::mutex> lk(lock);

    if(!imu_init)   return;
 
    Eigen::Matrix<double, 6, 29> H;
    H.setZero();

    V3d v_R = R_.log(); 
    
    SO3d R_I_L = R_L_I.inverse();

    H.block<3, 3>(0, 0) = R_I_L.matrix() * SO3d::leftJacobianInverse(-v_R);
    H.block<3, 3>(0, 18) = matrix::skewd(R_I_L * v_R);
    
    H.block<3, 3>(3, 0) = -(R_I_L * R_).matrix() * matrix::skewd(t_L_I);
    H.block<3, 3>(3, 3) = R_I_L.matrix();
    H.block<3, 3>(3, 18) = R_I_L.matrix() * matrix::skewd(p_ + R_ * t_L_I - t_L_I);
    H.block<3, 3>(3, 21) = (R_I_L * R_).matrix() - R_I_L.matrix();

    Eigen::Matrix<double, 29, 6> K = cov * H.transpose() * (H * cov * H.transpose() + V_Lidar).inverse();
    Eigen::Matrix<double, 6, 1> res;
    res.head<3>() = SO3d::fitToSO3(pos.rotation()).log() - (R_I_L * R_ * R_L_I).log();
    res.tail<3>() = pos.translation() - R_I_L * (p_ + R_ * t_L_I - t_L_I);
    // res = pos.translation() - R_I_L * (p_ + (R_ - M3d::Identity()) * t_L_I);

    es = K * res;
    boxplus(es);
    cov = (CovMat::Identity() - K * H) * cov.eval();
    project(es);

    f_tLI << timestamp << " " << t_L_I.transpose() << endl;
    f_RLI << timestamp << " "
          << R_L_I.angleX() * 180 / M_PI << " " 
          << R_L_I.angleY() * 180 / M_PI << " "
          << R_L_I.angleZ() * 180 / M_PI << " " << endl;

    f_bgba << timestamp << " " << bg.transpose() << " " << ba.transpose() << endl;
}

Pose6d State::get_lidar_prior()
{
    std::lock_guard<std::mutex> lk(lock);
    Pose6d pri;
    pri.setIdentity();

    pri.matrix().block<3, 3>(0, 0) = (R_L_I.inverse() * R_ * R_L_I).matrix();
    pri.matrix().block<3, 1>(0, 3) = R_L_I.inverse() * (p_ + R_ * t_L_I - t_L_I);

    return pri;        
}

}