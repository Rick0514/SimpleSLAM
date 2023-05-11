#include <geometry/trans.hpp>

#include <backend/Backend.hpp>
#include <backend/LoopClosureManager.hpp>
#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <config/params.hpp>
#include <utils/File.hpp>

namespace backend
{

using namespace gtsam;
using KFEvent = frontend::KeyFramesObj::Event;

Backend::Backend(const FrontendPtr& ft, const MapManagerPtr& mp) : mRunning(true), mFrontendPtr(ft), mMapManagerPtr(mp)
{
    lg = logger::Logger::getInstance();
    lg->info("backend is constructing!!");

    auto cfg = config::Params::getInstance();
    mSaveMapDir = cfg["saveMapDir"];

    // noise --> gtsam use double
    Eigen::Matrix<double, 6, 1> priorNoise, odomNoise, lcNoise;

    priorNoise << 1e-2, 1e-2, M_PI / 72, 1e-1, 1e-1, 1e-1;
    odomNoise << 1e-4, 1e-4, 1e-4, 1e-1, 1e-1, 1e-1;
    lcNoise << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
    
    gtOdomNoise = noiseModel::Diagonal::Variances(odomNoise);
    gtPriorNoise = noiseModel::Diagonal::Variances(priorNoise);
    gtLcNoise = noiseModel::Diagonal::Variances(lcNoise);

    // get keyframe obj to optimize
    mKFObjPtr = mMapManagerPtr->getKeyFrameObjPtr();

    gtsam::ISAM2Params param;
    param.relinearizeThreshold = 0.1;
    param.relinearizeSkip = 1;
    isam2 = std::make_unique<gtsam::ISAM2>(param);

    loadFactorGraph();

    mOptimThread = std::make_unique<trd::ResidentThread>(&Backend::optimHandler, this);
}

void Backend::myReadG2o(const std::string& file, gtsam::NonlinearFactorGraph& fg, gtsam::Values& v)
{
    std::ifstream inf(file);
    if(!inf.is_open()){
        auto msg = fmt::format("failed to open g2o file: [{}]!", file);
        lg->error(msg);
        throw std::invalid_argument(msg);
    }

    std::string line;
    index_t id = 0;
    while(std::getline(inf, line))
    {
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;

        if (tag == "VERTEX3") {
            double x, y, z, roll, pitch, yaw;
            ss >> id >> x >> y >> z >> roll >> pitch >> yaw;
            Rot3 R = Rot3::ypr(yaw,pitch,roll);
            Point3 t = Point3(x, y, z);
            v.insert(id++, Pose3(R,t));
        }else if (tag == "VERTEX_SE3:QUAT") {
            double x, y, z, qx, qy, qz, qw;
            ss >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
            Rot3 R = Rot3::quaternion(qw, qx, qy, qz);
            Point3 t = Point3(x, y, z);

            if(id == 0)
                factorGraph.add(PriorFactor<gtsam::Pose3>(0, Pose3(R, t), gtPriorNoise));

            v.insert(id++, Pose3(R,t));
        }else if (tag == "EDGE3") {
            index_t id1, id2;
            double x, y, z, roll, pitch, yaw;
            ss >> id1 >> id2 >> x >> y >> z >> roll >> pitch >> yaw;
            Rot3 R = Rot3::ypr(yaw,pitch,roll);
            Point3 t = Point3(x, y, z);
            Matrix m = eye(6);
            for (int i = 0; i < 6; i++)
                for (int j = i; j < 6; j++)
                    ss >> m(i, j);
            SharedNoiseModel model = noiseModel::Gaussian::Information(m);
            NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(id1, id2, Pose3(R,t), model));
            fg.push_back(factor);
        }else if (tag == "EDGE_SE3:QUAT") {
            Matrix m = eye(6);
            index_t id1, id2;
            double x, y, z, qx, qy, qz, qw;
            ss >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw;
            Rot3 R = Rot3::quaternion(qw, qx, qy, qz);
            Point3 t = Point3(x, y, z);
            for (int i = 0; i < 6; i++){
                for (int j = i; j < 6; j++){
                    double mij;
                    ss >> mij;
                    m(i, j) = mij;
                    m(j, i) = mij;
                }
            }
            Matrix mgtsam = eye(6);
            mgtsam.block(0,0,3,3) = m.block(3,3,3,3); // cov rotation
            mgtsam.block(3,3,3,3) = m.block(0,0,3,3); // cov translation
            mgtsam.block(0,3,3,3) = m.block(0,3,3,3); // off diagonal
            mgtsam.block(3,0,3,3) = m.block(3,0,3,3); // off diagonal
            SharedNoiseModel model = noiseModel::Gaussian::Information(mgtsam);
            NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(id1, id2, Pose3(R,t), model));
            fg.push_back(factor);
        }
    }
}

void Backend::loadFactorGraph()
{
    std::string fn = fmt::format("{}/fg.g2o", mSaveMapDir);
    if(!utils::file::isFileExist(fn)){
        lg->info("no factor graph file, maybe first build or rebuild!!");
        return;
    }

    myReadG2o(fn, factorGraph, initialEstimate);

    recordFactorGraph = factorGraph;
    recordEstimate = initialEstimate;
    // isam2->update(factorGraph, initialEstimate);
    // isam2->update();
    // factorGraph.resize(0);
    // initialEstimate.clear();

    lg->info("load factor graph done!");
}

void Backend::addOdomFactor()
{
    auto n = mKFObjPtr->mKFNums;
    auto& keyframes = mKFObjPtr->keyframes;

    if(n == 0){
        
        // turn to 2d
        pose_t kfp = keyframes.front().pose;
        // kfp = geometry::trans::SixDof2Mobile(kfp);
        auto pose = gtsam::Pose3(kfp.matrix().cast<double>());
        factorGraph.add(PriorFactor<gtsam::Pose3>(0, pose, gtPriorNoise));
        recordFactorGraph.add(PriorFactor<gtsam::Pose3>(0, pose, gtPriorNoise));
        initialEstimate.insert(0, pose);
        n++;
    }

    int cidx = 0;
    for(int i=n; i<keyframes.size(); i++){
        auto from_idx = mKFObjPtr->mClosestKfIdx[cidx++];
        auto from = gtsam::Pose3(keyframes[from_idx].pose.matrix().cast<double>());
        auto to = gtsam::Pose3(keyframes[i].pose.matrix().cast<double>());
        lg->info("factor graph add edge from {} to {}", from_idx, i);
        factorGraph.add(BetweenFactor<gtsam::Pose3>(from_idx, i, from.between(to), gtOdomNoise));
        recordFactorGraph.add(BetweenFactor<gtsam::Pose3>(from_idx, i, from.between(to), gtOdomNoise));
        // insert pose that project to 2d
        // pose_t kf_to = keyframes[i].pose;
        // kf_to = geometry::trans::SixDof2Mobile(kf_to);
        // to = gtsam::Pose3(kf_to.matrix().cast<double>());
        initialEstimate.insert(i, to);
    }
}

void Backend::addLoopFactor()
{
    // consume lcq
    auto& lcq = mLCManagerPtr->getLCQ();
    auto n = lcq.size();

    for(int i=0; i<n; i++){
        auto r = lcq.consume_front();
        auto p = gtsam::Pose3(r->between.matrix().cast<double>());
        factorGraph.add(BetweenFactor<gtsam::Pose3>(r->from, r->to, p, gtLcNoise));
        recordFactorGraph.add(BetweenFactor<gtsam::Pose3>(r->from, r->to, p, gtLcNoise));
    }
}

void Backend::optimHandler()
{
    std::unique_lock<std::mutex> lk(mKFObjPtr->mLockKF);
    mKFObjPtr->mKFcv.wait(lk, [&](){ return (mKFObjPtr->isEventComing() || lg->isProgramExit()); });
    KFEvent kfe = mKFObjPtr->getEvent();
    mKFObjPtr->resetEvent();

    lg->info("backend start to optimize!!");

    if(lg->isProgramExit()){
        lg->info("program is about exit, give up this optim!");
        return;
    }

    if(kfe == KFEvent::NewKFCome){
        // before saveKfs, use raw pc to make context, ds rate depend on loop-closure module
        mLCManagerPtr->addContext();
        mMapManagerPtr->saveKfs();
        // new kf is put
        addOdomFactor();
        mKFObjPtr->mKFNums = mKFObjPtr->keyframes.size();
        mKFObjPtr->mClosestKfIdx.clear();
    }else if(kfe == KFEvent::LC){
        addLoopFactor();        
    }

    lk.unlock();    // ------------------------------------

    //  maybe time consuming
    isam2->update(factorGraph, initialEstimate);
    isam2->update();

    if(kfe == KFEvent::LC)  for(int i=0; i<3; i++)  isam2->update();
    
    factorGraph.resize(0);
    initialEstimate.clear();
    optimizedEstimate = isam2->calculateEstimate();
    recordEstimate = optimizedEstimate;

    lk.lock();     // ------------------------------------  
    
    auto n = mKFObjPtr->mKFNums;
    auto& keyframes = mKFObjPtr->keyframes;
    // save newest kf
    pose_t latest_pose = keyframes.back().pose; 
    // update kfs
    for(int i=0; i<n; i++){
        const auto& p = optimizedEstimate.at<gtsam::Pose3>(i);
        keyframes[i].pose.matrix() = p.matrix().cast<scalar_t>();
    }

    // update frontend, make it se3
    pose_t delta = keyframes.back().pose * latest_pose.inverse();

    lk.unlock();

    {
        std::stringstream ss;
        ss << delta.translation().transpose();
        lg->info("backend delta: {}", ss.str());
    }

    geometry::trans::T2SE3(delta.matrix());

    const auto& gb = mFrontendPtr->getGlobal();

    {
        lg->info("update globalodom queue!!");

        std::lock_guard<std::mutex> _lk(gb->getLock());
        auto gbq = gb->getDequeInThreadUnsafeWay();
        // will pose become non-se3??
        for(auto& e : gbq) e->odom = delta * e->odom;
    }

    // update odom2map
    pose_t p = delta * mFrontendPtr->get().load();
    mFrontendPtr->get().store(p);
}

Backend::~Backend()
{
    mKFObjPtr->mKFcv.notify_all();
    lg->info("exit backend, save kfs and factor graph...");
    mOptimThread->Stop();
    // save tum
    std::lock_guard<std::mutex> lk(mKFObjPtr->mLockKF);
    utils::file::writeAsTum(mSaveMapDir, mKFObjPtr->keyframes);
    gtsam::writeG2o(recordFactorGraph, recordEstimate, fmt::format("{}/fg.g2o", mSaveMapDir));
}

}