#include <frontend/LidarOdometry.hpp>

#include <frontend/Frontend.hpp>
#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

#include <PCR/LoamRegister.hpp>
#include <PCR/NdtRegister.hpp>

#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>
#include <pcp/pcp.hpp>

#include <utils/Shared_ptr.hpp>
#include <time/tictoc.hpp>
#include <geometry/trans.hpp>

using namespace PCLTypes;
using namespace EigenTypes;

namespace frontend
{

LidarOdometry::LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, RelocDataProxyPtr& rdp)
: mDataProxyPtr(dp), mFrontendPtr(ft), reloc(false)
{
    mSubmap = pcl::make_shared<pc_t>();
    mRelocPose.setIdentity();
    // xyz for temp
    mPcr.reset(new PCR::LoamRegister());
    // mPcr.reset(new PCR::NdtRegister<PointType>());
    rdp->registerFunc(std::bind(&LidarOdometry::setRelocFlag, this, std::placeholders::_1));
}

void LidarOdometry::setRelocFlag(const pose_t& p)
{
    // atomic bool ensure compiler not to reorder exec!! so when reloc is set, mRelocPose is set already
    // but it is not safe for fast scenarios. if mRelocPose is call faster than generateOdom, generateOdom
    // will always get true flag, but mRelocPose is reading while be writing

    // now use lock, safe now
    std::lock_guard<std::mutex> lk(mRelocLock);
    mRelocPose = p;
    reloc.store(true);
}

void LidarOdometry::initSubmapFromPCD(std::string pcd_file)
{
    // load global map mode
    if(pcl::io::loadPCDFile<pc_t>(pcd_file, *mSubmap) == -1)
    {
        auto msg = fmt::format("can't load globalmap from: {}", pcd_file);
        lg->error(msg);
        throw std::runtime_error(msg);
    }

    lg->info("load map success!!");

    // downsample global pc
    pcp::voxelDownSample<pc_t>(mSubmap, 0.7f);
    lg->info("submap size: {}", mSubmap->size());
}

void LidarOdometry::selectKeyFrame(KF&& kf)
{
    // std::lock_guard<std::mutex> lk(mKFlock);
    // keyframes.emplace_back(kf);
    // mKFcv.notify_one();
}

void LidarOdometry::generateOdom()
{
    // get current scan
    auto scans = mDataProxyPtr->get();

    // make init pose
    // 1. get latest scan  
    pc_t::Ptr scan;

#if PCL_VERSION_COMPARE(<=, 1, 10, 0)
    auto stdscan = scans->consume_front();
    scan = utils::make_shared_ptr(stdscan); // convert std::shared_ptr to boost::shared_ptr
#else
    scan = scans->consume_front();
#endif
    
    if(scan){
        // 2. localodom * odom2map
        auto odom2map = mFrontendPtr->get();
        // find closest localodom
        // think how to get the stamp
        double stamp = (double)scan->header.stamp / 1e6;
        pose_t init_pose;

        {
            std::lock_guard<std::mutex> lk(mRelocLock);
            init_pose = mRelocPose;
        }

        lg->info("get one scan, regist pc!");
        auto local_odom = mFrontendPtr->getClosestLocalOdom(stamp);

        // add if reloc
        if(reloc.load()){
            reloc.store(false);
            init_pose = mRelocPose;
            lg->info("reloc-ing...");
            // clear global odom
            mFrontendPtr->getGlobal()->clear();  
        }else if(local_odom){
            init_pose.matrix() = local_odom->odom.matrix() * odom2map.matrix();
        }else{
            // if not get, use average velocity model
            const auto& gb = mFrontendPtr->getGlobal();
            std::lock_guard<std::mutex> _lk(*gb->getLock());
            auto gbq = gb->getDequeInThreadUnsafeWay();

            auto cidx = Frontend::getClosestItem(gbq, stamp);
            // here maybe some bug when dt is larger than 0.1
            if(cidx <= 0){
                lg->warn("global odom deque has not enough items to infer average velocity model!!");
            }else{

                if(std::abs(gbq->at(cidx)->stamp - stamp) > 0.15)
                    lg->warn("closest odom is out-dated!!");
                            
                pose_t last_1 = gbq->at(cidx)->odom;
                pose_t last_2 = gbq->at(cidx-1)->odom;
                init_pose = last_1 * (last_2.inverse() * last_1);
                geometry::trans::T2SE3(init_pose.matrix());
            }
        }

        // use pcr to get refined pose
        common::time::tictoc tt;
        // Pose6d tmp_init_pose = init_pose;
        
        {
            std::stringstream ss;
            ss << "before pose: \n" << init_pose.matrix();
            lg->debug("{}", ss.str());
        }

        // for now, pure LO, scan2map should be considered always success!!
        // lock here
        {
            std::lock_guard<std::mutex> lk(mLockMap);
            lg->debug("scan pts: {}, submap pts: {}", scan->points.size(), mSubmap->points.size());
            mPcr->scan2Map(scan, mSubmap, init_pose);
        }
        lg->info("scan2map cost: {:.3f}s", tt);
        // if(!mPcr->scan2Map(scan, submap, init_pose)){
        //     init_pose = tmp_init_pose;
        //     lg->warn("scan2map not converge!!");   
        // }

        {
            std::stringstream ss;
            ss << "after pose: \n" << init_pose.matrix();
            lg->debug("{}", ss.str());
        }

        // visualize
        mDataProxyPtr.get()->setVisAligned(scan, init_pose);

        // push the refined odom to deque
        auto global_odom = std::make_shared<Odometry>(stamp, init_pose);
        mFrontendPtr->getGlobal()->template push_back<constant::usebag>(global_odom);

        // update odom2map
        if(local_odom)  odom2map = init_pose * local_odom->odom.inverse();

    }else{
        lg->warn("scan deque is empty for now, please check!!");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

LidarOdometry::~LidarOdometry(){}

} // namespace frontend

