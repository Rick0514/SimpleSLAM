#include <frontend/LidarOdometry.hpp>

#include <frontend/Frontend.hpp>
#include <frontend/MapManager.hpp>

#include <dataproxy/LidarDataProxy.hpp>
#include <dataproxy/RelocDataProxy.hpp>

#include <PCR/LoamRegister.hpp>
#include <PCR/NdtRegister.hpp>

#include <pcl/pcl_config.h>

#include <utils/Shared_ptr.hpp>
#include <time/tictoc.hpp>
#include <geometry/trans.hpp>

using namespace PCLTypes;
using namespace EigenTypes;

namespace frontend
{
LidarOdometry::LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, RelocDataProxyPtr& rdp, MapManagerPtr& mmp)
: mDataProxyPtr(dp), mFrontendPtr(ft), mMapManagerPtr(mmp), reloc(false)
{
    mLastPos.setZero();
    mRelocPose.setIdentity();
    // xyz for temp
    mPcr.reset(new PCR::LoamRegister);
    // mPcr.reset(new PCR::NdtRegister);

    if(rdp) rdp->registerFunc(std::bind(&LidarOdometry::setRelocFlag, this, std::placeholders::_1));
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

// just check the distant from last one to cur
void LidarOdometry::selectKeyFrame(const KeyFrame& kf)
{
    const V3<scalar_t>& mCurPos = kf.pose.translation();
    if((mCurPos - mLastPos).norm() > minKFGap){
        mMapManagerPtr->putKeyFrame(kf);
        mLastPos = mCurPos;
        lg->info("first select pass, try push kf!!");
    }
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
        auto& odom2map = mFrontendPtr->get();
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
            init_pose.matrix() = odom2map.load().matrix() * local_odom->odom.matrix();
        }else{
            // if not get, use average velocity model
            const auto& gb = mFrontendPtr->getGlobal();
            std::lock_guard<std::mutex> _lk(*gb->getLock());
            auto gbq = gb->getDequeInThreadUnsafeWay();

            auto cidx = Frontend::getClosestItem(gbq, stamp);
            lg->info("cidx: {}", cidx);
            
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

        {
            std::stringstream ss;
            ss << init_pose.translation().transpose();
            lg->info("before pose: {}", ss.str());
        }

        // for now, pure LO, scan2map should be considered always success!!
        // lock here
        {
            std::lock_guard<std::mutex> lk(mMapManagerPtr->getSubmapLock());
            if(!mMapManagerPtr->getKeyFrameObjPtr()->isSubmapEmpty())
            {
                const auto& submap = mMapManagerPtr->getSubmap();
                lg->info("scan pts: {}, submap pts: {}", scan->points.size(), submap->points.size());
                mPcr->scan2Map(scan, submap, init_pose);    
                lg->info("scan2map cost: {:.3f}s", tt);
            }
        }

        {
            std::stringstream ss;
            ss << init_pose.translation().transpose();
            lg->info("pose: {}", ss.str());
        }

        // use scan2map refined pose for current pose for now!! 
        mMapManagerPtr->setCurPose(init_pose);

        KeyFrame kf(scan, init_pose);
        if(mMapManagerPtr->getKeyFrameObjPtr()->isSubmapEmpty())
        {
            lg->warn("at first, no submap here for now, build the map!!");
            mMapManagerPtr->putKeyFrame(kf);
        }else{
            selectKeyFrame(kf);
        }

        // visualize
        mDataProxyPtr.get()->setVisAligned(kf);

        // push the refined odom to deque
        auto global_odom = std::make_shared<Odometry>(stamp, init_pose);
        mFrontendPtr->getGlobal()->template push_back<false>(global_odom);  // false for now!!
        
        // update odom2map
        if(local_odom)  odom2map.store(init_pose * local_odom->odom.inverse());

    }else{
        lg->debug("scan deque is empty for now, please check!!");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

LidarOdometry::~LidarOdometry()
{
    mDataProxyPtr->get()->abort();
}

} // namespace frontend

