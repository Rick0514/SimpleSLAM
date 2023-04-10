#include <frontend/LidarOdometry.hpp>

#include <frontend/Frontend.hpp>
#include <backend/Backend.hpp>
#include <dataproxy/LidarDataProxy.hpp>

#include <PCR/LoamRegister.hpp>
#include <PCR/NdtRegister.hpp>

#include <pcl/pcl_config.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <macro/templates.hpp>
#include <utils/Shared_ptr.hpp>
#include <time/tictoc.hpp>

using namespace PCLTypes;

namespace frontend
{

template <typename PointType, bool UseBag>
LidarOdometry<PointType, UseBag>::LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, BackendPtr& bk)
: mDataProxyPtr(dp), mFrontendPtr(ft), mBackendPtr(bk), reloc(false)
{
    mRelocPose.setIdentity();
    // xyz for temp
    mPcr.reset(new PCR::LoamRegister<PointType>());
}

template <typename PointType, bool UseBag>
void LidarOdometry<PointType, UseBag>::setRelocFlag(EigenTypes::Pose6d& p)
{
    // atomic bool ensure compiler not to reorder exec!! so when reloc is set, mRelocPose is set already
    // but it is not safe for fast scenarios. if mRelocPose is call faster than generateOdom, generateOdom
    // will always get true flag, but mRelocPose is reading while be writing

    // now use lock, safe now
    std::lock_guard<std::mutex> lk(mRelocLock);
    mRelocPose = p;
    reloc.store(true);
}

template <typename PointType, bool UseBag>
void LidarOdometry<PointType, UseBag>::generateOdom()
{
    // get current scan
    auto scans = mDataProxyPtr->get();
    
    // get submap from backend -- may be need mutex !!
    const auto& submap = mBackendPtr->getSubMap();

    // make init pose
    // 1. get latest scan  
    typename PC<PointType>::Ptr scan;

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
        Pose6d init_pose;

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
                            
                auto last_1 = gbq->at(cidx)->odom;
                auto last_2 = gbq->at(cidx-1)->odom;
                init_pose = last_1 * (last_2.inverse() * last_1);
            }
        }

        // use pcr to get refined pose
        common::time::tictoc tt;
        Pose6d tmp_init_pose = init_pose;
        if(!mPcr->scan2Map(scan, submap, init_pose)){
            init_pose = tmp_init_pose;
            lg->warn("scan2map not converge!!");   
        }
        lg->info("scan2map cost: {:.3f}s", tt);

        // visualize
        auto ldp = static_cast<LidarDataProxy<PC<PointType>, UseBag>*>(mDataProxyPtr.get());
        ldp->setVisAligned(scan, init_pose);
        std::stringstream ss;
        ss << "pose: \n" << init_pose.matrix();
        lg->info("{}", ss.str());

        // push the refined odom to dequez
        auto global_odom = std::make_shared<Odometry>(stamp, init_pose);
        mFrontendPtr->getGlobal()->push_back<UseBag>(global_odom);

        // update odom2map
        if(local_odom)  odom2map = init_pose * local_odom->odom.inverse();

    }else{
        lg->warn("scan deque is empty for now, please check!!");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

template <typename PointType, bool UseBag>
LidarOdometry<PointType, UseBag>::~LidarOdometry(){}

PCTemplateInstantiateExplicitly(LidarOdometry)
PCTemplateInstantiateExplicitlyWithFixedType(LidarOdometry, true)

} // namespace frontend

