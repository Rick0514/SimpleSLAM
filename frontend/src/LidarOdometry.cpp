#include <frontend/LidarOdometry.hpp>

#include <frontend/Frontend.hpp>
#include <backend/Backend.hpp>

#include <PCR/LoamRegister.hpp>
#include <PCR/NdtRegister.hpp>

#include <pcl/pcl_config.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <macro/templates.hpp>
#include <utils/Shared_ptr.hpp>

using namespace PCLTypes;

namespace frontend
{

template <typename PointType, bool UseBag>
LidarOdometry<PointType, UseBag>::LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, BackendPtr& bk)
: mDataProxyPtr(dp), mFrontendPtr(ft), mBackendPtr(bk), reloc(false)
{
    // xyz for temp
    mPcr.reset(new PCR::NdtRegister<PointType>());
}

template <typename PointType, bool UseBag>
void LidarOdometry<PointType, UseBag>::setRelocFlag(EigenTypes::Pose6d& p)
{
    // atomic bool ensure compiler not to reorder exec!! so when reloc is set, mRelocPose is set already
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
    // auto stdscan = scans->consume_front();
    // auto scan = utils::make_shared_ptr(stdscan);    // convert std::shared_ptr to boost::shared_ptr
    typename PC<PointType>::Ptr scan;
    if constexpr (UseBag){

        #if PCL_VERSION_COMPARE(<=, 1, 10, 0)
            auto stdscan = scans->consume_front();
            scan = utils::make_shared_ptr(stdscan);
        #else
            scan = scans->consume_front();
        #endif
    }else{
        // for now, real-time mode use newest scan

        #if PCL_VERSION_COMPARE(<=, 1, 10, 0)   
            auto stdscan = scans->back();
            scan = utils::make_shared_ptr(stdscan);
        #else
            scan = scans->back();
        #endif
    }

    if(scan){
        // 2. localodom * odom2map
        auto odom2map = mFrontendPtr->get();
        // find closest localodom
        // think how to get the stamp
        double stamp = (double)scan->header.stamp / 1e6;
        Pose6d init_pose;
        init_pose.setIdentity();
        auto local_odom = mFrontendPtr->getClosestLocalOdom(stamp);

        // add if reloc
        if(reloc.load()){
            reloc.store(false);
            init_pose = mRelocPose;
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
            if(cidx > 0 && std::abs(gbq->at(cidx)->stamp - stamp) < 0.1){
                auto last_1 = gbq->at(cidx)->odom;
                auto last_2 = gbq->at(cidx-1)->odom;
                init_pose = last_1 * (last_2.inverse() * last_1);

            }else{
                lg->warn("global odom deque has not enough items to infer average velocity model!!");
            }
        }

        // use pcr to get refined pose
        if(!mPcr->scan2Map(scan, submap, init_pose))    lg->warn("scan2map not converge!!");   

        // push the refined odom to dequez
        auto global_odom = std::make_shared<Odometry>(stamp, init_pose);
        mFrontendPtr->getGlobal()->push_back<UseBag>(std::move(global_odom));

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

