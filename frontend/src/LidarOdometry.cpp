#include <frontend/LidarOdometry.hpp>

#include <frontend/Frontend.hpp>
#include <backend/Backend.hpp>

#include <PCR/LoamRegister.hpp>
#include <PCR/NdtRegister.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <macro/templates.hpp>
#include <utils/Shared_ptr.hpp>

using namespace PCLTypes;

namespace frontend
{

template <typename PointType, bool UseBag>
LidarOdometry<PointType, UseBag>::LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, BackendPtr& bk)
: mDataProxyPtr(dp), mFrontendPtr(ft), mBackendPtr(bk)
{
    // xyz for temp
    mPcr.reset(new PCR::NdtRegister<PointType>());
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
    auto scan = scans->consume_front();

    if(scan){
        // 2. localodom * odom2map
        auto odom2map = mFrontendPtr->get();
        
        // find closest localodom
        // think how to get the stamp
        double stamp;
        Pose6d init_pose;
        init_pose.setIdentity();

        auto local_odom = mFrontendPtr->getClosestLocalOdom(stamp);
        if(local_odom){
            init_pose.matrix() = local_odom->odom.matrix() * odom2map.matrix();
        }else{
            // if not get, use average velocity model
            const auto& gb = mFrontendPtr->getGlobal();
            std::lock_guard<std::mutex> _lk(*gb->getLock());
            auto gbq = gb->getDequeInThreadUnsafeWay();

            auto cidx = Frontend::getClosestItem(gbq, stamp);
            if(cidx > 0 && std::abs(gbq->at(cidx)->stamp - stamp) < 0.1){
                auto last_1 = gbq->at(cidx)->odom;
                auto last_2 = gbq->at(cidx-1)->odom;
                init_pose = last_1 * (last_2.inverse() * last_1);

            }else{
                lg->warn("global odom deque has not enough items to infer average velocity model!!");
            }
        }

        // use pcr to get refined pose
        mPcr->scan2Map(scan, submap, init_pose);

        auto global_odom = std::make_shared<Odometry>(stamp, init_pose);
        mFrontendPtr->getGlobal()->push_back<UseBag>(std::move(global_odom));

        // push the refined odom to deque  
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

