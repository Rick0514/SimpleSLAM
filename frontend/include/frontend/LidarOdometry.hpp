#pragma once
#include <deque>
#include <atomic>
#include <types/basic.hpp>
#include <frontend/OdometryBase.hpp>

#include <pcl/filters/voxel_grid.h>

// ------------ forward declaration ------------
namespace dataproxy { 
    class LidarDataProxy; 
    class RelocDataProxy;
}

namespace PCR { class PointCloudRegister; }
// ------------ forward declaration ------------

namespace frontend
{

using namespace dataproxy;
using namespace PCLTypes;

class Frontend;
class MapManager;

class LidarOdometry : public OdometryBase
{

public:
    using DataProxyPtr = std::shared_ptr<LidarDataProxy>;
    using FrontendPtr = std::shared_ptr<Frontend>;
    using MapManagerPtr = std::shared_ptr<MapManager>;
    using RelocDataProxyPtr = std::shared_ptr<RelocDataProxy>;

private:

    DataProxyPtr mDataProxyPtr;
    FrontendPtr mFrontendPtr;

    std::unique_ptr<PCR::PointCloudRegister> mPcr; 

    std::atomic_bool reloc;
    std::mutex mRelocLock;
    pose_t mRelocPose;
    V3<scalar_t> mLastPos;
    static constexpr float minKFGap{1.0}; 

    MapManagerPtr mMapManagerPtr;

    pcl::VoxelGrid<pt_t> mVoxelGrid;

public:

    void setRelocFlag(const pose_t& p);
    auto getMapManger() { return mMapManagerPtr; }

    explicit LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, RelocDataProxyPtr& rdp, MapManagerPtr& mmp);

    void selectKeyFrame(const KeyFrame&);

    virtual void generateOdom() override;

    ~LidarOdometry();
};
    
} // namespace frontend

