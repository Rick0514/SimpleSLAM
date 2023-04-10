#pragma once
#include <atomic>
#include <frontend/OdometryBase.hpp>

#include <types/PCLTypes.hpp>
#include <types/EigenTypes.hpp>

// ------------ forward declaration ------------
namespace frontend { class Frontend; }
namespace backend { template<typename PointType> class Backend; }
namespace dataproxy { template <typename T, bool> class DataProxy; }
namespace PCR { template<typename PointType> class PointCloudRegister; }
// ------------ forward declaration ------------

namespace frontend
{

using namespace dataproxy;
using namespace backend;
using namespace PCLTypes;

class Frontend;

template <typename PointType, bool UseBag=false>
class LidarOdometry : public OdometryBase
{

public:
    using DataProxyPtr = std::shared_ptr<DataProxy<PC<PointType>, UseBag>>;
    using FrontendPtr = std::shared_ptr<Frontend>;
    using BackendPtr = std::shared_ptr<Backend<PointType>>;

private:

    DataProxyPtr mDataProxyPtr;
    FrontendPtr mFrontendPtr;
    BackendPtr mBackendPtr;

    std::unique_ptr<PCR::PointCloudRegister<PointType>> mPcr; 

    std::atomic_bool reloc;
    std::mutex mRelocLock;
    EigenTypes::Pose6d mRelocPose;

public:

    void setRelocFlag(EigenTypes::Pose6d& p);

    explicit LidarOdometry(DataProxyPtr& dp, FrontendPtr& ft, BackendPtr& bk);

    virtual void generateOdom() override;

    ~LidarOdometry();
};
    
} // namespace frontend

