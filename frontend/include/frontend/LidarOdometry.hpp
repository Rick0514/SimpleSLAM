#pragma once
#include <frontend/OdometryBase.hpp>

#include <types/PCLTypes.hpp>

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
    using ConstDataProxyPtr = const std::shared_ptr<DataProxy<PC<PointType>, UseBag>>;

    using FrontendPtr = std::shared_ptr<Frontend>;
    using ConstFrontendPtr = const std::shared_ptr<Frontend>;

    using BackendPtr = std::shared_ptr<Backend<PointType>>;
    using ConstBackendPtr = const std::shared_ptr<Backend<PointType>>;

private:

    DataProxyPtr mDataProxyPtr;
    FrontendPtr mFrontendPtr;
    BackendPtr mBackendPtr;

    std::unique_ptr<PCR::PointCloudRegister<PointType>> mPcr; 

public:
    explicit LidarOdometry(ConstDataProxyPtr& dp, ConstFrontendPtr& ft, ConstBackendPtr& bk);

    virtual void generateOdom() override;

    ~LidarOdometry();
};
    
} // namespace frontend

