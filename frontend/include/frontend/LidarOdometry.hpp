#pragma once
#include <types/PCLTypes.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <frontend/OdometryBase.hpp>
#include <frontend/Frontend.hpp>
#include <dataproxy/DataProxy.hpp>

// forward declaration
// #include <backend/Backend.hpp>

#include <PCR/LoamRegister.hpp>
#include <PCR/NdtRegister.hpp>

using namespace EigenTypes;
using namespace PCLTypes;

namespace backend
{
    class Backend;
}

namespace frontend
{

using namespace utils;
using namespace dataproxy;
using namespace backend;

template <typename PointType, bool UseBag=false>
class LidarOdometry : public OdometryBase
{

public:
    using DataProxyPtr = std::shared_ptr<DataProxy<PointType, UseBag>>;
    using ConstDataProxyPtr = const std::shared_ptr<DataProxy<PointType, UseBag>>;

    using FrontendPtr = std::shared_ptr<Frontend>;
    using ConstFrontendPtr = const std::shared_ptr<Frontend>;

    using BackendPtr = std::shared_ptr<Backend>;
    using ConstBackendPtr = const std::shared_ptr<Backend>;

private:

    DataProxyPtr mDataProxyPtr;
    FrontendPtr mFrontendPtr;
    BackendPtr mBackendPtr;

    // pointtype xyz for temp
    std::unique_ptr<PCR::PointCloudRegister<PointType>> mPcr; 

public:
    explicit LidarOdometry(ConstDataProxyPtr& dp, ConstFrontendPtr& ft, ConstBackendPtr& bk);

    virtual void generateOdom() override;

    ~LidarOdometry();
};
    
} // namespace frontend

