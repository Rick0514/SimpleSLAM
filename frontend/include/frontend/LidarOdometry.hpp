#pragma once
#include <types/PCLTypes.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <dataproxy/LidarDataProxy.hpp>
#include <frontend/OdometryBase.hpp>
#include <frontend/Frontend.hpp>
#include <backend/Backend.hpp>

#include <PCR/LoamRegister.hpp>
#include <PCR/NdtRegister.hpp>

namespace frontend
{

using namespace EigenTypes;
using namespace PCLTypes;
using namespace utils;
using namespace dataproxy;
using namespace backend;

template <bool UseBag>
class LidarOdometry : public OdometryBase<UseBag>
{

public:
    using DataProxyPtr = std::shared_ptr<DataProxy<PCxyz>>;
    using ConstDataProxyPtr = const std::shared_ptr<DataProxy<PCxyz>>;

    using FrontendPtr = std::shared_ptr<Frontend>;
    using ConstFrontendPtr = const std::shared_ptr<Frontend>;

    using BackendPtr = std::shared_ptr<Backend>;
    using ConstBackendPtr = const std::shared_ptr<Backend>;

private:

    DataProxyPtr mDataProxyPtr;
    FrontendPtr mFrontendPtr;
    BackendPtr mBackendPtr;

    // pointtype xyz for temp
    std::unique_ptr<PCR::PointCloudRegister<pcl::PointXYZ>> mPcr; 

public:
    explicit LidarOdometry(ConstDataProxyPtr& dp, ConstFrontendPtr& ft, ConstBackendPtr& bk);

    virtual void generateOdom() override;

    ~LidarOdometry();
};
    
} // namespace frontend

