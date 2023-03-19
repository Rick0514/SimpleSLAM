#include <pcl/point_types.h>

#include <frontend/OdometryBase.hpp>

#include <PCR/PointCloudRegister.hpp>

namespace frontend
{

class Frontend;
class Backend;

using namespace EigenTypes;
using namespace utils;

class LidarOdometry : public OdometryBase
{
private:

    std::shared_ptr<Frontend> mFrontendPtr;
    std::shared_ptr<Backend> mBackendPtr;

    // pointtype xyz for temp
    std::unique_ptr<PCR::PointCloudRegister<pcl::PointXYZ>> mPcr;

public:
    explicit LidarOdometry(const std::shared_ptr<Frontend>& ft);

    virtual void generateOdom() override;

    ~LidarOdometry();
};
    
} // namespace frontend

