#pragma once
#include <types/basic.hpp>
#include <types/EigenTypes.hpp>
#include <dataproxy/DataProxy.hpp>

namespace ros { class NodeHandle; }

namespace dataproxy
{
using namespace EigenTypes;

class EkfOdomProxy : public DataProxy<Odometry>
{
public:
    struct Odom{
        stamp_t stamp;
        V3<scalar_t> t;
        Qt<scalar_t> q;
    };

protected:
    
    template <typename T>
    using NoiseVector = Eigen::Matrix<scalar_t, T::RowsAtCompileTime, 1>;

    class Ros;
    std::unique_ptr<Ros> mRosImpl;

    class Filter;
    std::unique_ptr<Filter> mFilterImpl;

    bool mUpdateImuFlag;

public:

    explicit EkfOdomProxy(ros::NodeHandle& nh, int size);

    void imuHandler(const Odom&);
    void wheelHandler(const Odom&);

    ~EkfOdomProxy();
};
    
} // namespace dataproxy
