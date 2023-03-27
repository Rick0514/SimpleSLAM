#include <types/EigenTypes.hpp>
#include <utils/SafeDeque.hpp>

#include <dataproxy/DataProxy.hpp>

#include <thread>

namespace frontend
{
using namespace EigenTypes;
using namespace utils;
using namespace dataproxy;

class Frontend
{
private:

    Pose6d mOdom2Map;

    std::shared_ptr<concurrency::SafeDeque<Odometry>> mLocalOdometry;
    std::shared_ptr<concurrency::SafeDeque<Odometry>> mGlobalOdometry;

public:
    Frontend();

    void publish() const;

    Pose6d get() const { return mOdom2Map; }

    void pushLocalOdometry(const Odometry&);
    void pushGlobalOdometry(const Odometry&);
    
    void getClosestLocalOdom(double stamp, Odometry&) const;

    ~Frontend();
};

    
} // namespace frontend


