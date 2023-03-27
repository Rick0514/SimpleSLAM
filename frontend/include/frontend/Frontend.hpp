#include <types/EigenTypes.hpp>
#include <utils/SafeDeque.hpp>

#include <thread>

namespace frontend
{
using namespace EigenTypes;
using namespace utils;

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


