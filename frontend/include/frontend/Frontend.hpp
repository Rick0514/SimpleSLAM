#pragma once

#include <types/EigenTypes.hpp>

#include <utils/SafeDeque.hpp>
#include <utils/Math.hpp>

#include <dataproxy/DataProxy.hpp>

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

    template<typename Elem>
    void pushLocalOdometry(Elem&&);

    template<typename Elem>
    void pushGlobalOdometry(Elem&&);
    
    Odometry::Ptr getClosestLocalOdom(double stamp) const;

    ~Frontend();
};

    
} // namespace frontend


