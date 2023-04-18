#pragma once
#include <types/PCLTypes.hpp>

// provide basic types for the whole project !!

using index_t = size_t;
using stamp_t = double;
using scalar_t = double;
using pt_t = PCLTypes::Pxyzi;
using pc_t = PCLTypes::PC<pt_t>;
using pose_t = EigenTypes::Pose6<scalar_t>;

struct constant
{
    static constexpr bool usebag = false;
};

struct KeyFrame
{
    pose_t pose;
    pc_t::Ptr pc;    
};

struct Odometry
{
    stamp_t stamp;
    pose_t odom;
    Odometry(){}
    Odometry(stamp_t t, const pose_t& p) : stamp(t), odom(p){}

    using Ptr = std::shared_ptr<Odometry>;
};