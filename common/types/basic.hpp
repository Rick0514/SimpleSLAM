#pragma once
#include <types/PCLTypes.hpp>
#include <pcl/pcl_config.h>

#if PCL_VERSION_COMPARE(<=, 1, 10, 0)
#include <pcl/make_shared.h>
#else
#include <pcl/memory.h>
#endif

// provide basic types for the whole project !!

using index_t = size_t;
using stamp_t = double;
using scalar_t = double;
using pt_t = PCLTypes::Pxyzi;
using pc_t = PCLTypes::PC<pt_t>;
using pose_t = EigenTypes::Pose6<scalar_t>;

struct constant
{

#ifdef USE_BAG
    static constexpr bool usebag = true;
#else
    static constexpr bool usebag = false;
#endif

    static constexpr int numCores{2};
};

struct KeyFrame
{
    using Scalar = scalar_t;
    pose_t pose;
    pc_t::Ptr pc;    
    KeyFrame() : pc(pcl::make_shared<pc_t>()){};
    KeyFrame(const pc_t::Ptr& _pc, const pose_t& _p) : pc(_pc), pose(_p){}      
};

struct Odometry
{
    stamp_t stamp;
    pose_t odom;
    Odometry(){}
    Odometry(stamp_t t, const pose_t& p) : stamp(t), odom(p){}

    using Ptr = std::shared_ptr<Odometry>;
};