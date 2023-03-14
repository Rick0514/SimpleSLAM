#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// define double type first

namespace EigenTypes
{
    using Pose6d = Eigen::Isometry3d;
    using M4d = Eigen::Matrix4d;
    using M3d = Eigen::Matrix3d;
    using V3d = Eigen::Vector3d;
    using V4d = Eigen::Vector4d;
    using V6d = Eigen::Matrix<double, 6, 1>;

} // namespace EigenTypes