#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// define double type first

namespace EigenTypes
{
    template<typename Scalar>
    using Pose6 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
    
    template<typename Scalar>
    using V3 = Eigen::Matrix<Scalar, 3, 1>;
    
    template<typename Scalar>
    using Qt = Eigen::Quaternion<Scalar>;

    template<typename Scalar>
    using M3 = Eigen::Matrix<Scalar, 3, 3>;

    template<typename Scalar>
    using M4 = Eigen::Matrix<Scalar, 4, 4>;

    using Pose6d = Eigen::Isometry3d;
    using M4d = Eigen::Matrix4d;
    using M3d = Eigen::Matrix3d;
    using V3d = Eigen::Vector3d;
    using V3f = Eigen::Vector3f;

    using V4d = Eigen::Vector4d;
    using V6d = Eigen::Matrix<double, 6, 1>;

    using Qd = Eigen::Quaterniond;
    using Qf = Eigen::Quaternionf;

} // namespace EigenTypes