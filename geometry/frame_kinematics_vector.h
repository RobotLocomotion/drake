#pragma once

#include "drake/geometry/kinematics_vector.h"

namespace drake {
namespace geometry {

template <typename T>
using FramePoseVector DRAKE_DEPRECATED(
    "2022-10-01", "Include drake/geometry/kinematics_vector.h instead") =
    KinematicsVector<FrameId, math::RigidTransform<T>>;

}  // namespace geometry
}  // namespace drake
