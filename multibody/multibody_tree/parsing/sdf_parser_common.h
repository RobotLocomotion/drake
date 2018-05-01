#pragma once

#include <sdf/sdf.hh>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

Eigen::Vector3d ToVector3(const ignition::math::Vector3d& vector);

Eigen::Isometry3d ToIsometry3(const ignition::math::Pose3d& pose);

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
