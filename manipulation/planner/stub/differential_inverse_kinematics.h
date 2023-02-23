#pragma once

#warning The include path drake/manipulation/planner/differential_inverse_kinematics.h is deprecated and will be removed on or after 2023-06-01; instead, use drake/multibody/inverse_kinematics/differential_inverse_kinematics.h.

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"

namespace drake {
namespace manipulation {
namespace planner {

using DifferentialInverseKinematicsStatus DRAKE_DEPRECATED(
    "2023-06-01",
    "Please use drake::multibody::DifferentialInverseKinematicsStatus "
    "instead.") = drake::multibody::DifferentialInverseKinematicsStatus;

using DifferentialInverseKinematicsParameters DRAKE_DEPRECATED(
    "2023-06-01",
    "Please use drake::multibody::DifferentialInverseKinematicsParameters "
    "instead.") = drake::multibody::DifferentialInverseKinematicsParameters;

using DifferentialInverseKinematicsResult DRAKE_DEPRECATED(
    "2023-06-01",
    "Please use drake::multibody::DifferentialInverseKinematicsResult "
    "instead.") = drake::multibody::DifferentialInverseKinematicsResult;

// We can't deprecate this namespace hoisting; instead, we rely on the pragma
// and assume that correcting the include file will be sufficient.
using drake::multibody::ComputePoseDiffInCommonFrame;
using drake::multibody::DoDifferentialInverseKinematics;

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
