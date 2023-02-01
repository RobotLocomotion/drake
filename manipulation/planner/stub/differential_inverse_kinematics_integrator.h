#pragma once

#warning The include path drake/manipulation/planner/differential_inverse_kinematics_integrator.h is deprecated and will be removed on or after 2023-06-01; instead, use drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h.

#include "drake/common/drake_deprecated.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h"

namespace drake {
namespace manipulation {
namespace planner {

using DifferentialInverseKinematicsIntegrator DRAKE_DEPRECATED(
    "2023-06-01",
    "Please use drake::multibody::DifferentialInverseKinematicsIntegrator "
    "instead.") = drake::multibody::DifferentialInverseKinematicsIntegrator;

}
}  // namespace manipulation
}  // namespace drake
