#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace systems {
namespace controllers_test {

// Computes torque predicted by inverse dynamics for use with inverse dynamics
// and inverse dynamics controller testing.
VectorX<double> ComputeTorque(
    const multibody::MultibodyPlant<double>& plant,
    const VectorX<double>& q,
    const VectorX<double>& v,
    const VectorX<double>& vd_d,
    systems::Context<double>* context) {
  // Populate the context.
  plant.SetPositions(context, q);
  plant.SetVelocities(context, v);
  // Compute inverse dynamics.
  multibody::MultibodyForces<double> external_forces(plant);
  plant.CalcForceElementsContribution(*context, &external_forces);
  return plant.CalcInverseDynamics(*context, vd_d, external_forces);
}

}  // namespace controllers_test
}  // namespace systems
}  // namespace drake
