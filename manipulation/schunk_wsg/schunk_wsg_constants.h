/**
 * @file
 *
 * Constants defined in this file are for the Schunk WSG gripper modeled in
 * models/schunk_wsg_50.sdf.
 */

#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

constexpr int kSchunkWsgNumActuators = 2;
constexpr int kSchunkWsgNumPositions = 2;
constexpr int kSchunkWsgNumVelocities = 2;

// TODO(russt): These constants are predicated on the idea that we can map
// one finger's state => gripper state.  We should prefer using
// MakeMultibodyStateToWsgState and MakeMultibodyForceToWsgForce instead.
// But I cannot deprecate them yet without performing surgery on the old
// controller.
constexpr int kSchunkWsgPositionIndex = 0;
constexpr int kSchunkWsgVelocityIndex =
    kSchunkWsgNumPositions + kSchunkWsgPositionIndex;

// TODO(sammy): need to double check this.
constexpr double kSchunkWsgLcmStatusPeriod = 0.05;

/**
 * Returns the position vector corresponding to the open position of the
 * gripper.
 */
template <typename T>
VectorX<T> GetSchunkWsgOpenPosition() {
  // clang-format off
  return (VectorX<T>(kSchunkWsgNumPositions) <<
      -0.0550667,
       0.0550667).finished();
  // clang-format on
}

/// Extract the distance between the fingers (and its time derivative) out
/// of the plant model which pretends the two fingers are independent.
/// @ingroup manipulation_systems
template <typename T>
std::unique_ptr<systems::MatrixGain<T>>
MakeMultibodyStateToWsgStateSystem() {
  Eigen::Matrix<double, 2, 4> D;
  // clang-format off
  D << -1, 1, 0,  0,
      0,  0, -1, 1;
  // clang-format on
  return std::make_unique<systems::MatrixGain<T>>(D);
}

/// Extract the gripper measured force from the generalized forces on the two
/// fingers.
/// @ingroup manipulation_systems
template <typename T>
class MultibodyForceToWsgForceSystem : public systems::VectorSystem<T> {
 public:
  MultibodyForceToWsgForceSystem()
      : systems::VectorSystem<T>(
            systems::SystemTypeTag<MultibodyForceToWsgForceSystem>{}, 2, 1) {}

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MultibodyForceToWsgForceSystem(
      const MultibodyForceToWsgForceSystem<U>&)
      : MultibodyForceToWsgForceSystem<T>() {}

  void DoCalcVectorOutput(
      const systems::Context<T>&,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
    unused(state);
    // gripper force = abs(-finger0 + finger1).
    using std::abs;
    (*output)(0) = abs(input(0) - input(1));
  }
};

/// Helper method to create a MultibodyForceToWsgForceSystem.
/// @ingroup manipulation_systems
template <typename T>
std::unique_ptr<systems::VectorSystem<T>>
MakeMultibodyForceToWsgForceSystem() {
  return std::make_unique<MultibodyForceToWsgForceSystem<T>>();
}


}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
