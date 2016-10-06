#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

/// A gravity compensator system that computes a vector of generalized gravity
/// forces that exactly counteracts the effects of gravity for a given
/// `RigidBodyTree` configuration. The input to this block is a vector valued
/// port containing to generalized positions from a `RigidBodyPlant` `q`.
/// The output is a vector valued port containing the value `y = G(q)`, where,
/// for a given `RigidBodyTree`, `G(q)` is a vector of generalized gravity
/// forces corresponding to `q`. The size of the input corresponds to the
/// number of generalized positions in the `RigidBodyTree` and the size of the
/// output corresponds to the number of actuators. Note that the current
/// implementation assumes that every DoF of the `RigidBodyPlant` is actuated.
/// If used on an underactuated system, gravity compensation will not be
/// achieved.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup control_systems
template <typename T>
class DRAKE_EXPORT GravityCompensator : public LeafSystem<T> {
 public:
  /// Constructs a gravity compensator for a given `RigidBodyTree`.
  explicit GravityCompensator(const RigidBodyTree& rigid_body_tree);

  /// Sets the output port value to the generalised gravity forces
  /// corresponding to a joint configuration as specified in the input.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  const RigidBodyTree& rigid_body_tree_;
};

}  // namespace systems
}  // namespace drake
