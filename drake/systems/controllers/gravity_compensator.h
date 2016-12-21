#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

/// A gravity compensator system that computes a vector of generalized gravity
/// forces that exactly counteracts the effects of gravity for a given
/// `RigidBodyTree` configuration. The input to this block is a vector valued
/// port containing generalized positions `q` from a `RigidBodyPlant`.
/// The output is a vector valued port containing the value `y = G(q)`, where,
/// for a given `RigidBodyTree`, `G(q)` is a vector of generalized gravity
/// forces corresponding to `q`. The size of the input corresponds to the
/// number of generalized positions in the `RigidBodyTree` and the size of the
/// output corresponds to the number of actuators. Note that the current
/// implementation assumes that every DoF of the `RigidBodyPlant` is actuated.
/// If an under-actuated system is provided, the process will abort.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup control_systems
template <typename T>
class GravityCompensator : public LeafSystem<T> {
 public:
  /// Constructs a gravity compensator for the given @tree. The provided @p tree
  /// must be fully actuated, i.e., the number of actuators must equal the
  /// number of positions in the RigidBodyTree. Otherwise, the process will
  /// abort.
  explicit GravityCompensator(const RigidBodyTree<T>& tree);

 private:
  // Sets the output port value to the generalized gravity forces
  // corresponding to a joint configuration as specified in the input.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  const RigidBodyTree<T>& tree_;
};

}  // namespace systems
}  // namespace drake
