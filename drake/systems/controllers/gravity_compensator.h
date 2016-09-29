#pragma once

#include <memory>

#include "drake/drakeSystemControllers_export.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

/// A gravity compensator system that computes a vector of generalized gravity
/// forces that exactly counteracts the effects of gravity for a given
/// `RigidBodyTree`. The input to this block is the vector valued port
/// corresponding generalized positions from a `RigidBodyPlant` `q` and the
/// output is a vector valued port given by `y = G(q)`, where, for a given
/// `RigidBodyTree`, `G(q)` is a vector of generalised gravity forces
/// corresponding to a given joint configuration `q`. The size of the input
/// corresponds to the number of generalized positions in the `RigidBodyTree`
/// and the size of the output corresponds to the number of actuators.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class DRAKESYSTEMCONTROLLERS_EXPORT GravityCompensator : public LeafSystem<T> {
 public:
  /// Constructs a gravity compensator for a given `RigidBodyTree`.
  explicit GravityCompensator(const RigidBodyTree& rigid_body_tree);

  /// Sets the output port value to the generalised gravity forces
  /// corresponding to a joint configuration as specified in the input.
  /// If the number of connected input or output ports differs from one or the
  /// input ports are not of size length_, a std::runtime_error will be thrown.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  const RigidBodyTree& rigid_body_tree_;
};

}  // namespace systems
}  // namespace drake
