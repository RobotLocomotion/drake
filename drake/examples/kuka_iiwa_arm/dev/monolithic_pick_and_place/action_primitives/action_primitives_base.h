#pragma once

#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/// This class is the base of an `ActionPrimitive`. `ActionPrimitive` in the
/// sense of the pick-and-place demo are systems that can be used to
/// independently
/// perform some kind of action on the simulated robot / gripper system. The
/// functionality they offer is to decouple the state transition style logic of
/// high level planners and the implementation level logic of how long an action
/// needs
/// to run, or what kind of LCM message an action must be encoded in etc.
/// Each `ActionPrimitive` can maintain internal state as well as timing
/// information
/// that are task specific. Every `ActionPrimitive` has some default features,
/// as
/// defined by this base class :
/// 1. They have atleast 1 internal state variable, the `ActionPrimitiveState`.
/// 2. They have at least one Abstract output port that sends the current
/// `ActionPrimitiveState`.
/// 3. They have a declared periodic unrestricted update.
/// 4. They are not direct feedthrough (child classes must continue to enforce
/// this in their DoCalcUpdate design).
class ActionPrimitive : public systems::LeafSystem<double> {
 public:
  /// The constructor to an action primitive.
  /// @param desired_update_interval The update period for the unrestricted
  /// state
  /// update.
  ActionPrimitive(double desired_update_interval = 0.01);

  /// This method gets the status output port. Child classes may implement
  /// Additional getters as needed.
  const systems::OutputPortDescriptor<double>& get_status_output_port() const {
    return this->get_output_port(status_output_port_);
  }

  // LeafSystem override.
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState() const final;

  // LeafSystem override.
  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const final;

  // LeadSystemOverride.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const final;

  // LeafSystem override.
  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const final {
    DoExtendedCalcUnrestrictedUpdate(context, state);
  }

  // This kind of a system
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port, int output_port) const final {
    return false;
  }

 protected:
  /// Derived class need to implement this.
  virtual std::unique_ptr<systems::AbstractValue>
  ExtendedAllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const = 0;

  /// Derived class need to implement this.
  virtual std::vector<std::unique_ptr<systems::AbstractValue>>
  AllocateExtendedAbstractState() const = 0;

  /// Derived class need to implement this.
  virtual void DoExtendedCalcOutput(
      const systems::Context<double>& context,
      systems::SystemOutput<double>* output) const = 0;

  /// Derived class need to implement this.
  virtual void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const = 0;

 private:
  int status_output_port_{-1};
  const double update_interval_{0.01};
};

}  // namespace drake
}  // namespace examples
}  // namespace kuka_iiwa_arm
}  // namespace pick_and_place