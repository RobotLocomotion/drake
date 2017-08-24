#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A ZeroOrderHold block with input `u`, which may be vector-valued (discrete
/// or continuous) or abstract, and discrete output `y`, where the y is sampled
/// from u with a fixed period.
/// @note For an abstract-valued ZeroOrderHold, transmografication is not
/// supported since AbstractValue does not support it.
/// @ingroup primitive_systems
template <typename T>
class ZeroOrderHold : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ZeroOrderHold)

  /// Constructs a ZeroOrderHold system with the given @p period_sec, over a
  /// vector-valued input of size @p size. The default initial value for this
  /// system will be zero.
  ZeroOrderHold(double period_sec, int size);

  /// Constructs a ZeroOrderHold system with the given @p period_sec, over a
  /// abstract-valued input @p model_value. The default initial value for this
  /// system will be @p model_value.
  ZeroOrderHold(double period_sec, const AbstractValue& model_value);

  ~ZeroOrderHold() override {}

  // TODO(eric.cousineau): Create a SisoSystem that is type-agnostic, and
  // have both this and SisoVectorSystem inherit from it (#6490).

  /// Returns the sole input port.
  const InputPortDescriptor<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;


 protected:
  // Override feedthrough detection to avoid the need for `DoToSymbolic()`.
  bool DoHasDirectFeedthrough(const SymbolicSystemInspector* sparsity,
                              int input_port, int output_port) const override;

  // System<T> override.  Returns a ZeroOrderHold<symbolic::Expression> with
  // the same dimensions as this ZeroOrderHold.
  ZeroOrderHold<symbolic::Expression>* DoToSymbolic() const override;

  /// Sets the output port value to the vector value that is currently
  /// latched in the zero-order hold.
  void DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const;

  /// Latches the input port into the discrete vector-valued state.
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>& events,
      DiscreteValues<T>* discrete_state) const override;

  // Return a cloned copy of the initial abstract value.
  std::unique_ptr<AbstractValue> AllocateAbstractValue(const Context<T>&) const;

  // Same as `DoCalcVectorOutput`, but for abstract values.
  void DoCalcAbstractOutput(
      const Context<T>& context,
      AbstractValue* output) const;

  // Same as `DoCalcDiscreteVariablesUpdate`, but for abstract values.
  void DoCalcUnrestrictedUpdate(
      const Context<T>& context,
      const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
      State<T>* state) const override;

 private:
  bool is_abstract() const { return abstract_model_value_ != nullptr; }

  const double period_sec_{};
  const std::unique_ptr<const AbstractValue> abstract_model_value_;
};

}  // namespace systems
}  // namespace drake
