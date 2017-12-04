#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
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
  /// vector-valued input of size `vector_size`. The default initial
  /// value for this system will be zero.
  ZeroOrderHold(double period_sec, int vector_size)
      : ZeroOrderHold(period_sec, vector_size, nullptr) {}

  /// Constructs a ZeroOrderHold system with the given @p period_sec, over a
  /// abstract-valued input `abstract_model_value`. The default initial value
  /// for this system will be `abstract_model_value`.
  ZeroOrderHold(double period_sec, const AbstractValue& abstract_model_value)
      : ZeroOrderHold(period_sec, -1, abstract_model_value.Clone()) {}

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit ZeroOrderHold(const ZeroOrderHold<U>& other);

  ~ZeroOrderHold() override {}

  // TODO(eric.cousineau): Possibly share single port interface with
  // PassThrough (#6490).

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
  optional<bool> DoHasDirectFeedthrough(
      int input_port, int output_port) const override;

  // Sets the output port value to the vector value that is currently
  // latched in the zero-order hold.
  void DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const;

  // Latches the input port into the discrete vector-valued state.
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>& events,
      DiscreteValues<T>* discrete_state) const override;

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

  ZeroOrderHold(double period_sec, int vector_size,
                std::unique_ptr<const AbstractValue> model_value);

  // Allow different specializations to access each other's private data.
  template <typename U> friend class ZeroOrderHold;

  double period_sec_{};
  std::unique_ptr<const AbstractValue> abstract_model_value_;
};

}  // namespace systems
}  // namespace drake
