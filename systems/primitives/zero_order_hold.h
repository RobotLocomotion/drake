#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
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
  const InputPort<T>& get_input_port() const {
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

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(
    double period_sec, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<systems::ZeroOrderHold>()),
      period_sec_(period_sec),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    // TODO(david-german-tri): remove the size parameter from the constructor
    // once #3109 supporting automatic sizes is resolved.
    BasicVector<T> model_value(vector_size);
    this->DeclareVectorInputPort(model_value);
    this->DeclareVectorOutputPort(
        model_value, &ZeroOrderHold::DoCalcVectorOutput);
    this->DeclareDiscreteState(vector_size);
    this->DeclarePeriodicDiscreteUpdate(period_sec_);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    this->DeclareAbstractInputPort(*abstract_model_value_);
    // Use the std::function<> overloads to work with `AbstractValue` type
    // directly and maintain type erasure.
    auto abstract_value_allocator = [this]() {
      return abstract_model_value_->Clone();
    };
    namespace sp = std::placeholders;
    this->DeclareAbstractOutputPort(
        abstract_value_allocator,
        std::bind(&ZeroOrderHold::DoCalcAbstractOutput, this, sp::_1, sp::_2));
    this->DeclareAbstractState(abstract_model_value_->Clone());
    this->DeclarePeriodicUnrestrictedUpdate(period_sec_, 0.);
  }
}

template <typename T>
template <typename U>
ZeroOrderHold<T>::ZeroOrderHold(const ZeroOrderHold<U>& other)
    : ZeroOrderHold(other.period_sec_,
                    other.is_abstract() ? -1 : other.get_input_port().size(),
                    other.is_abstract() ? other.abstract_model_value_->Clone()
                                        : nullptr) {}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& state_value = context.get_discrete_state(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcDiscreteVariableUpdates(
    const Context<T>& context,
    const std::vector<const DiscreteUpdateEvent<T>*>&,
    DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input = get_input_port().Eval(context);
  BasicVector<T>& state_value = discrete_state->get_mutable_vector(0);
  state_value.SetFromVector(input);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcAbstractOutput(const Context<T>& context,
                                            AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  // Do not use `get_abstract_state<AbstractValue>` since it would cast
  // the value to `Value<AbstractValue>`, which is an invalid type by design.
  const AbstractValue& state_value =
      context.get_abstract_state().get_value(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcUnrestrictedUpdate(
    const Context<T>& context,
    const std::vector<const UnrestrictedUpdateEvent<T>*>&,
    State<T>* state) const {
  DRAKE_ASSERT(is_abstract());
  const auto& input = get_input_port().template Eval<AbstractValue>(context);
  // See `DoCalcAbstractOutput` for rationale regarding non-templated value
  // accessor.
  AbstractValue& state_value =
      state->get_mutable_abstract_state().get_mutable_value(0);
  state_value.SetFrom(input);
}

template <typename T>
optional<bool> ZeroOrderHold<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a zero-order hold will not have direct feedthrough, as the
  // output only depends on the state, not the input.
  return false;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ZeroOrderHold)
