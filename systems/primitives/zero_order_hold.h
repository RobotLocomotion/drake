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

/// A zero order hold block with input u, which may be vector-valued (discrete
/// or continuous) or abstract, and discrete output y, where the y is sampled
/// from u with a fixed period.
///
/// @system{ZeroOrderHold, @input_port{u}, @output_port{y}}
///
/// The discrete state space dynamics of %ZeroOrderHold is:
/// ```
///   xₙ₊₁ = uₙ     // update
///   yₙ   = xₙ     // output
///   x₀   = xᵢₙᵢₜ  // initialize
/// ```
/// where xᵢₙᵢₜ = 0 for vector-valued %ZeroOrderHold, and xᵢₙᵢₜ is a given
/// value for abstract-valued %ZeroOrderHold.
///
/// See @ref discrete_systems "Discrete Systems" for general information about
/// discrete systems in Drake, including how they interact with continuous
/// systems.
///
/// @note This system uses a periodic update with zero offset, so the first
///       update occurs at t=0. When used with a Simulator, the output port
///       is equal to xᵢₙᵢₜ after simulator.Initialize(), but is immediately
///       updated to u₀ at the start of the first step. If you want to force
///       that initial update, use simulator.AdvanceTo(0.).
///
/// @note For an abstract-valued ZeroOrderHold, scalar-type conversion is not
///       supported since AbstractValue does not support it.
///
/// @ingroup primitive_systems
template <typename T>
class ZeroOrderHold final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ZeroOrderHold)

  /// Constructs a ZeroOrderHold system with the given `period_sec`, over a
  /// vector-valued input of size `vector_size`. The default initial
  /// value for this system will be zero. The offset is always zero, meaning
  /// that the first update occurs at t=0.
  ZeroOrderHold(double period_sec, int vector_size)
      : ZeroOrderHold(period_sec, vector_size, nullptr) {}

  /// Constructs a ZeroOrderHold system with the given `period_sec`, over a
  /// abstract-valued input `abstract_model_value`. The default initial value
  /// for this system will be `abstract_model_value`. The offset is always
  /// zero, meaning that the first update occurs at t=0.
  ZeroOrderHold(double period_sec, const AbstractValue& abstract_model_value)
      : ZeroOrderHold(period_sec, -1, abstract_model_value.Clone()) {}

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit ZeroOrderHold(const ZeroOrderHold<U>& other);

  ~ZeroOrderHold() final = default;

  /// Returns the sole input port.
  const InputPort<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

  /// (Advanced) Manually sample the input port and copy ("latch") the value
  /// into the state. This emulates an update event and is mostly useful for
  /// testing.
  void LatchInputPortToState(Context<T>* context) const {
    if (is_abstract()) {
      LatchInputAbstractValueToState(*context, &context->get_mutable_state());
    } else {
      LatchInputVectorToState(*context, &context->get_mutable_discrete_state());
    }
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename U> friend class ZeroOrderHold;

  // All of the other constructors delegate here.
  ZeroOrderHold(double period_sec, int vector_size,
                std::unique_ptr<const AbstractValue> model_value);

  // Sets the output port value to the vector value that is currently
  // latched in the zero-order hold.
  void CopyLatchedVector(
      const Context<T>& context,
      BasicVector<T>* output) const;

  // Latches the input port into the discrete vector-valued state.
  void LatchInputVectorToState(
      const Context<T>& context,
      DiscreteValues<T>* discrete_state) const;

  // Sets the output port value to the abstract value that is currently
  // latched in the zero-order hold.
  void CopyLatchedAbstractValue(
      const Context<T>& context,
      AbstractValue* output) const;

  // Latches the abstract input port into the abstract-valued state.
  void LatchInputAbstractValueToState(
      const Context<T>& context,
      State<T>* state) const;

  bool is_abstract() const { return abstract_model_value_ != nullptr; }

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
    this->DeclareVectorInputPort("u", model_value);
    this->DeclareVectorOutputPort("y",
        model_value, &ZeroOrderHold::CopyLatchedVector,
        {this->xd_ticket()});
    this->DeclareDiscreteState(vector_size);
    this->DeclarePeriodicDiscreteUpdateEvent(period_sec_, 0.,
        &ZeroOrderHold::LatchInputVectorToState);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    this->DeclareAbstractInputPort("u", *abstract_model_value_);

    // Because we're working with type-erased AbstractValue objects directly
    // (not typical), there isn't a nice sugar method available that takes
    // class methods. We have to use the generic port declaration method that
    // uses free functions.
    this->DeclareAbstractOutputPort("y",
        // Allocator function.
        [this]() { return abstract_model_value_->Clone(); },
        // Calculator function.
        [this](const Context<T>& context, AbstractValue* output) {
          this->CopyLatchedAbstractValue(context, &*output);
        },
        {this->xa_ticket()});

    this->DeclareAbstractState(abstract_model_value_->Clone());
    this->DeclarePeriodicUnrestrictedUpdateEvent(period_sec_, 0.,
        &ZeroOrderHold::LatchInputAbstractValueToState);
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
void ZeroOrderHold<T>::CopyLatchedVector(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& state_value = context.get_discrete_state(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::LatchInputVectorToState(
    const Context<T>& context,
    DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input = get_input_port().Eval(context);
  BasicVector<T>& state_value = discrete_state->get_mutable_vector(0);
  state_value.SetFromVector(input);
}

template <typename T>
void ZeroOrderHold<T>::CopyLatchedAbstractValue(const Context<T>& context,
                                                AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValue& state_value = context.get_abstract_state().get_value(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::LatchInputAbstractValueToState(
    const Context<T>& context,
    State<T>* state) const {
  DRAKE_ASSERT(is_abstract());
  const auto& input = get_input_port().template Eval<AbstractValue>(context);
  AbstractValue& state_value =
      state->get_mutable_abstract_state().get_mutable_value(0);
  state_value.SetFrom(input);
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ZeroOrderHold)
