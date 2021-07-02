#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A zero order hold block with input u, which may be vector-valued (discrete
/// or continuous) or abstract, and discrete output y, where the y is sampled
/// from u with a fixed period.
///
/// @system
/// name: ZeroOrderHold
/// input_ports:
/// - u
/// output_ports:
/// - y
/// @endsystem
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
/// @tparam_default_scalar
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

  /// Reports the period of this hold (in seconds).
  double period() const { return period_sec_; }

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

  // Latches the input port into the discrete vector-valued state.
  void LatchInputVectorToState(
      const Context<T>& context,
      DiscreteValues<T>* discrete_state) const;

  // Latches the abstract input port into the abstract-valued state.
  void LatchInputAbstractValueToState(
      const Context<T>& context,
      State<T>* state) const;

  bool is_abstract() const { return abstract_model_value_ != nullptr; }

  double period_sec_{};
  std::unique_ptr<const AbstractValue> abstract_model_value_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ZeroOrderHold)
