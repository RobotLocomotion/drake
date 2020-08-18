#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A discrete time delay block with input u, which is vector-valued (discrete
/// or continuous) or abstract, and output delayed_u which is previously
/// received input, delayed by the given amount.  The initial output will be a
/// vector of zeros for vector-valued or a given value for abstract-valued
/// until the delay time has passed.
///
/// @system
/// name: DiscreteTimeDelay
/// input_ports:
/// - u
/// output_ports:
/// - delayed_u
/// @endsystem
///
/// Let t,z ∈ ℕ be the number of delay time steps and the input vector size.
/// For abstract-valued %DiscreteTimeDelay, z is 1.
/// The state x ∈ ℝ⁽ᵗ⁺¹⁾ᶻ is partitioned into t+1 blocks x[0] x[1] ... x[t],
/// each of size z. The input and output are u,y ∈ ℝᶻ.
/// The discrete state space dynamics of %DiscreteTimeDelay is:
/// ```
///   xₙ₊₁ = xₙ[1] xₙ[2] ... xₙ[t] uₙ  // update
///   yₙ = xₙ[0]                       // output
///   x₀ = xᵢₙᵢₜ                       // initialize
/// ```
/// where xᵢₙᵢₜ = 0 for vector-valued %DiscreteTimeDelay and xᵢₙᵢₜ is a
/// given value for abstract-valued %DiscreteTimeDelay.
///
/// See @ref discrete_systems "Discrete Systems" for general information about
/// discrete systems in Drake, including how they interact with continuous
/// systems.
///
/// @note While the output port can be sampled at any continuous time, this
///       system does not interpolate.
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class DiscreteTimeDelay final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeDelay)

  /// Constructs a DiscreteTimeDelay system updating every `update_sec` and
  /// delaying a vector-valued input of size `vector_size` for
  /// `delay_timesteps` number of updates.
  DiscreteTimeDelay(double update_sec, int delay_timesteps, int vector_size)
      : DiscreteTimeDelay(update_sec, delay_timesteps, vector_size, nullptr) {}

  /// Constructs a DiscreteTimeDelay system updating every `update_sec` and
  /// delaying an abstract-valued input of type `abstract_model_value` for
  /// `delay_timesteps` number of updates.
  DiscreteTimeDelay(double update_sec, int delay_timesteps,
                    const AbstractValue& abstract_model_value)
      : DiscreteTimeDelay(update_sec, delay_timesteps, -1,
                          abstract_model_value.Clone()) {}

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeDelay(const DiscreteTimeDelay<U>& other);

  ~DiscreteTimeDelay() final = default;

  /// (Advanced) Manually samples the input port and updates the state of the
  /// block, sliding the delay buffer forward and placing the sampled input at
  /// the end. This emulates an update event and is mostly useful for testing.
  void SaveInputToBuffer(Context<T>* context) const {
    if (is_abstract()) {
      SaveInputAbstractValueToBuffer(*context, &context->get_mutable_state());
    } else {
      SaveInputVectorToBuffer(*context, &context->get_mutable_discrete_state());
    }
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename U> friend class DiscreteTimeDelay;

  // All of the other constructors delegate here.
  DiscreteTimeDelay(double update_sec, int delay_timesteps, int vector_size,
                    std::unique_ptr<const AbstractValue> model_value);

  // Sets the output port value to the properly delayed vector value.
  void CopyDelayedVector(const Context<T>& context,
                         BasicVector<T>* output) const;

  // Saves the input port into the discrete vector-valued state,
  void SaveInputVectorToBuffer(const Context<T>& context,
                               DiscreteValues<T>* discrete_state) const;

  // Sets the output port value to the properly delayed abstract value.
  void CopyDelayedAbstractValue(const Context<T>& context,
                                AbstractValue* output) const;

  // Saves the input port into the abstract-valued state,
  void SaveInputAbstractValueToBuffer(const Context<T>& context,
                                      State<T>* state) const;

  bool is_abstract() const { return abstract_model_value_ != nullptr; }

  const double update_sec_{};
  const int delay_buffer_size_{};
  const int vector_size_{};
  const std::unique_ptr<const AbstractValue> abstract_model_value_;
};

}  // namespace systems
}  // namespace drake
