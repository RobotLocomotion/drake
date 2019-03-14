#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A discrete time delay block with input u, which is vector-valued (discrete
/// or continuous), and output delayed_u which is previously received input,
/// delayed by the given amount.  The initial output will be a vector of zeros
/// until the delay time has passed.
///
/// @ingroup primitive_systems
/// @system{DiscreteTimeDelay, @input_port{u}, @output_port{delayed_u}}
///
/// Let t,z ∈ ℕ be the number of delay time steps and the input vector size.
/// The state x ∈ ℝ⁽ᵗ⁺¹⁾ᶻ is partitioned into t+1 blocks x[0] x[1] ... x[t],
/// each of size z. The input and output are u,y ∈ ℝᶻ.
/// The discrete state space dynamics of %DiscreteTimeDelay is:
/// ```
///   xₙ₊₁ = xₙ[1] xₙ[2] ... xₙ[t] uₙ  // update
///   yₙ = xₙ[0]                       // output
///   x₀ = xᵢₙᵢₜ                       // initialize
/// ```
/// where xᵢₙᵢₜ = 0 for vector-valued %DiscreteTimeDelay.
///
/// See @ref discrete_systems "Discrete Systems" for general information about
/// discrete systems in Drake, including how they interact with continuous
/// systems.
///
/// @note While the output port can be sampled at any continuous time, this
///       system does not interpolate.
///
/// @tparam T Must be one of drake's default scalar types.
template <typename T>
class DiscreteTimeDelay final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeDelay)

  /// Constructs a DiscreteTimeDelay system updating every `update_sec` and
  /// delaying a vector-valued input of size `vector_size` for
  /// `delay_timesteps` number of updates.
  DiscreteTimeDelay(double update_sec, int delay_timesteps, int vector_size);

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeDelay(const DiscreteTimeDelay<U>& other);

  ~DiscreteTimeDelay() final = default;

  /// Returns the sole input port.
  const InputPort<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

  /// (Advanced) Manually samples the input port and updates the state of the
  /// block, sliding the delay buffer forward and placing the sampled input at
  /// the end. This emulates an update event and is mostly useful for testing.
  void SaveInputToBuffer(Context<T>* context) const {
    SaveInputVectorToBuffer(*context, &context->get_mutable_discrete_state());
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename U> friend class DiscreteTimeDelay;

  // Override feedthrough detection to avoid the need for `DoToSymbolic()`.
  optional<bool> DoHasDirectFeedthrough(int input_port,
                                        int output_port) const final;

  // Sets the output port value to the properly delayed vector value.
  void CopyDelayedVector(const Context<T>& context,
                         BasicVector<T>* output) const;

  // Saves the input port into the discrete vector-valued state,
  void SaveInputVectorToBuffer(const Context<T>& context,
                               DiscreteValues<T>* discrete_state) const;

  const double update_sec_{};
  const int delay_buffer_size_{};
  const int vector_size_{};
};

}  // namespace systems
}  // namespace drake
