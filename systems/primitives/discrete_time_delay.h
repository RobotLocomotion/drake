#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A DiscreteTimeDelay block with input `u`, which is vector-valued, and
/// output `delayed u` which is previously received input, delayed by the
/// given amount.
///
/// @ingroup primitive_systems
/// @system{DiscreteTimeDelay, @input_port{u}, @output_port{delayed u}}
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

  /// Manually sample the input port and save the value into the state.
  /// This emulates an update event and is mostly useful for testing.
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

  double update_sec_{};
  int delay_timesteps_{};
  int vector_size_{};
};

}  // namespace systems
}  // namespace drake
