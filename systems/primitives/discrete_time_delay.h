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

/// @tparam T Must be one of drake's default scalar types.
template <typename T>
class DiscreteTimeDelay final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeDelay)

  /// Constructs a DiscreteTimeDelay system updating every @p update_sec and
  /// delaying a vector-valued input of size @p vector_size for
  /// @p delay_timesteps number of updates.
  DiscreteTimeDelay(double update_sec, int delay_timesteps, int vector_size)
      : DiscreteTimeDelay(update_sec, delay_timesteps, vector_size, nullptr) {}

  /// Constructs a DiscreteTimeDelay system updating every @p update_sec and
  /// delaying a abstract-valued input of type @p abstract_model_value for
  /// @p delay_timesteps number of updates.
  DiscreteTimeDelay(double update_sec, int delay_timesteps,
                    const AbstractValue& abstract_model_value)
      : DiscreteTimeDelay(update_sec, delay_timesteps, -1,
                          abstract_model_value.Clone()) {}

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeDelay(const DiscreteTimeDelay<U>& other);

  ~DiscreteTimeDelay() final = default;

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

 private:
  // Allow different specializations to access each other's private data.
  template <typename U> friend class DiscreteTimeDelay;

  // All of the other constructors delegate here.
  DiscreteTimeDelay(double update_sec, int delay_timesteps, int vector_size,
                    std::unique_ptr<const AbstractValue> model_value);

  // Override feedthrough detection to avoid the need for `DoToSymbolic()`.
  optional<bool> DoHasDirectFeedthrough(int input_port,
                                        int output_port) const final;

  // Sets the output port value to the properly delayed vector value.
  void CalcVectorOutput(const Context<T>& context,
                        BasicVector<T>* output) const;

  // Saves the input port into the discrete vector-valued state,
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>& events,
      DiscreteValues<T>* discrete_state) const final;

  // Same as `DoCalcVectorOutput`, but for abstract values.
  void CalcAbstractOutput(const Context<T>& context,
                          AbstractValue* output) const;

  // Same as `DoCalcDiscreteVariablesUpdate`, but for abstract values.
  void DoCalcUnrestrictedUpdate(
      const Context<T>& context,
      const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
      State<T>* state) const final;

  bool is_abstract() const { return abstract_model_value_ != nullptr; }

  double update_sec_{};
  int delay_timesteps_{};
  int vector_size_{};
  std::unique_ptr<const AbstractValue> abstract_model_value_;
};

}  // namespace systems
}  // namespace drake
