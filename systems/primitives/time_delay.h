#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// @tparam T Must be one of drake's default scalar types.
template <typename T>
class TimeDelay : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeDelay)

  /// Constructs a TimeDelay system with the given @p delay_sec and updating
  /// every @p update_sec, over a vector-valued input of size `vector_size`.
  /// The default initial value for this system will be zero.
  TimeDelay(double delay_sec, double update_sec, int vector_size)
      : TimeDelay(delay_sec, update_sec, vector_size, nullptr) {}

  /// Constructs a TimeDelay system with the given @p delay_sec and updating
  /// every @p update_sec, over a abstract-valued input `abstract_model_value`.
  /// The default initial value for this system will be `abstract_model_value`.
  TimeDelay(double delay_sec, double update_sec,
      const AbstractValue& abstract_model_value)
      : TimeDelay(delay_sec, update_sec, -1, abstract_model_value.Clone()) {}

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit TimeDelay(const TimeDelay<U>& other);

  ~TimeDelay() override {}

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

  // Sets the output port value to the properly delayed vector value.
  void DoCalcVectorOutput(
      const Context<T>& context, BasicVector<T>* output) const;

  // Same as `DoCalcVectorOutput`, but for abstract values.
  void DoCalcAbstractOutput(
      const Context<T>& context, AbstractValue* output) const;

  // Saves the input port into the delay buffer.
  void DoCalcUnrestrictedUpdate(const Context<T>& context,
      const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
      State<T>* state) const override;

 private:
  bool is_abstract() const { return abstract_model_value_ != nullptr; }

  TimeDelay(double delay_sec, double update_sec, int vector_size,
      std::unique_ptr<const AbstractValue> model_value);

  // Allow different specializations to access each other's private data.
  template <typename U> friend class TimeDelay;

  double delay_sec_{};
  double update_sec_{};
  std::unique_ptr<const AbstractValue> abstract_model_value_;
};

}  // namespace systems
}  // namespace drake

// DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class ::drake::systems::TimeDelay)
