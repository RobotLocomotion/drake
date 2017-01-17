#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// An element-wise variable hard saturation block with inputs signal `u`,
/// saturation values @f$ u_{min} @f$ and/or @f$ u_{max} @f$, and output
/// `y ` respectively as in:
///
///   @f[ y = u, u_{min} < u < u_{min} @f]
///   @f[ y = u_{min}, u \le u_{min} @f]
///   @f[ y = u_{max}, u \ge u_{max}  @f]
///
/// The input to this system directly feeds through to its output.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// Note that @f$ u_{min} @f$, and @f$ u_{max} @f$, and @f$ u @f$ are all
/// vectors of same dimension, and the following condition holds elementwise in
/// runtime.
///
///   @f[ u_{min} <=  u_{max} @f]
///
/// Atleast one or both of the max_value_port or the min_value_port must
/// be enabled. If the min_value_port is not enabled, then
/// @f$ u_{min} = \infty @f$, and likewise, if max_value_port is not enabled,
/// then
/// @f$ u_{max} = \infty @f$.
///
/// For a constant saturation block @see systems::Saturation
///
/// @ingroup primitive_systems
template <typename T>
class VariableSaturation : public LeafSystem<T> {
 public:
  /// Constructs a %Saturation system where the upper and lower values are
  /// represented by scalars.
  ///
  /// @param[in] enable_max_value_port Enables the max value port
  /// @param[in] enable_min_value_port Enables the min value port
  /// @param[in] input_size sets size of the input and output ports.
  ///
  /// Atleast one of enable_max_value_port or enable_min_value_port
  /// must be set and input_size must be a positive integer.
  explicit VariableSaturation(bool enable_max_value_port,
                              bool enable_min_value_port, int input_size);

  /// Returns the input port.
  const InputPortDescriptor<T>& get_input_port() const {
    return System<T>::get_input_port(input_port_index_);
  }

  /// Returns the min value port.
  const InputPortDescriptor<T>& get_min_value_port() const {
    DRAKE_THROW_UNLESS(kMinValuePortEnabled_);
    return System<T>::get_input_port(min_value_port_index_);
  }

  /// Returns the max value port.
  const InputPortDescriptor<T>& get_max_value_port() const {
    DRAKE_THROW_UNLESS(kMaxValuePortEnabled_);
    return System<T>::get_input_port(max_value_port_index_);
  }

  /// Returns the input port index.
  int get_input_port_index() const { return input_port_index_; }

  /// Returns the min value port index.
  int get_min_value_port_index() const { return min_value_port_index_; }

  /// Returns the max value port index.
  int get_max_value_port_index() const { return max_value_port_index_; }

  /// Returns the output port.
  const OutputPortDescriptor<T>& get_output_port() const {
    return System<T>::get_output_port(output_port_index_);
  }

  /// Returns true if min_value_port is enabled.
  bool is_min_value_port_enabled() const { return kMinValuePortEnabled_; }

  /// Returns true if max_value_port is enabled.
  bool is_max_value_port_enable() const { return kMaxValuePortEnabled_; }

  /// Returns the size.
  int get_size() const { return input_size_; }
  // TODO(naveenoid) : Add a witness function for capturing events when
  // saturation limits are reached.

 protected:
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

 private:
  int input_port_index_{};
  int min_value_port_index_{};
  int max_value_port_index_{};
  int output_port_index_{};
  const bool kMinValuePortEnabled_{false};
  const bool kMaxValuePortEnabled_{false};
  int input_size_;
};

}  // namespace systems
}  // namespace drake
