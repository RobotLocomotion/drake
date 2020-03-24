#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// An element-wise hard saturation block with inputs signal `u`,
/// saturation values @f$ u_{min} @f$ and/or @f$ u_{max} @f$, and output
/// `y` respectively as in:
///
///   @f[ y = u, u_{min} < u < u_{min} @f]
///   @f[ y = u_{min}, u \le u_{min} @f]
///   @f[ y = u_{max}, u \ge u_{max}  @f]
///
/// The input to this system directly feeds through to its output.
///
/// Note that @f$ u_{min} @f$, and @f$ u_{max} @f$, and @f$ u @f$ are all
/// vectors of same dimension, and the following condition holds elementwise in
/// runtime.
///
///   @f[ u_{min} <=  u_{max} @f]
///
/// The quantities @f$ u_{min} @f$, and @f$ u_{max} @f$ can be supplied as
/// inputs in separate ports or be initialised as constants using the
/// appropriate constructor by passing their default value. If these quantities
/// are not defined as constants but they are not connected to appropriate
/// sources, their values are taken by default to be
/// @f$ u_{min} = -\infty @f$, and  @f$ u_{max} = \infty @f$ respectively.
/// In this "variable" configuration, at least one of the input ports must be
/// connected.
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class Saturation final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Saturation)

  /// Constructs a variable %Saturation system where the upper and lower values
  /// are represented by vectors of identical size and can be supplied via the
  /// max_value_port and min_value_port respectively.
  ///
  /// @param[in] input_size sets size of the input and output ports.
  ///
  /// Please consult this class's description for the requirements of
  /// @p u_min and @p u_max to be supplied via the corresponding ports.
  explicit Saturation(int input_size);

  /// Constructs a constant %Saturation system where the upper and lower
  /// values are represented by vectors of identical size supplied via this
  /// constructor.
  ///
  /// @param[in] u_min the lower (vector) limit to the
  /// saturation.
  /// @param[in] u_max the upper (vector) limit to the
  /// saturation.
  ///
  /// Please consult this class's description for the requirements of
  /// @p u_min and @p u_max.
  Saturation(const VectorX<T>& min_value, const VectorX<T>& max_value);

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

  /// Returns the input port.
  const InputPort<T>& get_input_port() const {
    return System<T>::get_input_port(input_port_index_);
  }

  /// Returns the min value port.
  const InputPort<T>& get_min_value_port() const {
    DRAKE_THROW_UNLESS(min_max_ports_enabled_);
    return System<T>::get_input_port(min_value_port_index_);
  }

  /// Returns the max value port.
  const InputPort<T>& get_max_value_port() const {
    DRAKE_THROW_UNLESS(min_max_ports_enabled_);
    return System<T>::get_input_port(max_value_port_index_);
  }

  /// Returns the output port.
  const OutputPort<T>& get_output_port() const {
    return System<T>::get_output_port(output_port_index_);
  }

  /// Returns the size.
  int get_size() const { return input_size_; }
  // TODO(naveenoid) : Add a witness function for capturing events when
  // saturation limits are reached.

 private:
  void CalcSaturatedOutput(const Context<T>& context,
                           BasicVector<T>* output_vector) const;

  int input_port_index_{};
  int min_value_port_index_{};
  int max_value_port_index_{};
  int output_port_index_{};
  const bool min_max_ports_enabled_{false};
  const int input_size_{};
  const VectorX<T> max_value_;
  const VectorX<T> min_value_;
};

}  // namespace systems
}  // namespace drake
