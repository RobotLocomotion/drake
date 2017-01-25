#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// An element-wise hard saturation block with input `u` and output
/// `y ` with lower and upper saturation limit values @f$ u_{min} @f$, and
/// @f$ u_{max} @f$ respectively as in:
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
/// vectors of same dimension, and the following condition holds elementwise.
///
///   @f[ u_{min} <=  u_{max} @f]
///
/// @ingroup primitive_systems
template <typename T>
class Saturation : public LeafSystem<T> {
 public:
  /// Constructs a %Saturation system where the upper and lower values are
  /// represented by scalars.
  ///
  /// @param[in] u_min the lower (scalar) limit to the
  /// saturation.
  /// @param[in] u_max the upper (scalar) limit to the
  /// saturation.
  ///
  /// Please consult this class' description for the requirements of
  /// @p u_min and @p u_max.
  Saturation(const T& u_min, const T& u_max);

  /// Constructs a %Saturation system where the upper and lower values are
  /// represented by vectors of identical size.
  ///
  /// @param[in] u_min the lower (vector) limit to the
  /// saturation.
  /// @param[in] u_max the upper (vector) limit to the
  /// saturation.
  ///
  /// Please consult this class' description for the requirements of
  /// @p u_min and @p u_max.
  explicit Saturation(const Eigen::Ref<const VectorX<T>>& u_min,
                      const Eigen::Ref<const VectorX<T>>& u_max);

  /// Returns the input port.
  const InputPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const OutputPortDescriptor<T>& get_output_port() const;

  /// Returns the lower limit `u_{min}` as a scalar value. Aborts if the lower
  /// limit cannot be represented by a single scalar value. This can occur if
  /// the lower limit is a vector containing more than one value.
  const T& get_u_min_scalar() const;

  /// Returns the upper limit `u_{max}` as a scalar value. Aborts if the upper
  /// limit cannot be represented by a single scalar value. This can occur if
  /// the upper limit is a vector containing more than one value.
  const T& get_u_max_scalar() const;

  /// Returns the lower limit `u_{min}` as a vector value.
  const VectorX<T>& u_min() const { return u_min_; }

  /// Returns the upper limit `u_{max}` as a vector value.
  const VectorX<T>& u_max() const { return u_max_; }

  // TODO(naveenoid) : Add a witness function for capturing events when
  // saturation limits are reached.

 protected:
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  const VectorX<T> u_min_;
  const VectorX<T> u_max_;

 private:
  int input_port_index_{};
  int output_port_index_{};
};

}  // namespace systems
}  // namespace drake
