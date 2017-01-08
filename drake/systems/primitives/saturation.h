#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// An element-wise hard saturation block with input `u` and output
/// `y ` with lower and upper saturation limit values `\sigma_l`, and
/// `\sigma_u` respectively as in :
///
///   @f[ y = u, \sigma_l < u < \sigma_u @f]
///   @f[ y = \sigma_l, u \le \sigma_l @f]
///   @f[ y = \sigma_u, u \ge \sigma_u  @f]
///
/// The input to this system directly feeds through to its output.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in drakeSystemFramework.
///
/// @param `\sigma_l` the lower saturation limit.
/// @param `\sigma_u` the lower saturation limit.
///
/// Note that `\sigma_l`, and `\sigma_u`, and `u` are all of same dimension,
/// and the following condition holds along each dimension :
/// @f[ \sigma_l <  \sigma_u @f]
///
/// @ingroup primitive_systems
template <typename T>
class Saturation : public LeafSystem<T> {
 public:
  /// Constructs a %Saturation system where the upper and lower values are
  /// represented by scalars.
  ///
  /// @param[in] sigma_lower the lower (scalar) limit to the saturation.
  /// @param[in] sigma_upper the upper (scalar) limit to the saturation.
  Saturation(const T& sigma_lower, const T& sigma_upper);

  /// Constructs a %Saturation system where the upper and lower values are
  /// represented by vectors of identical size.
  ///
  /// @param[in] sigma_lower the lower (vector) limit to the saturation.
  /// @param[in] sigma_upper the upper (vector) limit to the saturation.
  explicit Saturation(const Eigen::Ref<const VectorX<T>>& sigma_lower,
                      const Eigen::Ref<const VectorX<T>>& sigma_upper);

  /// Returns the input port.
  const InputPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const OutputPortDescriptor<T>& get_output_port() const;

  /// A scalar valued getter for the lower limit of the `Saturation` block.
  /// Throws
  /// an error in case the `Saturation` block is initialised with vector limits.
  const T& get_sigma_lower() const;

  /// A scalar valued getter for the upper limit of the `Saturation` block.
  /// Throws
  /// an error in case the `Saturation` block is initialised with vector limits.
  const T& get_sigma_upper() const;

  /// A vector getter for the lower limit of the `Saturation` block.
  const VectorX<T>& get_sigma_lower_vector() const { return kSigmaLower; }

  /// A vector getter for the upper limit of the `Saturation` block.
  const VectorX<T>& get_sigma_upper_vector() const { return kSigmaUpper; }

 protected:
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  const VectorX<T> kSigmaLower;
  const VectorX<T> kSigmaUpper;
};

}  // namespace systems
}  // namespace drake
