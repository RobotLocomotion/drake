#pragma once

#include "drake/systems/framework/primitives/linear_system.h"

namespace drake {
namespace systems {

// TODO(liang.fok) Rename this system to be "Gain" and replace the existing
// "Gain" system. See: #3922.

/// A MIMO (Multi-Input Multi-Output) gain system that is a specialization of a
/// LinearSystem where coefficient matrices `A`, `B`, and `C` are all zero.
/// Given an input signal `u` and a state `x` the output of this system, `y`,
/// is:
///
/// <pre>
///   y = Du
/// </pre>
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
///
/// @see AffineSystem
/// @see LinearSystem
template <typename T>
class MimoGain: public LinearSystem<T> {
 public:
  /**
   * A constructor where the gain matrix `D` is a square identity matrix of size
   * @p size.
   */
  explicit MimoGain(int size);

  /**
   * A constructor where the gain matrix `D` is @p D.
   */
  explicit MimoGain(const Eigen::Ref<const MatrixX<T>>& D);
};

}  // namespace systems
}  // namespace drake
