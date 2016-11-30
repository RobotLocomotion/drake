#pragma once

#include "drake/systems/framework/primitives/linear_system.h"

namespace drake {
namespace systems {

/// A system that specializes LinearSystem by setting coefficient matrices `A`,
/// `B`, and `C` to all be zero. Thus, the only non-zero coefficient matrix is
/// `D`. Specifically, given an input signal `u` and a state `x`, the output of
/// this system, `y`, is:
///
/// @f[
///   y = D u
/// @f]
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
class MatrixGain : public LinearSystem<T> {
 public:
  /**
   * A constructor where the gain matrix `D` is a square identity matrix of size
   * @p size.
   */
  explicit MatrixGain(int size);

  /**
   * A constructor where the gain matrix `D` is @p D.
   */
  explicit MatrixGain(const Eigen::MatrixXd& D);
};

}  // namespace systems
}  // namespace drake
