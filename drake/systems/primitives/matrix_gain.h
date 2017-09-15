#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/primitives/linear_system.h"

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
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
///
/// @see AffineSystem
/// @see LinearSystem
template <typename T>
class MatrixGain final : public LinearSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MatrixGain)

  /// A constructor where the gain matrix `D` is a square identity matrix of
  /// size @p size.
  explicit MatrixGain(int size);

  /// A constructor where the gain matrix `D` is @p D.
  explicit MatrixGain(const Eigen::MatrixXd& D);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MatrixGain(const MatrixGain<U>&);
};

}  // namespace systems
}  // namespace drake
