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
/// @system
/// name: MatrixGain
/// input_ports:
/// - u0
/// output_ports:
/// - y0
/// @endsystem
///
/// @tparam_default_scalar
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
