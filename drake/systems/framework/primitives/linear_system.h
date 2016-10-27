#pragma once

#include "drake/systems/framework/primitives/affine_system.h"

namespace drake {
namespace systems {

/// A continuous linear system that is a specialization of an affine system
/// where the inital time derivative of the system state `xDot0`
/// and the initial system output `y0` are both zero. Given an input
/// signal `u` and a state `x` the output of this sytem 'y' is:
/// @f[\dot{x} = Ax + Bu @f] \newline
/// @f[y = Cx + Du @f]
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class LinearSystem : public AffineSystem<T> {
 public:
  /// Constructs a Linear system with a fixed set of coefficient matrices `A`,
  /// `B`,`C`, and `D`.
  /// The coefficient matrices must obey the following dimensions :
  /// | Matrix  | Num Rows    | Num Columns |
  /// |:-------:|:-----------:|:-----------:|
  /// | A       | num states  | num states  |
  /// | B       | num states  | num inputs  |
  /// | C       | num outputs | num states  |
  /// | D       | num outputs | num inputs  |
  LinearSystem(const Eigen::Ref<const MatrixX<T>>& A,
               const Eigen::Ref<const MatrixX<T>>& B,
               const Eigen::Ref<const MatrixX<T>>& C,
               const Eigen::Ref<const MatrixX<T>>& D);
};

}  // namespace systems
}  // namespace drake
