#pragma once

#include "drake/systems/framework/primitives/affine_system.h"

namespace drake {
namespace systems {

/// A continuous linear system that is a specialization of an AffineSystem
/// where the inital time derivative of the system state `xDot0`
/// and the initial system output `y0` are both zero. Given an input
/// signal `u` and a state `x` the output of this system 'y' is:
///
/// @f[\dot{x} = A x + B u @f]
/// @f[y = C x + D u @f]
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
/// @see MatrixGain
template <typename T>
class LinearSystem : public AffineSystem<T> {
 public:
  /// Constructs a %LinearSystem with a fixed set of coefficient matrices `A`,
  /// `B`,`C`, and `D`.
  /// The coefficient matrices must obey the following dimensions:
  /// | Matrix  | Num Rows    | Num Columns |
  /// |:-------:|:-----------:|:-----------:|
  /// | A       | num states  | num states  |
  /// | B       | num states  | num inputs  |
  /// | C       | num outputs | num states  |
  /// | D       | num outputs | num inputs  |
  LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D);
};

}  // namespace systems
}  // namespace drake
