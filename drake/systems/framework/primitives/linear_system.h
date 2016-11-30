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

/// Takes the first-order Taylor expansion of a System around a nominal
/// operating point (defined by the Context).
///
/// @param system The system or subsystem to linearize.
/// @param context Defines the nominal operating point about which the system
/// should be linearized.  See note below.
/// @param equilibrium_check_tolerance Specifies the tolerance on ensuring that
/// the derivative vector isZero at the nominal operating point.  @default 1e-6.
/// @returns A LinearSystem that approximates the original system in the
/// vicinity of the operating point.  See note below.
/// @throws std::runtime_error if the system the operating point is not an
/// equilibrium point of the system (within the specified tolerance)
///
/// Note: The inputs in the Context must be connected, either to the
/// output of some upstream System within a Diagram (e.g., if system is a
/// reference to a subsystem in a Diagram), or to a constant value using, e.g.
///   context->FixInputPort(0,default_input);
///
/// Note: The inputs, states, and outputs of the returned system are NOT the
/// same as the original system.  Denote x0,u0 as the nominal state and input
/// defined by the Context, and y0 as the value of the output at (x0,u0),
/// then the created systems inputs are (u-u0), states are (x-x0), and
/// outputs are (y-y0).

std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    const double equilibrium_check_tolerance = 1e-6);

}  // namespace systems
}  // namespace drake
