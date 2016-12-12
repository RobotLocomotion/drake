#pragma once

#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace systems {

/// A discrete OR continuous linear system.
///
/// If time_period>0.0, then the linear system will have the following discrete-
/// time state update:
///   @f[ x[n+1] = A x[n] + B u[n], @f]
///
/// or if time_period==0.0, then the linear system will have the following
/// continuous-time state update:
///   @f[\dot{x} = A x + B u. @f]
///
/// In both cases, the system will have the output:
///   @f[y = C x + D u, @f]
/// where `u` denotes the input vector, `x` denotes the state vector, and
/// `y` denotes the output vector.
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
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               double time_period = 0.0);
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

/// Returns the controllability matrix:  R = [B, AB, ..., A^{n-1}B].
Eigen::MatrixXd ControllabilityMatrix(const LinearSystem<double>& sys);

/// Returns true iff the controllability matrix is full row rank.
bool IsControllable(const LinearSystem<double>& sys,
                    double threshold = Eigen::Default);

/// Returns the observability matrix: O = [ C; CA; ...; CA^{n-1} ].
Eigen::MatrixXd ObservabilityMatrix(const LinearSystem<double>& sys);

/// Returns true iff the observability matrix is full column rank.
bool IsObservable(const LinearSystem<double>& sys,
                  double threshold = Eigen::Default);

}  // namespace systems
}  // namespace drake
