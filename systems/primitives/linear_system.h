#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
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
/// @tparam_default_scalar
/// @ingroup primitive_systems
///
/// @see AffineSystem
/// @see MatrixGain
template <typename T>
class LinearSystem : public AffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystem)

  /// Constructs a %LinearSystem with a fixed set of coefficient matrices `A`,
  /// `B`,`C`, and `D`.
  /// The coefficient matrices must obey the following dimensions:
  /// | Matrix  | Num Rows    | Num Columns |
  /// |:-------:|:-----------:|:-----------:|
  /// | A       | num states  | num states  |
  /// | B       | num states  | num inputs  |
  /// | C       | num outputs | num states  |
  /// | D       | num outputs | num inputs  |
  ///
  /// Subclasses must use the protected constructor, not this one.
  LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               double time_period = 0.0);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit LinearSystem(const LinearSystem<U>&);

  /// Creates a unique pointer to LinearSystem<T> by decomposing @p dynamics and
  /// @p outputs using @p state_vars and @p input_vars.
  ///
  /// @throws std::runtime_error if either @p dynamics or @p outputs is not
  /// linear in @p state_vars and @p input_vars.
  static std::unique_ptr<LinearSystem<T>> MakeLinearSystem(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& output,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& state_vars,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& input_vars,
      double time_period = 0.0);

 protected:
  /// Constructor that specifies scalar-type conversion support.
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  LinearSystem(SystemScalarConverter converter,
               const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D, double time_period);
};

/// Base class for a discrete or continuous linear time-varying (LTV) system.
///
/// If `time_period > 0.0`, the system will have the following discrete-time
/// state update:
///   @f[ x(t+h) = A(t) x(t) + B(t) u(t), @f]
/// where `h` is the time_period.  If `time_period == 0.0`, the system will have
/// the following continuous-time state update:
///   @f[ \dot{x}(t) = A(t) x(t) + B(t) u(t), @f]
///
/// both with the output:
///   @f[ y(t) = C(t) x(t) + D(t) u(t). @f]
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class TimeVaryingLinearSystem : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeVaryingLinearSystem)

 protected:
  /// Constructor.
  ///
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  /// @param num_states size of the system's state vector
  /// @param num_inputs size of the system's input vector
  /// @param num_outputs size of the system's output vector
  /// @param time_period discrete update period, or 0.0 to use continuous time
  TimeVaryingLinearSystem(SystemScalarConverter converter, int num_states,
                          int num_inputs, int num_outputs, double time_period)
      : TimeVaryingAffineSystem<T>(std::move(converter), num_states, num_inputs,
                                   num_outputs, time_period) {}

 private:
  // N.B. A linear system is simply a restricted form of an affine system with
  // the affine terms set to zero.  The following adds this restriction.
  VectorX<T> f0(const T&) const final {
    return VectorX<T>::Zero(this->num_states());
  }
  VectorX<T> y0(const T&) const final {
    return VectorX<T>::Zero(this->num_outputs());
  }
};

/// Takes the first-order Taylor expansion of a System around a nominal
/// operating point (defined by the Context).
///
/// This method currently supports linearizing around at most a single vector
/// input port and at most a single vector output port.  For systems with
/// more ports, use @p input_port_index and @p output_port_index to select
/// the input for the newly constructed system.  Any additional _vector_
/// input ports will be treated as constants (fixed at the value specified in
/// `context`). Abstract-valued input ports must be unconnected (i.e., the
/// system must treat the port as optional and it must be unused).
///
/// @param system The system or subsystem to linearize.
/// @param context Defines the nominal operating point about which the system
/// should be linearized.  See note below.
/// @param input_port_index A valid input port index for @p system or
/// InputPortSelection.  All other inputs are assumed to be fixed to the
/// value described by the @p context. @default kUseFirstInputIfItExists.
/// @param output_port_index A valid output port index for @p system or
/// an OutputPortSelection. @default kUseFirstOutputIfItExists.
/// @param equilibrium_check_tolerance Specifies the tolerance on ensuring that
/// the derivative vector isZero at the nominal operating point.  @default 1e-6.
/// @returns A LinearSystem that approximates the original system in the
/// vicinity of the operating point.  See note below.
/// @throws std::runtime_error if the operating point is not an
/// equilibrium point of the system (within the specified tolerance)
/// @throws std::runtime_error if the system is not (only)
/// continuous or (only) discrete time with a single periodic update.
///
/// @note All _vector_ inputs in the system must be connected, either to the
/// output of some upstream System within a Diagram (e.g., if system is a
/// reference to a subsystem in a Diagram), or to a constant value using, e.g.
/// `port.FixValue(context, default_input)`. Any _abstract_ inputs in the
/// system must be unconnected (the port must be both optional and unused).
///
/// @note The inputs, states, and outputs of the returned system are NOT the
/// same as the original system.  Denote x0,u0 as the nominal state and input
/// defined by the Context, and y0 as the value of the output at (x0,u0),
/// then the created systems inputs are (u-u0), states are (x-x0), and
/// outputs are (y-y0).
///
/// @note This method does *not* (yet) set the initial conditions (default nor
/// random) of the LinearSystem based on `system`.
///
/// @ingroup primitive_systems
///
std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index =
        InputPortSelection::kUseFirstInputIfItExists,
    std::variant<OutputPortSelection, OutputPortIndex> output_port_index =
        OutputPortSelection::kUseFirstOutputIfItExists,
    double equilibrium_check_tolerance = 1e-6);

/// A first-order Taylor series approximation to a @p system in the neighborhood
/// of an arbitrary point.  When Taylor-expanding a system at a non-equilibrium
/// point, it may be represented either of the form:
///   @f[ \dot{x} - \dot{x}_0 = A (x - x_0) + B (u - u_0), @f]
/// for continuous time, or
///   @f[ x[n+1] - x_0[n+1] = A (x[n] - x_0[n]) + B (u[n] - u_0[n]), @f]
/// for discrete time.  As above, we denote @f$ x_0, u_0 @f$ to be the nominal
/// state and input at the provided @p context.  The system description is
/// affine when the terms @f$ \dot{x}_0 - A x_0 - B u_0 @f$ and @f$ x_0[n+1] -
/// A x_0[n] - B u_0[n] @f$ are nonzero.
///
/// More precisely, let x be a state and u be an input.  This function returns
/// an AffineSystem of the form:
///   @f[ \dot{x} = A x + B u + f_0, @f] (CT)
///   @f[ x[n+1] = A x[n] + B u[n] + f_0, @f] (DT)
/// where @f$ f_0 = \dot{x}_0 - A x_0 - B u_0 @f$ (CT) and
/// @f$ f_0 = x_0[n+1] - A x[n] - B u[n] @f$ (DT).
///
/// This method currently supports approximating around at most a single vector
/// input port and at most a single vector output port.  For systems with
/// more ports, use @p input_port_index and @p output_port_index to select
/// the input for the newly constructed system.  Any additional input ports
/// will be treated as constants (fixed at the value specified in @p context).
///
/// @param system The system or subsystem to linearize.
/// @param context Defines the nominal operating point about which the system
/// should be linearized.
/// @param input_port_index A valid input port index for @p system or
/// InputPortSelection. @default kUseFirstInputIfItExists.
/// @param output_port_index A valid output port index for @p system or
/// OutputPortSelection. @default kUseFirstOutputIfItExists.
/// @returns An AffineSystem at this linearization point.
/// @throws if any abstract inputs are connected, if any
///         vector-valued inputs are unconnected, if the system is not (only)
///         continuous or not (only) discrete time with a single periodic
///         update.
///
/// @note x, u and y are in the same coordinate system as the original
/// @p system, since the terms involving @f$ x_0, u_0 @f$ reside in
/// @f$ f_0 @f$.
///
/// @note This method does *not* (yet) set the initial conditions (default nor
/// random) of the AffineSystem based on `system`.
///
/// @ingroup primitive_systems
///
// Note: The TypeSafeIndices (InputPortIndex and OutputPortIndex) didn't let
// me handle the additional options without a lot of boilerplate.
std::unique_ptr<AffineSystem<double>> FirstOrderTaylorApproximation(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index =
        InputPortSelection::kUseFirstInputIfItExists,
    std::variant<OutputPortSelection, OutputPortIndex> output_port_index =
        OutputPortSelection::kUseFirstOutputIfItExists);

/// Returns the controllability matrix:  R = [B, AB, ..., A^{n-1}B].
/// @ingroup control_systems
Eigen::MatrixXd ControllabilityMatrix(const LinearSystem<double>& sys);

/// Returns true iff the controllability matrix is full row rank.
/// @ingroup control_systems
bool IsControllable(const LinearSystem<double>& sys,
                    std::optional<double> threshold = std::nullopt);

/// Returns the observability matrix: O = [ C; CA; ...; CA^{n-1} ].
/// @ingroup estimator_systems
Eigen::MatrixXd ObservabilityMatrix(const LinearSystem<double>& sys);

/// Returns true iff the observability matrix is full column rank.
/// @ingroup estimator_systems
bool IsObservable(const LinearSystem<double>& sys,
                  std::optional<double> threshold = std::nullopt);

}  // namespace systems
}  // namespace drake
