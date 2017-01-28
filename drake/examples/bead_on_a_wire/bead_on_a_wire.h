#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace bead_on_a_wire {

/// Dynamical system of a point mass constrained to lie on a wire. The system
/// is currently frictionless. The equation for the wire can be provided
/// parametrically *by the user*. Dynamics equations can be computed in both
/// minimal coordinates or absolute coordinates (with Lagrange Multipliers).
/// Equations for the dynamics in absolute coordinates are provided by
/// R. Rosales, "Bead Moving Along a Thin, Rigid Wire". Available from:
/// https://ocw.mit.edu/courses/mathematics/18-385j-nonlinear-dynamics-and-chaos-fall-2004/lecture-notes/bead_on_wire.pdf 
///
/// The presence of readily available solutions coupled with the potential
/// for highly irregular geometric constraints (which can be viewed 
/// as complex contact constraints), make this a powerful example.
///
/// The dynamic equations for the bead in minimal coordinates comes from
/// Lagrangian Dynamics of the "first kind": forming the Lagrangian and
/// using the Euler-Lagrange equation. The potential energy (V) of the bead with
/// respect to the parametric function f(s) is:
/// <pre>
/// 1/2⋅f₃(s)⋅ag
/// </pre>
/// where `ag` is the magnitude (i.e., non-negative value) of the acceleration
/// due to gravity. The velocity of the bead is df/ds⋅ṡ, and the kinetic
/// energy of the bead (T) is therefore 1/2⋅(df/ds⋅ṡ)².
///
/// The Lagrangian is then defined as:
/// <pre>
/// ℒ = T - V = 1/2⋅(df/ds)²⋅ṡ² - f₃(s)⋅ag
/// </pre>
/// And Lagrangian Dynamics specifies that the dynamics for the system are
/// defined by:
/// <pre>
/// ∂ℒ/∂s - d/dt ∂ℒ/∂ṡ =ṡ⋅
/// </pre>
/// where τ is the generalized force on the system. Thus:
/// <pre>
/// df(s)/ds⋅ṡ⋅d²f/ds² - (df(s)/ds)₃⋅ag - (df/ds)²⋅dṡ/dt = τ
/// </pre>
/// The dynamic equations for the bead in absolute coordinates comes from the
/// Lagrangian Dynamics of the "second kind" (i.e., by formulating the problem
/// as an Index-3 DAE):
/// <pre>
/// d²x/dt² = fg + fext + J^T*λ
/// g(x) = 0
/// </pre>
/// where `x` is the three dimensional position of the bead, `fg` is a three
/// dimensional gravitational acceleration vector, `fext` is a three dimensional
/// vector of "external" (user applied) forces acting on the bead, and J ≡ ∂g/∂x
/// is the Jacobian matrix of the positional constraints (computed with respect
/// to the bead location). We define `g(x) : ℝ³ → ℝ³` as the following function:
/// <pre>
/// g(x) = f(f⁻¹(x)) - x
/// </pre>
/// where f⁻¹ : ℝ³ → ℝ maps points in Cartesian space to wire parameters.
/// g(x) = 0 will only be satisfied if x corresponds to a point on the wire.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
///
/// They are already available to link against in the library implementation.
///
/// Inputs: One input (generalized external force) for the bead on a wire
///         simulated in minimal coordinates, three inputs (three dimensional
///         external force applied to the center of mass) for the bead on a
///         wire simulated in absolute coordinates.
///
/// States: Two state variables (arc length of the bead along the wire and the
///         velocity of the bead along the wire) for the bead simulated in
///         minimal coordinates; 3D position (state indices 0,1, and 2), and
///         3D linear velocity (state indices 3, 4, and 5) in units of m and
///         m/s, respectively, for the bead simulated in absolute coordinates.
///
/// Outputs: same as state.
template <typename T>
class BeadOnAWire : public systems::LeafSystem<T> {
 public:
  typedef Eigen::AutoDiffScalar<drake::Vector1d> AScalar;
  typedef Eigen::AutoDiffScalar<Eigen::Matrix<AScalar, 1, 1>> DScalar;

  enum CoordinateType {
    // Coordinate representation will be wire parameter.
    kMinimalCoordinates,

    // Coordinate representation will be the bead location (in 3D).
    kAbsoluteCoordinates
  };

  /// Constructs the object using either minimal or absolute coordinates (the
  /// latter uses the method of Lagrange Multipliers to compute the time
  /// derivatives).
  BeadOnAWire(CoordinateType type);

  void DoCalcOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  /// Sets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  void set_gravitational_acceleration(double g) { g_ = g; }

  /// Gets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  double get_gravitational_acceleration() const { return g_; }

  /// Computes the change in velocity that would occur by applying impulsive
  /// force lambda to the constraints.
  Eigen::VectorXd CalcVelocityChangeFromConstraintImpulses(
              const systems::Context<T>& context,
              const Eigen::MatrixXd& J,
              const Eigen::VectorXd& lambda) const;

  /// Allows the user to reset the wire parameter function (which points to the
  /// sinusoidal function by default.
  /// @param f The pointer to a function that takes a wire parameter
  ///          (0 <= s <= 1) as default and outputs a point in 3D as output.
  /// @throws std::logic_error if f is a nullptr (the function must always be
  ///         set).
  void reset_wire_parameter_function(std::function<Eigen::Matrix<DScalar,3,1>(
      const DScalar&)> f) {
    if (!f) throw std::logic_error("Function must be non-null.");
    f_ = f;
  }

  /// Allows the user to reset the inverse wire parameter function (which
  /// points to the inverse sinusoidal function by default).
  /// @param inv_f The pointer to a function that takes a point in 3D as input
  ///              and outputs a floating point scalar as output. nullptr is
  ///              an acceptable value (BeadOnAWire will use a generic inversion
  ///              routine).
  void reset_inverse_wire_parameter_function(std::function<DScalar(
      const Eigen::Matrix<DScalar,3,1>&)> inv_f) {
    inv_f_ = inv_f;
  }

  /// Example wire parametric function for the bead on a wire example. The
  /// exact function definition is:
  /// <pre>
  ///        | cos(s) |
  /// f(s) = | sin(s) |
  ///        | s      |
  static Eigen::Matrix<DScalar, 3, 1> sinusoidal_function(const DScalar& s);

  /// Example wire parametric function for the bead on a wire example. The
  /// exact function definition is:
  /// <pre>
  ///        | s |
  /// f(s) = | 0 |
  ///        | 0 |
  static Eigen::Matrix<DScalar, 3, 1> linear_function(const DScalar& s);

  /// Inverse parametric function for the bead on a wire system that uses the
  /// sinusoidal parametric example function.
  static DScalar inverse_sinusoidal_function(const Vector3<DScalar>& v);

  /// Inverse parametric function for the bead on a wire system that uses the
  /// linear parametric example function.
  static DScalar inverse_linear_function(const Vector3<DScalar>& v);

 protected:
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

  /// Gets the number of constraint equations.
  int do_get_num_constraint_equations(const systems::Context<T>& context) const
  override;

  /// Evaluates the constraint equations for a bead in absolute coordinates.
  /// This method is computationally expensive. To evaluate these equations,
  /// the method locates the variable s that minimizes the univariate function
  /// ||f(s) - x||, where x is the location of the bead.
  /// @warning The solution returned by this approach is not generally a global
  ///          minimum.
  /// @TODO(edrumwri): Provide a special function that calculates the parameters
  ///                  for a given function?
  Eigen::VectorXd DoEvalConstraintEquations(
      const systems::Context<T>& context) const override;

  /// Computes the time derivative of each constraint equation, evaluated at
  /// the current generalized coordinates and generalized velocity.
  Eigen::VectorXd DoEvalConstraintEquationsDot(
      const systems::Context<T>& context) const override;

  /// Computes the change in generalized velocity from applying constraint
  /// impulses @p lambda.
  /// @param context The current state of the system.
  /// @param lambda The vector of constraint forces.
  Eigen::VectorXd
  DoCalcVelocityChangeFromConstraintImpulses(
      const systems::Context<T>& context, const Eigen::MatrixXd& J,
      const Eigen::VectorXd& lambda) const override;

 private:

  // The coordinate representation used for kinematics and dynamics.
  CoordinateType coordinate_type_;

  // The wire parameter function. See set_wire_parameter_function() for more
  // information. This pointer must always be empty
  std::function<Eigen::Matrix<DScalar, 3, 1>(const DScalar&)> f_{
      &sinusoidal_function};

  // The (optional) inverse of the wire parameter function.
  // This function takes a point in 3D as input and outputs a point
  // (0 <= s <= 1) as output. If this function is non-null, constraint
  // stabilization methods for DAEs can use it (via
  // EvaluateConstraintEquations()) to efficiently project a bead represented
  // in absolute coordinates back onto the wire. If this function is null,
  // EvaluateConstraintEquations() will use generic, presumably less efficient
  // methods instead. This function is not to nullptr by default.
  std::function<DScalar(const Eigen::Matrix<DScalar, 3, 1>&)> inv_f_{
      &inverse_sinusoidal_function};

  double g_{-9.81};
};

}  // namespace bead_on_a_wire
}  // namespace drake
