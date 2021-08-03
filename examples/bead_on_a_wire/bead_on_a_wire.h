#pragma once

#include <Eigen/Core>

#include "drake/common/autodiff.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace bead_on_a_wire {

/// Dynamical system of a point (unit) mass constrained to lie on a wire. The
/// system is currently frictionless. The equation for the wire can be provided
/// parametrically by the user. Dynamics equations can be computed in both
/// minimal coordinates or absolute coordinates (with Lagrange Multipliers).
///
/// The presence of readily available solutions coupled with the potential
/// for highly irregular geometric constraints (which can be viewed
/// as complex contact constraints), can make this a challenging problem
/// to simulate.
///
/// The dynamic equations for the bead in minimal coordinates comes from
/// Lagrangian Dynamics of the "second kind": forming the Lagrangian and
/// using the Euler-Lagrange equation. The potential energy (V) of the bead with
/// respect to the parametric function f(s) : ℝ → ℝ³ is:
/// <pre>
/// -f₃(s)⋅ag
/// </pre>
/// where `ag` is the magnitude (i.e., non-negative value) of the acceleration
/// due to gravity. The velocity of the bead is df/ds⋅ṡ, and the kinetic
/// energy of the bead (T) is therefore 1/2⋅(df/ds⋅ṡ)².
///
/// The Lagrangian is then defined as:
/// <pre>
/// ℒ = T - V = 1/2⋅(df/ds)²⋅ṡ² + f₃(s)⋅ag
/// </pre>
/// And Lagrangian Dynamics specifies that the dynamics for the system are
/// defined by:
/// <pre>
/// ∂ℒ/∂s - d/dt ∂ℒ/∂ṡ = τ̇⋅
/// </pre>
/// where τ is the generalized force on the system. Thus:
/// <pre>
/// df(s)/ds⋅ṡ²⋅d²f/ds² + (df(s)/ds)₃⋅ag - (df/ds)²⋅dṡ/dt = τ
/// </pre>
/// The dynamic equations for the bead in absolute coordinates comes from the
/// Lagrangian Dynamics of the "first kind" (i.e., by formulating the problem
/// as an Index-3 DAE):
/// <pre>
/// d²x/dt² = fg + fext + Jᵀλ
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
/// where f⁻¹ : ℝ³ → ℝ maps points in Cartesian space to wire parameters (this
/// class expects that this inverse function may be ill-defined).
/// g(x) = 0 will only be satisfied if x corresponds to a point on the wire.
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
///
/// @tparam_double_only
template <typename T>
class BeadOnAWire : public systems::LeafSystem<T> {
 public:
  /// AutoDiff scalar used for constraint function automatic differentiation.
  /// The constraint function maps from an arc length (s) to a point in
  /// three-dimensional Cartesian space. This type represents the input
  /// variable s and- on return from that constraint function- would hold the
  /// first derivative of the constraint function computed with respect to s.
  typedef Eigen::AutoDiffScalar<drake::Vector1d> AScalar;

  /// AutoDiff scalar used for computing the second derivative of the constraint
  /// function using automatic differentiation. The constraint function maps
  /// from an arc length (s) to a point in three-dimensional Cartesian space.
  /// This type represents the input variable s and- on return from that
  /// constraint function- would hold the second derivative of the constraint
  /// function computed with respect to s.
  typedef Eigen::AutoDiffScalar<Eigen::Matrix<AScalar, 1, 1>> ArcLength;

  /// The type of coordinate representation to use for kinematics and dynamics.
  enum CoordinateType {
    /// Coordinate representation will be wire parameter.
    kMinimalCoordinates,

    /// Coordinate representation will be the bead location (in 3D).
    kAbsoluteCoordinates
  };

  /// Constructs the object using either minimal or absolute coordinates (the
  /// latter uses the method of Lagrange Multipliers to compute the time
  /// derivatives).
  explicit BeadOnAWire(CoordinateType type);

  /// Sets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  void set_gravitational_acceleration(double g) { g_ = g; }

  /// Gets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  double get_gravitational_acceleration() const { return g_; }

  /// Allows the user to reset the wire parameter function and its inverse
  /// (which points to helix_function and inverse_helix_function by
  /// default.
  /// @param f The pointer to a function that takes a wire parameter
  ///          (scalar `s`) as input and outputs a point in 3D as output.
  /// @param inv_f The pointer to a function that takes a point in 3D as input
  ///              and outputs a floating point scalar as output.
  /// @throws std::exception if f or inv_f is a nullptr (the functions must
  ///         always be set).
  void reset_wire_parameter_functions(
        std::function<Eigen::Matrix<ArcLength, 3, 1>(const ArcLength&)> f,
        std::function<ArcLength(const Eigen::Matrix<ArcLength, 3, 1>&)> inv_f) {
    if (!f || !inv_f) throw std::logic_error("Function must be non-null.");
    f_ = f;
    inv_f_ = inv_f;
  }

  /// Example wire parametric function for the bead on a wire example. The
  /// exact function definition is:
  /// <pre>
  ///        | cos(s) |
  /// f(s) = | sin(s) |
  ///        | s      |
  /// </pre>
  static Eigen::Matrix<ArcLength, 3, 1> helix_function(const ArcLength &s);

  /// Inverse parametric function for the bead on a wire system that uses the
  /// helix parametric example function.
  static ArcLength inverse_helix_function(const Vector3<ArcLength> &v);

  /// Gets the output from the parametric function in Vector3d form.
  /// @param m the output from the parametric wire function.
  static Eigen::Vector3d get_pfunction_output(
      const Eigen::Matrix<ArcLength, 3, 1>& m);

  /// Gets the first derivative from the parametric function, in Vector3d form,
  /// using the output from that parametric function.
  /// @param m the output from the parametric wire function.
  static Eigen::Vector3d get_pfunction_first_derivative(
      const Eigen::Matrix<ArcLength, 3, 1>& m);

  /// Gets the second derivative from the parametric function, in Vector3d form,
  /// using the output from that parametric function.
  /// @param m the output from the parametric wire function.
  static Eigen::Vector3d get_pfunction_second_derivative(
      const Eigen::Matrix<ArcLength, 3, 1>& m);

  /// Gets the output from the inverse parametric function as a double, using
  /// the output from that inverse parametric function.
  /// @param m the output from the inverse parametric wire function.
  static double get_inv_pfunction_output(const ArcLength& m);

  /// Gets the first derivative from the inverse parametric function as a
  /// double, using the output from that inverse parametric function.
  /// @param m the output from the inverse parametric wire function.
  static double get_inv_pfunction_first_derivative(const ArcLength& m);

  /// Gets the second derivative from the inverse parametric function as a
  /// double, using the output from that inverse parametric function.
  /// @param m the output from the inverse parametric wire function.
  static double get_inv_pfunction_second_derivative(const ArcLength& m);

  /// Gets the number of constraint equations used for dynamics.
  int get_num_constraint_equations(const systems::Context<T>& context) const;

  /// Evaluates the constraint equations for a bead represented in absolute
  /// coordinates (no constraint equations are used for a bead represented in
  /// minimal coordinates).
  Eigen::VectorXd EvalConstraintEquations(
      const systems::Context<T>& context) const;

  /// Computes the time derivative of the constraint equations, evaluated at
  /// the current generalized coordinates and generalized velocity.
  Eigen::VectorXd EvalConstraintEquationsDot(
      const systems::Context<T>& context) const;

  /// Computes the change in generalized velocity from applying constraint
  /// impulses @p lambda.
  /// @param context The current state of the system.
  /// @param lambda The vector of constraint forces.
  /// @returns a `n` dimensional vector, where `n` is the dimension of the
  ///          quasi-coordinates.
  Eigen::VectorXd CalcVelocityChangeFromConstraintImpulses(
      const systems::Context<T>& context, const Eigen::MatrixXd& J,
      const Eigen::VectorXd& lambda) const;

 protected:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  // The coordinate representation used for kinematics and dynamics.
  CoordinateType coordinate_type_;

  // The wire parameter function. See set_wire_parameter_function() for more
  // information. This pointer is expected to never be null.
  std::function<Eigen::Matrix<ArcLength, 3, 1>(const ArcLength&)> f_{
      &helix_function};

  // The inverse of the wire parameter function.
  // This function takes a point in 3D as input and outputs a scalar
  // in ℝ as output. Constraint stabilization methods for DAEs can use this
  // function (via EvaluateConstraintEquations()) to efficiently project the
  // bead represented in absolute coordinates back onto the wire. If this
  // function is null, EvaluateConstraintEquations() will use generic,
  // presumably less efficient methods instead. This pointer is expected to
  // never be null.
  std::function<ArcLength(const Eigen::Matrix<ArcLength, 3, 1>&)> inv_f_{
      &inverse_helix_function};

  // Signed acceleration due to gravity.
  double g_{-9.81};
};

}  // namespace bead_on_a_wire
}  // namespace examples
}  // namespace drake
