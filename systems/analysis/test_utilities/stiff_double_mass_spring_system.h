#pragma once

#include <cmath>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace test {

/// A coupled, mass spring system taken from the SD/FAST user's manual.
/// This simple example is used to provide a stiff system for testing
/// implicit integration. Mass cannot be set, nor can spring and damping
/// constants be set.
///
/// The system of ODEs for the double spring mass system follows:<pre>
/// ẍ₁ = (f₁ - f₂)/m₁
/// ẍ₂ = f₂/m₂
/// </pre>
/// where <pre>
/// f₁ = -k₁x₁ - b₁ẋ₁
/// f₂ = -k₂(x₂ - x₁ - 1) - b₂(ẋ₂ - ẋ₁)
/// </pre>
/// and f₁ and f₂ are the spring and damper forces acting between the
/// world/the first mass and the first and second masses, respectively, k are
/// spring constants, b are damping constants, and m are masses. Note
/// that the resting position of the system is at x₁ = 0, x₂ = 1, ẋ₁ = ẋ₂ = 0.
///
/// This system uses m₁ = m₂ and an extremely stiff spring between the two
/// masses (1e20 kg/s²) in order to approximate the masses being rigidly
/// connected. The other spring, which connects the first mass to "the world",
/// uses a much smaller stiffness of 750 kg/s². Damping is currently set to
/// b₁ = b₂ = 0 (i.e., disabled).
template <class T>
class StiffDoubleMassSpringSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StiffDoubleMassSpringSystem)
  StiffDoubleMassSpringSystem() {
    this->DeclareContinuousState(2 /* num_q */, 2 /* num_v */, 0 /* num_z */);
  }

  /// Evaluates the spring and damping forces, returning the forces on each
  /// body.
  Vector2<T> EvalSpringDamperForces(const Context<T>& context) const {
    const Eigen::Vector2d k = get_spring_constants();
    const Eigen::Vector2d b = get_damping_constants();

    const Vector2<T> x = get_position(context);
    const Vector2<T> v = get_velocity(context);

    const T stretch0 = x(0) - 0;
    const T stretch1 = x(1) - x(0) - 1;

    // Get the force from Spring 1 and Spring 2.
    const T f1 = -k(0) * stretch0 - v(0) * b(0);
    const T f2 = -k(1) * stretch1 - (v(1) - v(0)) * b(1);

    // Return the force on each body. Spring 1 acts only Body 1. Spring 2
    // acts on Body 1 and Body 2.
    return Vector2<T>(f1 - f2, f2);
  }

  /// Gets the two spring constants.
  Eigen::Vector2d get_spring_constants() const {
    return
        Eigen::Vector2d(750, 1e20);
  }

  /// Gets the two damping constants.
  Eigen::Vector2d get_damping_constants() const {
    return
        Eigen::Vector2d(0, 0);
  }

  /// Gets the positions of the two point mass bodies.
  Vector2<T> get_position(const Context<T>& c) const {
    return
        c.get_continuous_state().get_generalized_position().CopyToVector();
  }

  /// Gets the velocity of the two point mass bodies.
  Vector2<T> get_velocity(const Context<T>& c) const {
    return
        c.get_continuous_state().get_generalized_velocity().CopyToVector();
  }

  /// Gets the mass for the bodies in the system.
  Eigen::Vector2d get_mass() const { return Eigen::Vector2d(1.0, 1.0); }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* deriv) const override {
    // Get velocity.
    const VectorBase<T>& xd = context.get_continuous_state().
        get_generalized_velocity();

    // Get the masses and spring and damping coefficients.
    const Vector2<T> mass = get_mass();

    // Compute the forces.
    const Vector2<T> f = EvalSpringDamperForces(context);

    // Compute the acceleration.
    const Vector2<T> a = f.array() / mass.array();

    // Set the derivatives.
    deriv->get_mutable_generalized_position().SetFrom(xd);
    deriv->get_mutable_generalized_velocity().SetFromVector(a);
  }

  /// Gets the end time for integration.
  T get_end_time() const { return 1e1; }

  /// Sets the initial conditions for the system.
  /// The first mass will be located at x1 = 0.5 and second will be located at
  /// x2 = 1.5. No initial velocity is present.
  void SetDefaultState(const Context<T>&,
                       State<T>* state) const override {
    Vector2<T> x, xd;
    x(0) = 0.5;
    x(1) = 1.5;
    xd.setZero();

    state->get_mutable_continuous_state().get_mutable_generalized_position().
        SetFromVector(x);
    state->get_mutable_continuous_state().get_mutable_generalized_velocity().
        SetFromVector(xd);
  }

  /// Gets the solution for the system with initial state defined at @p context,
  /// returning the solution at time @p t, in @p state. Aside from the
  /// assumption that there is zero initial stretching between the two point
  /// masses, initial conditions are arbitrary. The solution is predicated
  /// on zero damping (aborts if this is not true).
  void GetSolution(const Context<T>& context, const T& t,
                   ContinuousState<T>* state) const {
    const Eigen::Vector2d b = get_damping_constants();
    DRAKE_DEMAND(b[0] == b[1] && b[0] == 0.0);

    using std::cos;
    using std::sin;

    // Get the offset between the two bodies
    const T offset = state->get_generalized_position().GetAtIndex(1) -
        state->get_generalized_position().GetAtIndex(0);

    // Omega will use the first body (the one connected to the "world" with the
    // non-stiff spring).
    const double omega = std::sqrt(get_spring_constants()(0) /
        (get_mass()(0) + get_mass()(1)));

    // Setup c1 and c2 for ODE constants.
    const double c1 = context.get_continuous_state().
        get_generalized_position().GetAtIndex(0);
    const double c2 = context.get_continuous_state().
        get_generalized_velocity().GetAtIndex(0) / omega;

    // Set the position and velocity of the first body using the ODE solution.
    const double x1_final = c1 * cos(omega * t) + c2 * sin(omega * t);
    const double v1_final = c1 * -sin(omega * t) * omega +
        c2 * +cos(omega * t) * omega;
    state->get_mutable_generalized_position().SetAtIndex(0, x1_final);
    state->get_mutable_generalized_velocity().SetAtIndex(0, v1_final);

    // The position of the second body should be offset exactly from the first.
    state->get_mutable_generalized_position().SetAtIndex(1, x1_final + offset);

    // Velocity of the second body should be equal to that of the first body.
    state->get_mutable_generalized_velocity().SetAtIndex(1, v1_final);
  }
};

}  // namespace test
}  // namespace analysis
}  // namespace systems
}  // namespace drake
