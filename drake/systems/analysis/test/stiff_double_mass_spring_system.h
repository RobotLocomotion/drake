#pragma once

#include <cmath>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

/// A coupled, mass spring system taken from the SD/FAST user's manual.
/// This simple example is used to provide a stiff system for testing
/// implicit integration. Mass cannot be set, nor can spring and damping
/// constants be set.
template <class T>
class StiffDoubleMassSpringSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StiffDoubleMassSpringSystem)
  StiffDoubleMassSpringSystem() {
    this->DeclareContinuousState(2 /* num_q */, 2 /* num_v */, 1 /* num_z */);
  }

  /// Evaluates the spring forces, returning the forces on each body.
  Vector2<T> EvalSpringForce(const Context<T>& context) const {
    const Eigen::Vector2d k = get_spring_constants();
    const Eigen::Vector2d b = get_damping_constants();

    const Vector2<T> x = get_position(context);
    const Vector2<T> v = get_velocity(context);

    const T stretch0 = x(0) - 0;
    const T stretch1 = x(1) - x(0) - 1;

    // Get the force from Spring 1 and Spring 2.
    const T f1 = -k(0)*stretch0 - v(0)*b(0);
    const T f2 = -k(1)*stretch1 - v(1)*b(1);

    // Return the force on each body. Spring 1 acts only Body 1. Spring 2
    // acts on Body 1 and Body 2.
    return Vector2<T>(f1 - f2, f2);
  }

  T DoCalcPotentialEnergy(const Context<T>& context) const {
    const Eigen::Vector2d k = get_spring_constants();
    const Vector2<T> x = get_position(context);
    const T stretch0 = x(0) - 0;
    const T stretch1 = x(1) - x(0) - 1;
    const T pe = k(0) * stretch0 * stretch0 / 2 +
                 k(1) * stretch1 * stretch1 / 2;
    return pe;
  }

  T DoCalcKineticEnergy(const Context<T>& context) const override {
    const Vector2<T> v = get_velocity(context);
    return (Eigen::DiagonalMatrix<T, 2>(get_mass()) * v).dot(v) / 2;
  }

  T DoCalcConservativePower(const Context<T>& context) const override {
    const T& power_c = EvalSpringForce(context).dot(get_velocity(context));
    return power_c;
  }

  T DoCalcNonConservativePower(const Context<T>&) const override {
    const T& power_nc = 0.;
    return power_nc;
  }

  /// Gets the two spring constants.
  Eigen::Vector2d get_spring_constants() const { return
        Eigen::Vector2d(750, 1e20); }

  /// Gets the two damping constants.
  Eigen::Vector2d get_damping_constants() const { return
      Eigen::Vector2d(0, 0); }

  /// Gets the positions of the two point mass bodies.
  Vector2<T> get_position(const Context<T>& c) const { return
        c.get_continuous_state()->get_generalized_position().CopyToVector(); }

  /// Gets the velocity of the two point mass bodies.
  Vector2<T> get_velocity(const Context<T>& c) const { return
        c.get_continuous_state()->get_generalized_velocity().CopyToVector(); }

  /// Gets the mass for the bodies in the system.
  Eigen::Vector2d get_mass() const { return Eigen::Vector2d(1.0, 1.0); }

  bool has_any_direct_feedthrough() const override { return false; }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* deriv) const override {
    // Get velocity.
    const VectorBase<T>& xd = context.get_continuous_state()->
        get_generalized_velocity();

    // Get the masses and spring and damping coefficients.
    const Vector2<T> mass = get_mass();

    // Compute the forces.
    const Vector2<T> f = EvalSpringForce(context);

    // Compute the acceleration.
    const Vector2<T> a = f.array() / mass.array();

    // Set the derivatives.
    deriv->get_mutable_generalized_position()->SetFrom(xd);
    deriv->get_mutable_generalized_velocity()->SetFromVector(a);

    // We are integrating conservative power to get the work done by conservative
    // force elements, that is, the net energy transferred between the spring and
    // the mass over time.
    deriv->get_mutable_misc_continuous_state()->SetAtIndex(0,
        this->CalcConservativePower(context));
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
  }

  /// Gets the end time for integration.
  T get_end_time() const { return 1e1; }

  /// Sets the initial conditions for the system.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    Vector2<T> x, xd;
    x(0) = 0.5;
    x(1) = 1.5;
    xd.setZero();

    state->get_mutable_continuous_state()->get_mutable_generalized_position()->
        SetFromVector(x);
    state->get_mutable_continuous_state()->get_mutable_generalized_velocity()->
        SetFromVector(xd);
    state->get_mutable_continuous_state()->get_mutable_misc_continuous_state()->
        SetAtIndex(0, 0);
  }

    /// Gets the solution for the system with initial state defined at @p context.
  void get_solution(const Context<T>& context, T t,
                    ContinuousState<T>* state) const {
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
    const double c1 = context.get_continuous_state()->
        get_generalized_position().GetAtIndex(0);
    const double c2 = context.get_continuous_state()->
        get_generalized_velocity().GetAtIndex(0) / omega;

    // Set the position and velocity of the first body using the ODE solution.
    const double x1_final = c1 * cos(omega * t) + c2 * sin(omega * t);
    const double v1_final = c1 * -sin(omega * t) * omega +
                            c2 * +cos(omega * t) * omega;
    state->get_mutable_generalized_position()->SetAtIndex(0, x1_final);
    state->get_mutable_generalized_velocity()->SetAtIndex(0, v1_final);

    // The position of the second body should be offset exactly from the first.
    state->get_mutable_generalized_position()->SetAtIndex(1, x1_final + offset);

    // Velocity of the second body should be zero.
    state->get_mutable_generalized_velocity()->SetAtIndex(1, 0);
  }
};

}  // namespace
}  // namespace systems
}  // namespace drake
