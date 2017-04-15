#pragma once

#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

// TODO(edrumwri): Beef this class up and move it to drake/systems/plants.

namespace drake {
namespace systems {
namespace implicit_integrator_test {

// This is an unforced spring-mass-damper system.
template <class T>
class SpringMassDamperSystem : public SpringMassSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringMassDamperSystem);
  SpringMassDamperSystem(double spring_constant_N_per_m,
                         double damping_constant_N_per_m,
                         double mass_kg) :
      SpringMassSystem<T>(spring_constant_N_per_m, mass_kg,
                          false /* unforced */),
      damping_constant_N_per_m_(damping_constant_N_per_m) { }

  /// Returns the damping constant that was provided at construction in N/m
  double get_damping_constant() const { return damping_constant_N_per_m_; }

 protected:
  System<AutoDiffXd>* DoToAutoDiffXd() const override {
    return new SpringMassDamperSystem<AutoDiffXd>(this->get_spring_constant(),
                                                  get_damping_constant(),
                                                  this->get_mass());
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState<T>& state = *context.get_continuous_state();

    // First element of the derivative is spring velocity.
    const T xd = state[1];
    (*derivatives)[0] = xd;

    // Compute the force acting on the mass.
    const double k = this->get_spring_constant();
    const double b = get_damping_constant();
    const T x0 = 0;
    const T x = state[0];
    T force = -k * (x - x0) - b * xd;

    // Second element of the derivative is spring acceleration.
    (*derivatives)[1] = force / this->get_mass();
  }

  /// Returns the closed-form position and velocity solution for the unforced
  /// spring-mass-damper from the given initial conditions.
  /// @param x0 the position of the spring at time t = 0.
  /// @param v0 the velocity of the spring at time t = 0.
  /// @param tf the time at which to return the position and velocity.
  /// @param[out] xf the position of the spring at time tf, on return.
  /// @param[out] vf the velocity of the spring at time tf, on return.
  /// @throws std::logic_error if xf or vf is nullptr.
  void get_closed_form_solution(const T& x0, const T& v0, const T& tf,
                                T* xf, T* vf) const override {
    using std::sqrt;
    using std::sin;
    using std::cos;
    using std::exp;

    if (!xf || !vf)
      throw std::logic_error("Passed final position/velocity is null.");

    // m⋅d²x/dt² + c⋅dx/dt + kx = 0
    // Solution to this ODE: x(t) = c₁⋅eʳᵗ + c₂⋅eˢᵗ
    //   where r and s are the roots to the equation mz² + cz + k = 0.
    // Thus, dx/dt = r⋅c₁⋅eʳᵗ + s⋅c₂⋅eˢᵗ.

    // Step 1: Solve the equation for z, yielding r and s.
    T r, s;
    std::tie(r, s) = SolveQuadratic(this->get_mass(), get_damping_constant(),
                                    this->get_spring_constant());

    // Step 2: Substituting t = 0 into the equatinons above, solve the resulting
    // linear system:
    // c1 + c2 = x0
    // r⋅c1 + s⋅c2 = v0
    // yielding:
    // c1 = -(-v0 + s⋅x0)/(r - s) and c2 = -(v0 - r⋅x0)/(r - s)
    const T c1 = -(v0 + s*x0)/(r - s);
    const T c2 = -(v0 - r*x0)/(r - s);

    // Step 3: Set the solutions.
    *xf = c1*exp(r*tf) + c2*exp(s*tf);
    *vf = r*c1*exp(r*tf) + s*c2*exp(s*tf);
  }

 private:
  // Signum function.
  static T sgn(const T& x) {
    if (x > 0) {
      return 1;
    } else {
      if (x < 0) {
        return -1;
      } else {
        return 0;
      }
    }
  }

  // Solves the quadratic equation ax² + bx + c = 0 for x, returned as a pair
  // of two values. Cancellation error is avoided. Aborts if b is zero.
  static std::pair<T, T> SolveQuadratic(const T& a, const T& b, const T& c) {
    DRAKE_DEMAND(b != 0);
    const T x1 = (-b - sgn(b)*sqrt(b*b - 4*a*c))/(2*a);
    const T x2 = c/(a*x1);
    return std::make_pair(x1, x2);
  }

  double damping_constant_N_per_m_;
};

}  // namespace implicit_integrator_test
}  // namespace systems
}  // namespace drake

