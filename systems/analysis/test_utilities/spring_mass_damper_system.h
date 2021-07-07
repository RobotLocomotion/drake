#pragma once

#include <limits>
#include <utility>

#include "drake/systems/analysis/test_utilities/spring_mass_system.h"

// WARNING WARNING WARNING
// This test is currently used only as a stiff system test for implicit
// integration.
// TODO(edrumwri): This test should be upgraded to a reusable, closed-form
//                 benchmark by integrating this class with SpringMassSystem.
//                 See issue #6146.

namespace drake {
namespace systems {
namespace implicit_integrator_test {

// This is an unforced spring-mass-damper system.
template <class T>
class SpringMassDamperSystem : public SpringMassSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringMassDamperSystem);

  /// Constructs an unforced spring-mass-damper system.
  /// Subclasses must use the protected constructor, not this one.
  SpringMassDamperSystem(double spring_constant_N_per_m,
                         double damping_constant_Ns_per_m,
                         double mass_kg)
      : SpringMassDamperSystem<T>(
            SystemTypeTag<SpringMassDamperSystem>{},
            spring_constant_N_per_m, damping_constant_Ns_per_m, mass_kg) {}

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit SpringMassDamperSystem(const SpringMassDamperSystem<U>& other)
      : SpringMassDamperSystem(
            other.get_spring_constant(),
            other.get_damping_constant(),
            other.get_mass()) {}

  /// Returns the damping constant that was provided at construction in Ns/m
  double get_damping_constant() const { return damping_constant_Ns_per_m_; }

  /// Returns the closed-form position and velocity solution for the unforced
  /// spring-mass-damper from the given initial conditions *for the case that
  /// the spring-mass-damper is not underdamped*. In other words, this function
  /// requires that `c² - 4⋅m⋅k ≥ 0`, where c is the damping coefficient,
  /// m is the mass, and k is the spring coefficient. Put yet another way,
  /// the damping ratio must be greater than one (i.e., ξ = c/2sqrt(km)) > 1).
  /// @param x0 the position of the spring at time t = 0.
  /// @param v0 the velocity of the spring at time t = 0.
  /// @param tf the time at which to return the position and velocity.
  /// @param[out] xf the position of the spring at time tf, on return.
  /// @param[out] vf the velocity of the spring at time tf, on return.
  /// @throws std::exception if xf or vf is nullptr or the system is
  ///         damped, yet underdamped.
  void GetClosedFormSolution(const T& x0, const T& v0, const T& tf,
                             T* xf, T* vf) const {
    using std::exp;

    if (!xf || !vf)
      throw std::logic_error("Passed final position/velocity is null.");

    // Special case #1: no damping (uses the closed form solution from
    // the mass-spring system).
    if (get_damping_constant() == 0) {
      SpringMassSystem<T>::GetClosedFormSolution(x0, v0, tf, xf, vf);
      return;
    }

    // TODO(mitiguy): Provide solutions to the underdamped system.
    // Special case #2: underdamping.
    if (get_damping_constant() * get_damping_constant() <
        4 * this->get_mass() * this->get_spring_constant()) {
      throw std::logic_error("Closed form solution not available for "
                                 "underdamped system.");
    }

    // m⋅d²x/dt² + c⋅dx/dt + kx = 0
    // Solution to this ODE: x(t) = c₁⋅eʳᵗ + c₂⋅eˢᵗ
    //   where r and s are the roots to the equation mz² + cz + k = 0.
    // Thus, dx/dt = r⋅c₁⋅eʳᵗ + s⋅c₂⋅eˢᵗ.

    // Step 1: Solve the equation for z, yielding r and s.
    T r, s;
    std::tie(r, s) = SolveRestrictedQuadratic(this->get_mass(),
                                              get_damping_constant(),
                                              this->get_spring_constant());

    // Step 2: Substituting t = 0 into the equatinons above, solve the resulting
    // linear system:
    // c1 + c2 = x0
    // r⋅c1 + s⋅c2 = v0
    // yielding:
    // c1 = -(-v0 + s⋅x0)/(r - s) and c2 = -(v0 - r⋅x0)/(r - s)
    const T c1 = -(v0 + s * x0) / (r - s);
    const T c2 = -(v0 - r * x0) / (r - s);

    // Step 3: Set the solutions.
    *xf = c1 * exp(r * tf) + c2 * exp(s * tf);
    *vf = r * c1 * exp(r * tf) + s * c2 * exp(s * tf);
  }

 protected:
  /// Constructor that specifies @ref system_scalar_conversion support.
  SpringMassDamperSystem(SystemScalarConverter converter,
                         double spring_constant_N_per_m,
                         double damping_constant_Ns_per_m,
                         double mass_kg) :
      SpringMassSystem<T>(std::move(converter),
                          spring_constant_N_per_m, mass_kg,
                          false /* unforced */),
      damping_constant_Ns_per_m_(damping_constant_Ns_per_m) {}

  void DoCalcTimeDerivatives(const Context <T>& context,
                             ContinuousState <T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState<T>& state = context.get_continuous_state();

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

    // Third element of the derivative is the energy added into the spring.
    (*derivatives)[2] = this->CalcConservativePower(context);
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
  // of two values, assuming that b² >= 4ac, a != 0, and b != 0. Cancellation
  // error is avoided. Aborts if b² < 4ac, a = 0, or b = 0. This restricted
  // quadratic equation solver will work for the test case in this class; *do
  // not trust this code to solve generic quadratic equations*.
  static std::pair<T, T> SolveRestrictedQuadratic(const T& a, const T& b,
                                                  const T& c) {
    using std::sqrt;
    using std::abs;
    const T disc = b*b - 4 * a * c;
    DRAKE_DEMAND(disc >= 0);
    DRAKE_DEMAND(abs(a) > std::numeric_limits<double>::epsilon());
    DRAKE_DEMAND(abs(b) > std::numeric_limits<double>::epsilon());
    const T x1 = (-b - sgn(b) * sqrt(disc)) / (2 * a);
    const T x2 = c / (a * x1);
    return std::make_pair(x1, x2);
  }

  double damping_constant_Ns_per_m_;
};

}  // namespace implicit_integrator_test
}  // namespace systems
}  // namespace drake

