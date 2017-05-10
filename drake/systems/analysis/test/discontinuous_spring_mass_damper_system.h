#pragma once

#include "drake/systems/analysis/test/spring_mass_damper_system.h"

namespace drake {
namespace systems {

// This is a modified spring-mass-damper system for which the acceleration
// component of the derivative function is discontinuous with respect to the
// point mass state. A force of constant magnitude is applied to the
// spring-mass-damper. Tests the ability of an integrator to deal with
// such discontinuities.
template <class T>
class DiscontinuousSpringMassDamperSystem : public SpringMassDamperSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscontinuousSpringMassDamperSystem);
  DiscontinuousSpringMassDamperSystem(double spring_constant_N_per_m,
                                      double damping_constant_N_per_m,
                                      double mass_kg,
                                      double constant_force) :
      SpringMassDamperSystem<T>(spring_constant_N_per_m,
                                damping_constant_N_per_m,
                                mass_kg),
      constant_force_(constant_force) {
    DRAKE_ASSERT(constant_force >= 0.0);
  }

  /// Gets the magnitude of the constant force acting on the system.
  double get_constant_force() const { return constant_force_; }

  /// Re-implements SpringMassDamperSystem::get_closed_form_solution() to
  /// disable it: no closed form solution is currently available.
  /// @throws std::logic_error if called.
  void get_closed_form_solution(const T& x0, const T& v0, const T& tf,
                                T* xf, T* vf) const override {
    throw std::logic_error("No closed form solution available for "
                               "discontinuous mass spring damper.");
  }

 protected:
  System <AutoDiffXd>* DoToAutoDiffXd() const override {
    return new DiscontinuousSpringMassDamperSystem<AutoDiffXd>(
        this->get_spring_constant(),
        this->get_damping_constant(),
        this->get_mass(),
        get_constant_force());
  }

  void DoCalcTimeDerivatives(const Context <T>& context,
                             ContinuousState <T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState <T>& state = *context.get_continuous_state();

    // First element of the derivative is spring velocity.
    const T v = state[1];
    (*derivatives)[0] = v;

    // Compute the force acting on the mass. There is always a constant
    // force pushing the mass toward -inf. The spring and damping forces are
    // only active when x <= 0; the spring setpoint is x = 0.
    T force = -constant_force_;
    const double k = this->get_spring_constant();
    const double b = this->get_damping_constant();
    const T x0 = 0;
    const T x = state[0];
    if (x <= x0)
      force -= k * (x - x0) + b * v;

    // Second element of the derivative is spring acceleration.
    (*derivatives)[1] = force / this->get_mass();
  }

 private:
  double constant_force_;
};

}  // namespace systems
}  // namespace drake
