#pragma once

#include <memory>

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, semi-explicit Euler integrator. State is updated in the
 * following manner:
 * <pre>
 * v(t₀+h) = v(t₀) + dv/dt(t₀) * h
 * dq/dt  = N * v(t₀+h)
 * q(t₀+h) = q(t₀) + dq/dt * h
 * </pre>
 * where `v` are the generalized velocity variables and `q` are generalized
 * coordinates. `h` is the integration step size, and `N` is a matrix 
 * (dependent upon `q(t₀)`) that maps velocities to time derivatives of 
 * generalized coordinates. For rigid body systems in 2D, for example, `N`
 * will generally be an identity matrix. For a single rigid body in 3D, `N` and
 * its pseudo-inverse (`N` is generally non-square but always left invertible)
 * are frequently used to transform between time derivatives of Euler 
 * parameters (unit quaternions) and angular velocities (and vice versa), 
 * [Nikravesh 1988].
 * 
 * Note that these equations imply that the velocity variables are updated
 * first and that these new velocities are then used to update the generalized
 * coordinates (compare to ExplicitEulerIntegrator, where the generalized
 * coordinates are updated using the previous velocity variables).
 *
 * When a mechanical system is Hamiltonian (informally meaning that the
 * system is not subject to velocity-dependent forces), the semi-explicit
 * Euler integrator is a symplectic (energy conserving) integrator.
 * Symplectic integrators advertise energetically consistent behavior with large
 * step sizes compared to non-symplectic integrators. Multi-body systems
 * are not Hamiltonian, even in the absence of externally applied
 * velocity-dependent forces, due to the presence of both Coriolis and
 * gyroscopic forces. This integrator thus does not generally conserve energy
 * for such systems.
 *
 * <h4>Association between time stepping and the semi-explicit Euler
 * integrator:</h4>
 * Though many time stepping approaches use the formulations above, these
 * equations do not represent a "time stepping scheme". The semi-explicit
 * Euler integration equations can be applied from one point in state space to 
 * another, assuming smoothness in between, just like any other integrator using
 * the following process: 
 * (1) a simulator integrates to discontinuities, (2) the state of the ODE/DAE 
 * is re-initialized, and (3) integration continues.
 *
 * In contrast, time stepping schemes enforce all constraints at a single
 * time in the integration process: though a billiard break may consist of tens
 * of collisions occurring sequentially over a millisecond of time, a time
 * stepping method will treat all of these collisions as occurring
 * simultaneously. 
 *
 * - [Nikravesh 1988]  P. Nikravesh. Computer-Aided Analysis of Mechanical 
 *                       Systems. Prentice Hall. New Jersey, 1988.
 * - [Stewart 2000]    D. Stewart. Rigid-body Dynamics with Friction and 
 *                       Impact. SIAM Review, 42:1, 2000.
 */
template <class T>
class SemiExplicitEulerIntegrator : public IntegratorBase<T> {
 public:
  virtual ~SemiExplicitEulerIntegrator() {}

  // TODO(edrumwri): update documentation to account for stretching (after
  //                 stretching has become a user settable).
  /**
   * Constructs a fixed-step integrator for a given system using the given
   * context for initial conditions.
   * @param system A reference to the system to be simulated.
   * @param max_step_size The maximum (fixed) step size; the integrator will
   *                      not take larger step sizes than this.
   * @param context Pointer to the context (nullptr is ok, but the caller
   *                must set a non-null context before Initialize()-ing the
   *                integrator).
   * @sa Initialize()
   */
  SemiExplicitEulerIntegrator(const System<T>& system, const T& max_step_size,
                              Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs_ = system.AllocateTimeDerivatives();
    qdot_ = std::make_unique<BasicVector<T>>(
            context->get_continuous_state()->get_generalized_position().size());
  }

  /**
   * Gets the error estimate order (returns zero, since error estimation is
   * not provided).
   */
  int get_error_estimate_order() const override { return 0; }

  /**
   * Integrator does not support accuracy estimation.
   */
  bool supports_error_estimation() const override { return false; }

 private:
  void DoStepOnceFixedSize(const T& dt) override;

  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs_;
  std::unique_ptr<BasicVector<T>> qdot_;
};

/**
 * Integrates the system forward in time by dt. This value is determined
 * by IntegratorBase::StepOnce().
 */
template <class T>
void SemiExplicitEulerIntegrator<T>::DoStepOnceFixedSize(const T& dt) {
  // Find the continuous state xc within the Context, just once.
  auto context = this->get_mutable_context();
  const auto& xc = context->get_mutable_continuous_state();
  const auto& system = this->get_system();

  // Retrieve the generalized coordinates and velocities and auxiliary
  // variables.
  VectorBase<T>* q = xc->get_mutable_generalized_position();
  VectorBase<T>* v = xc->get_mutable_generalized_velocity();
  VectorBase<T>* z = xc->get_mutable_misc_continuous_state();

  // Calcuate the derivatives.
  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  system.CalcTimeDerivatives(this->get_context(), derivs_.get());

  // Retrieve the accelerations and auxiliary variable derivatives.
  const auto& vdot = derivs_->get_generalized_velocity();
  const auto& zdot = derivs_->get_misc_continuous_state();

  // Update the generalized velocity and auxiliary variables.
  v->PlusEqScaled({ {dt, vdot} });
  z->PlusEqScaled({ {dt, zdot} });

  // Convert the generalized velocity to the time derivative of generalized
  // coordinates and update the generalized coordinates.
  system.MapVelocityToQDot(*context, *v, qdot_.get());
  q->PlusEqScaled({ {dt, *qdot_} });

  // Update the time.
  context->set_time(context->get_time() + dt);
}
}  // namespace systems
}  // namespace drake

