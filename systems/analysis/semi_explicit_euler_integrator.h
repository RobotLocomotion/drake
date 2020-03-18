#pragma once

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, semi-explicit Euler integrator. State is updated in the
 * following manner: <pre>
 *     v(t₀+h) = v(t₀) + dv/dt(t₀) * h
 *       dq/dt = N(q(t₀)) * v(t₀+h)
 *     q(t₀+h) = q(t₀) + dq/dt * h
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
 *
 * @tparam_nonsymbolic_scalar
 * @ingroup integrators
 */
template <class T>
class SemiExplicitEulerIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SemiExplicitEulerIntegrator)

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
      : IntegratorBase<T>(system, context),
        qdot_(context->get_continuous_state().num_q()) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
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
  bool DoStep(const T& h) override;

  // This is a pre-allocated temporary for use by integration
  BasicVector<T> qdot_;
};

/**
 * Integrates the system forward in time by h. This value is determined
 * by IntegratorBase::StepOnce().
 */
template <class T>
bool SemiExplicitEulerIntegrator<T>::DoStep(const T& h) {
  const System<T>& system = this->get_system();
  Context<T>& context = *this->get_mutable_context();

  // CAUTION: This is performance-sensitive inner loop code that uses dangerous
  // long-lived references into state and cache to avoid unnecessary copying and
  // cache invalidation. Be careful not to insert calls to methods that could
  // invalidate any of these references before they are used.

  // Evaluate derivative xcdot(t₀) ← xcdot(t₀, x(t₀), u(t₀)).
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(context);
  // Retrieve the accelerations and auxiliary variable derivatives.
  const VectorBase<T>& vdot = xc_deriv.get_generalized_velocity();
  const VectorBase<T>& zdot = xc_deriv.get_misc_continuous_state();

  // Cache: vdot and zdot reference the live derivative cache value, currently
  // up to date but about to be marked out of date. We do not want to make
  // an unnecessary copy of this data.

  // This invalidates computations that are dependent on v or z.
  // Marks v- and z-dependent cache entries out of date, including vdot and
  // zdot; time doesn't change here.
  std::pair<VectorBase<T>*, VectorBase<T>*> vz = context.GetMutableVZVectors();
  VectorBase<T>& v = *vz.first;
  VectorBase<T>& z = *vz.second;

  // Cache: vdot and zdot still reference the derivative cache value, which is
  // unchanged, although it is marked out of date.

  // Update the velocity and auxiliary state variables.
  v.PlusEqScaled(h, vdot);
  z.PlusEqScaled(h, zdot);

  // Convert the updated generalized velocity to the time derivative of
  // generalized coordinates. Note that this mapping is q-dependent and
  // hasn't been invalidated if it was pre-computed.
  system.MapVelocityToQDot(context, v, &qdot_);

  // Now set time and q to their final values. This marks time- and
  // q-dependent cache entries out of date. That includes the derivative
  // cache entry though we don't need it again here.
  VectorBase<T>& q =
      context.SetTimeAndGetMutableQVector(context.get_time() + h);
  q.PlusEqScaled(h, qdot_);

  // This integrator always succeeds at taking the step.
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::SemiExplicitEulerIntegrator)
