#pragma once

#include <memory>
#include <stdexcept>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

/**
 * A first-order, fully implicit integrator optimized for second-order systems,
 * with a second-order error estimate.
 *
 * The velocity-implicit Euler integrator is a variant of the first-order
 * implicit Euler that takes advantage of the simple mapping q̇ = N(q) v
 * of second order systems to formulate a smaller problem in velocities (and
 * miscellaneous states if any) only. For systems with second-order dynamics,
 * %VelocityImplicitEulerIntegrator formulates a problem that is half as large
 * as that formulated by Drake's ImplicitEulerIntegrator, resulting in improved
 * run-time performance. Upon convergence of the resulting system of equations,
 * this method provides the same discretization as ImplicitEulerIntegrator, but
 * at a fraction of the computational cost.
 *
 * This integrator requires a system of ordinary differential equations (ODEs)
 * in state `x = (q,v,z)` to be expressible in the following form:
 *
 *     q̇ = N(q) v;                                   (1)
 *     ẏ = f_y(t,q,y),                               (2)
 * where `q̇` and `v` are linearly related via the kinematic mapping `N(q)`,
 * `y = (v,z)`, and `f_y` is a function that can depend on the time and state.
 *
 * Implicit Euler uses the following update rule at time step n:
 *
 *     qⁿ⁺¹ = qⁿ + h N(qⁿ⁺¹) vⁿ⁺¹;                   (3)
 *     yⁿ⁺¹ = yⁿ + h f_y(tⁿ⁺¹,qⁿ⁺¹,yⁿ⁺¹).            (4)
 *
 * To solve the nonlinear system for `(qⁿ⁺¹,yⁿ⁺¹)`, the velocity-implicit Euler
 * integrator iterates with a modified Newton's method: At iteration `k`, it
 * finds a `(qₖ₊₁,yₖ₊₁)` that attempts to satisfy
 *
 *     qₖ₊₁ = qⁿ + h N(qₖ) vₖ₊₁.                     (5)
 *     yₖ₊₁ = yⁿ + h f_y(tⁿ⁺¹,qₖ₊₁,yₖ₊₁);            (6)
 *
 * In this notation, the `n`'s index timesteps, while the `k`'s index the
 * specific Newton-Raphson iterations within each time step.
 *
 * Notice that we've intentionally lagged N(qₖ) one iteration behind in Eq (5).
 * This allows it to substitute (5) into (6) to obtain a non-linear system in
 * `y` only. Contrast this strategy with the one implemented by
 * ImplicitEulerIntegrator, which solves a larger non-linear system in the full
 * state x.
 *
 * To find a `(qₖ₊₁,yₖ₊₁)` that approximately satisfies (5-6), we linearize
 * the system (5-6) to compute a Newton step. Define
 *
 *     l(y) = f_y(tⁿ⁺¹,qⁿ + h N(qₖ) v,y),            (7)
 *     Jₗ(y) = ∂l(y) / ∂y.                           (8)
 *
 * To advance the Newton step, the velocity-implicit Euler integrator solves
 * the following linear equation for `Δy`:
 *
 *     (I - h Jₗ) Δy = - R(yₖ),                      (9)
 * where `R(y) = y - yⁿ - h l(y)` and `Δy = yₖ₊₁ - yₖ`. The `Δy` solution
 * directly gives us `yₖ₊₁`. It then substitutes the `vₖ₊₁` component of `yₖ₊₁`
 * in (5) to get `qₖ₊₁`.
 *
 * This implementation uses a Newton method and relies upon the convergence
 * to a solution for `y` in `R(y) = 0` where `R(y) = y - yⁿ - h l(y)`
 * as `h` becomes sufficiently small. General implementational details for
 * the Newton method were gleaned from Section IV.8 in [Hairer, 1996].
 *
 * ### Error Estimation
 *
 * In this integrator, we simultaneously take a large step at the requested
 * step size of h as well as two half-sized steps each with step size `h/2`.
 * The result from two half-sized steps is propagated as the solution, while
 * the difference between the two results is used as the error estimate for the
 * propagated solution. This error estimate is accurate to the second order.
 *
 * To be precise, let `x̅ⁿ⁺¹` be the computed solution from a large step,
 * `x̃ⁿ⁺¹` be the computed solution from two small steps, and `xⁿ⁺¹` be the true
 * solution. Since the integrator propagates `x̃ⁿ⁺¹` as its solution, we denote
 * the true error vector as `ε = x̃ⁿ⁺¹ - xⁿ⁺¹`. VelocityImplicitEulerIntegrator
 * uses `ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹`, the difference between the two solutions, as the
 * second-order error estimate, because for a smooth system, `‖ε*‖ = O(h²)`,
 * and `‖ε - ε*‖ = O(h³)`. See the notes in
 * VelocityImplicitEulerIntegrator<T>::get_error_estimate_order() for a
 * detailed derivation of the error estimate's truncation error.
 *
 * In this implementation, VelocityImplicitEulerIntegrator<T> attempts the
 * large full-sized step before attempting the two small half-sized steps,
 * because the large step is more likely to fail to converge, and if it is
 * performed first, convergence failures are detected early, avoiding the
 * unnecessary effort of computing potentially-successful small steps.
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996, Section IV.8, p. 118–130.
 *
 * @note In the statistics reported by IntegratorBase, all statistics that deal
 * with the number of steps or the step sizes will track the large full-sized
 * steps. This is because the large full-sized `h` is the smallest irrevocable
 * time-increment advanced by this integrator: if, for example, the second small
 * half-sized step fails, this integrator revokes to the state before the first
 * small step. This behavior is similar to other integrators with multi-stage
 * evaluation: the step-counting statistics treat a "step" as the combination of
 * all the stages.
 * @note Furthermore, because the small half-sized steps are propagated as the
 * solution, the large full-sized step is the error estimator, and the error
 * estimation statistics track the effort during the large full-sized step. If
 * the integrator is not in full-Newton mode (see
 * ImplicitIntegrator<T>::set_use_full_newton()), most of the work incurred by
 * constructing and factorizing matrices and by failing Newton-Raphson
 * iterations will be counted toward the error estimation statistics, because
 * the large step is performed first.
 * @note This integrator uses the integrator accuracy setting, even when run
 * in fixed-step mode, to limit the error in the underlying Newton-Raphson
 * process. See IntegratorBase::set_target_accuracy() for more info.
 *
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
 * @see ImplicitEulerIntegrator class documentation for information about
 *      the "implicit Euler" integration method.
 *
 * @tparam_nonsymbolic_scalar
 * @ingroup integrators
 */
template <class T>
class VelocityImplicitEulerIntegrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityImplicitEulerIntegrator)

  ~VelocityImplicitEulerIntegrator() override = default;

  explicit VelocityImplicitEulerIntegrator(const System<T>& system,
                                           Context<T>* context = nullptr)
      : ImplicitIntegrator<T>(system, context) {}

  /**
   * Returns true, because this integrator supports error estimation.
   */
  bool supports_error_estimation() const final { return true; }

  /**
   * Returns the asymptotic order of the difference between the large and small
   * steps (from which the error estimate is computed), which is 2. That is, the
   * error estimate, `ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹` has the property that `‖ε*‖ = O(h²)`,
   * and it deviates from the true error, `ε`, by `‖ε - ε*‖ = O(h³)`. 
   *
   * ### Derivation of the asymptotic order
   *
   * To derive the second-order error estimate, let us first define the vector-
   * valued function `e(tⁿ, h, xⁿ) = x̅ⁿ⁺¹ - xⁿ⁺¹`, the local truncation error
   * for a single, full-sized velocity-implicit Euler integration step, with
   * initial conditions `(tⁿ, xⁿ)`, and a step size of `h`. Furthermore, use
   * `ẍ` to denote `df/dt`, and `∇f` and `∇ẍ` to denote the Jacobians `df/dx`
   * and `dẍ/dx` of the ODE system `ẋ = f(t, x)`. Note that `ẍ` uses a total
   * time derivative, i.e., `ẍ = ∂f/∂t + ∇f f`.
   *
   * Let us use `x*` to denote the true solution after a half-step, `x(tⁿ+½h)`,
   * and `x̃*` to denote the velocity-implicit Euler solution after a single
   * half-sized step. Furthermore, let us use `xⁿ*¹` to denote the true solution
   * of the system at time `t = tⁿ+h` if the system were at `x̃*` when
   * `t = tⁿ+½h`. See the following diagram for an illustration.
   *
   *      Legend:
   *      ───── propagation along the true system
   *      :···· propagation using implicit Euler with a half step
   *      :---- propagation using implicit Euler with a full step
   *
   *      Time  tⁿ         tⁿ+½h         tⁿ+h
   *
   *      State :----------------------- x̅ⁿ⁺¹  <─── used for error estimation
   *            :
   *            :
   *            :
   *            :            :·········· x̃ⁿ⁺¹  <─── propagated result
   *            :            :
   *            :·········  x̃*   ─────── xⁿ*¹
   *            :
   *            xⁿ ───────  x*   ─────── xⁿ⁺¹  <─── true solution
   *
   * We will use superscripts to denote evaluating an expression with `x` at
   * that subscript and `t` at the corresponding time, e.g. `ẍⁿ` denotes
   * `ẍ(tⁿ, xⁿ)`, and `f*` denotes `f(tⁿ+½h, x*)`. We first present a shortened
   * derivation, followed by the longer, detailed version.
   *
   * We know the local truncation error for the implicit Euler method is:
   *
   *     e(tⁿ, h, xⁿ) = x̅ⁿ⁺¹ - xⁿ⁺¹ = ½ h²ẍⁿ + O(h³).    (10)
   *
   * The local truncation error ε from taking two half steps is composed of
   * these two terms:
   *
   *     e₁ = xⁿ*¹ - xⁿ⁺¹ = (1/8) h²ẍⁿ + O(h³),          (15)
   *     e₂ = x̃ⁿ⁺¹ - xⁿ*¹ = (1/8) h²ẍⁿ + O(h³).          (20)
   *
   * Taking the sum,
   *
   *     ε = x̃ⁿ⁺¹ - xⁿ⁺¹ = e₁ + e₂ = (1/4) h²ẍⁿ + O(h³). (21)
   *
   * These two estimations allow us to obtain an estimation of the local error
   * from the difference between the available quantities x̅ⁿ⁺¹ and x̃ⁿ⁺¹:
   *
   *     ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹ = e(tⁿ, h, xⁿ) - ε,
   *                      = (1/4) h²ẍⁿ + O(h³),          (22)
   *
   * and therefore our error estimate is second order.
   *
   * Below we will show this derivation in detail along with the proof that
   * `‖ε - ε*‖ = O(h³)`:
   *
   * Let us look at a single velocity-implicit Euler step. Upon Newton-Raphson
   * convergence, the truncation error for velocity-implicit Euler, which is the
   * same as the truncation error for implicit Euler (because both methods solve
   * Eqs. (3-4)), is
   *
   *     e(tⁿ, h, xⁿ) = ½ h²ẍⁿ⁺¹ + O(h³)
   *                  = ½ h²ẍⁿ + O(h³).                  (10)
   *
   * To see why the two are equivalent, we can Taylor expand about `(tⁿ, xⁿ)`,
   *
   *     ẍⁿ⁺¹ = ẍⁿ + h dẍ/dtⁿ + O(h²) = ẍⁿ + O(h).
   *     e(tⁿ, h, xⁿ) = ½ h²ẍⁿ⁺¹ + O(h³) = ½ h²(ẍⁿ + O(h)) + O(h³)
   *                  = ½ h²ẍⁿ + O(h³).
   *
   * Moving on with our derivation, after one small half-sized implicit Euler
   * step, the solution `x̃*` is
   *
   *     x̃* = x* + e(tⁿ, ½h, xⁿ)
   *        = x* + (1/8) h²ẍⁿ + O(h³),
   *     x̃* - x* = (1/8) h²ẍⁿ + O(h³).                   (11)
   *
   * Taylor expanding about `t = tⁿ+½h` in this `x = x̃*` alternate reality,
   *
   *     xⁿ*¹ = x̃* + ½h f(tⁿ+½h, x̃*) + O(h²).            (12)
   *
   * Similarly, Taylor expansions about `t = tⁿ+½h` and the true solution
   * `x = x*` also give us
   *
   *     xⁿ⁺¹ = x* + ½h f* + O(h²),                      (13)
   *     f(tⁿ+½h, x̃*) = f* + (∇f*) (x̃* - x*) + O(‖x̃* - x*‖²)
   *                  = f* + O(h²),                      (14)
   * where in the last line we substituted Eq. (11).
   *
   * Eq. (12) minus Eq. (13) gives us,
   *
   *     xⁿ*¹ - xⁿ⁺¹ = x̃* - x* + ½h(f(tⁿ+½h, x̃*) - f*) + O(h³),
   *                 = x̃* - x* + O(h³),
   * where we just substituted in Eq. (14). Finally, substituting in Eq. (11),
   *
   *     e₁ = xⁿ*¹ - xⁿ⁺¹ = (1/8) h²ẍⁿ + O(h³).          (15)
   *
   * After the second small step, the solution `x̃ⁿ⁺¹` is
   *
   *     x̃ⁿ⁺¹ = xⁿ*¹ + e(tⁿ+½h, ½h, x̃*),
   *          = xⁿ*¹ + (1/8)h² ẍ(tⁿ+½h, x̃*) + O(h³).     (16)
   *
   * Taking Taylor expansions about `(tⁿ, xⁿ)`,
   *
   *     x* = xⁿ + ½h fⁿ + O(h²) = xⁿ + O(h).            (17)
   *     x̃* - xⁿ = (x̃* - x*) + (x* - xⁿ) = O(h),         (18)
   * where we substituted in Eqs. (11) and (17), and
   *
   *     ẍ(tⁿ+½h, x̃*) = ẍⁿ + ½h ∂ẍ/∂tⁿ + ∇ẍⁿ (x̃* - xⁿ) + O(h ‖x̃* - xⁿ‖)
   *                  = ẍⁿ + O(h),                       (19)
   * where we substituted in Eq. (18).
   *
   * Substituting Eqs. (19) and (15) into Eq. (16),
   *
   *     x̃ⁿ⁺¹ = xⁿ*¹ + (1/8) h²ẍⁿ + O(h³)                (20)
   *          = xⁿ⁺¹ + (1/4) h²ẍⁿ + O(h³),
   * therefore
   *
   *     ε = x̃ⁿ⁺¹ - xⁿ⁺¹ = (1/4) h² ẍⁿ + O(h³).          (21)
   *
   * Subtracting Eq. (21) from Eq. (10),
   *
   *     e(tⁿ, h, xⁿ) - ε = (½ - 1/4) h²ẍⁿ + O(h³);
   *     ⇒ ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹ = (1/4) h²ẍⁿ + O(h³).        (22)
   *
   * Eq. (22) shows that our error estimate is second-order. Since the first
   * term on the RHS matches `ε` (Eq. (21)),
   *
   *     ε* = ε + O(h³).                                 (23)
   */
  int get_error_estimate_order() const final { return 2; }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  // These methods return the effort done by the large step, which is the error
  // estimator for the half-sized steps.
  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return this->get_num_derivative_evaluations() -
           num_half_vie_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return this->get_num_derivative_evaluations_for_jacobian() -
           num_half_vie_jacobian_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations() const final {
    return this->get_num_newton_raphson_iterations() -
           num_half_vie_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return this->get_num_jacobian_evaluations() -
           num_half_vie_jacobian_reforms_;
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return this->get_num_iteration_matrix_factorizations() -
           num_half_vie_iter_factorizations_;
  }

  void DoResetImplicitIntegratorStatistics() final;

  static void ComputeAndFactorImplicitEulerIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  void DoInitialize() final;

  bool DoImplicitIntegratorStep(const T& h) final;

  // Steps the system forward by a single step of h using the velocity-implicit
  // Euler method.
  // @param t0 the time at the left end of the integration interval.
  // @param h the time increment to step forward.
  // @param xn the continuous state at t0, which is xⁿ.
  // @param xtplus_guess the starting guess for xⁿ⁺¹.
  // @param [out] xtplus the computed value for xⁿ⁺¹ on successful return.
  // @param [in, out] iteration_matrix the cached iteration matrix, which is
  //        updated if get_use_full_newton() is true, if get_reuse() is false,
  //        or if the Newton-Raphson fails to converge on the first try.
  // @param [in, out] Jy the cached Jacobian Jₗ(y), which is updated if
  //        get_use_full_newton() is true, if get_reuse() is false, or if the
  //        Newton-Raphson fails to converge on the second try.
  // @param trial the attempt for this approach (1-4).
  //        StepVelocityImplicitEuler() uses increasingly computationally
  //        expensive methods as the trial numbers increase.
  // @returns `true` if the step of size `h` was successful, `false` otherwise.
  // @note The time and continuous state in the context are indeterminate upon
  //       exit.
  bool StepVelocityImplicitEuler(
      const T& t0, const T& h, const VectorX<T>& xn,
      const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jy, int trial = 1);

  // Steps the system forward by two half-sized steps of size h/2 using the
  // velocity-implicit Euler method, and keeps track of separate statistics
  // for the derivative evaluations, matrix refactorizations, and Jacobian
  // recomputations during these half-sized steps. This method calls
  // StepVelocityImplicitEuler() up to twice to perform the
  // two half-sized steps.
  // @param t0 the time at the left end of the integration interval.
  // @param h the combined time increment to step forward.
  // @param xn the continuous state at t0, which is xⁿ.
  // @param xtplus_guess the starting guess for xⁿ⁺¹.
  // @param [out] xtplus the computed value for xⁿ⁺¹ on successful return.
  // @param [in, out] iteration_matrix the cached iteration matrix, which is
  //        updated if either StepVelocityImplicitEuler() calls update it.
  // @param [in, out] Jy the cached Jacobian Jₗ(y), which is updated if
  //        either StepVelocityImplicitEuler() calls update it.
  // @returns `true` if both steps were successful, `false` otherwise.
  // @note The time and continuous state in the context are indeterminate upon
  //       exit.
  bool StepHalfVelocityImplicitEulers(
      const T& t0, const T& h, const VectorX<T>& xn,
      const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jy);

  // Takes a large velocity-implicit Euler step (of size h) and two half-sized
  // velocity-implicit Euler steps (of size h/2), if possible.
  // @param t0 the time at the left end of the integration interval.
  // @param h the integration step size to attempt.
  // @param [out] xtplus_vie contains the velocity-implicit Euler solution
  //              (i.e., `xⁿ⁺¹`) after the large step, if successful, on return.
  // @param [out] xtplus_hvie contains the velocity-implicit Euler solution
  //              (i.e., `xⁿ⁺¹`) after the two small steps, if successful, on
  //              return.
  // @returns `true` if all three step attempts were successful, `false`
  //          otherwise.
  bool AttemptStepPaired(const T& t0, const T& h, const VectorX<T>& xt0,
                         VectorX<T>* xtplus_vie, VectorX<T>* xtplus_hvie);

  // Compute the partial derivatives of the ordinary differential equations with
  // respect to the y variables of a given x(t). In particular, we compute the
  // Jacobian, Jₗ(y), of the function l(y), used in this integrator's
  // residual computation, with respect to y, where y = (v,z) and x = (q,v,z).
  // This Jacobian is then defined as:
  //     l(y)  = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)          (7)
  //     Jₗ(y) = ∂l(y)/∂y                              (8)
  // We use the Jacobian computation scheme from
  // get_jacobian_computation_scheme(), which is either a first-order forward
  // difference, a second-order centered difference, or automatic
  // differentiation. See math::ComputeNumericalGradient() for more details on
  // the first two methods.
  // @param t refers to tⁿ⁺¹, the time used in the definition of l(y)
  // @param h is the timestep size parameter, h, used in the definition of
  //        l(y)
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn refers to qⁿ, the initial position used in l(y)
  // @param [out] Jy is the Jacobian matrix, Jₗ(y).
  // @post The context's time will be set to t, and its continuous state will
  //       be indeterminate on return.
  void CalcVelocityJacobian(const T& t, const T& h, const VectorX<T>& y,
                            const VectorX<T>& qk, const VectorX<T>& qn,
                            MatrixX<T>* Jy);

  // Computes necessary matrices (Jacobian and iteration matrix) for
  // Newton-Raphson (NR) iterations, as necessary. This method is based off of
  // ImplicitIntegrator<T>::MaybeFreshenMatrices(). We implement our own version
  // here to use a specialized Jacobian Jₗ(y). The aformentioned method was
  // designed for use in DoImplicitIntegratorStep() processes that follow this
  // model:
  // 1. DoImplicitIntegratorStep(h) is called;
  // 2. One or more NR iterations is performed until either (a) convergence is
  //    identified, (b) the iteration is found to diverge, or (c) too many
  //    iterations were taken. In the case of (a), DoImplicitIntegratorStep(h)
  //    will return success. Otherwise, the Newton-Raphson process is attempted
  //    again with (i) a recomputed and refactored iteration matrix and (ii) a
  //    recomputed Jacobian and a recomputed an refactored iteration matrix, in
  //    that order. The process stage of that NR algorithm is indicated by the
  //    `trial` parameter below. In this model, DoImplicitIntegratorStep()
  //    returns failure if the NR iterations reach a fourth trial.
  //
  // We provide our own method to execute the same logic, but with the
  // following differences:
  // 1. We use the specialized Jacobian Jₗ(y) instead of the full Jacobian.
  // 2. We no longer use the get_reuse() logic to reuse a Jacobian
  //    when the time-step size (h) shrinks, because the specialized Jacobian
  //    Jₗ(y) depends on h.
  // These changes allow the velocity-implicit Euler method to use the smaller
  // specialized Jacobian Jₗ(y) in its Newton solves.
  //
  // @param t is the time at which to compute the Jacobian.
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y), which is used in the definition of Jₗ(y).
  // @param qn is the generalized position at the beginning of the step.
  // @param h is the integration step size.
  // @param trial specifies which trial (1-4) the Newton-Raphson process is in
  //        when calling this method.
  // @param compute_and_factor_iteration_matrix is a function pointer for
  //        computing and factoring the iteration matrix.
  // @param [in, out] iteration_matrix is the updated and factored iteration
  //        matrix on return.
  // @param [in, out] Jy is the updated and factored Jacobian matrix Jₗ(y) on
  //        return.
  // @returns `false` if the calling stepping method should indicate failure;
  //          `true` otherwise.
  // @pre 1 <= `trial` <= 4.
  // @post The internal context may or may not be altered on return; if
  //       altered, the time will be set to t and the continuous state will be
  //       indeterminate.
  bool MaybeFreshenVelocityMatrices(
      const T& t, const VectorX<T>& y, const VectorX<T>& qk,
      const VectorX<T>& qn, const T& h, int trial,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jy);

  // Computes necessary matrices (Jacobian and iteration matrix) for full
  // Newton-Raphson (NR) iterations, if full Newton-Raphson method is activated
  // (if it's not activated, this method is a no-op).
  // @param t the time at which to compute the Jacobian.
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y), which is used in the definition of Jₗ(y).
  // @param qn is qⁿ, the generalized position at the beginning of the step.
  // @param h the integration step size (for computing iteration matrices).
  // @param compute_and_factor_iteration_matrix a function pointer for
  //        computing and factoring the iteration matrix.
  // @param[out] iteration_matrix the updated and factored iteration matrix on
  //             return.
  // @param[out] Jy the updated Jacobian matrix Jₗ(y).
  // @post The internal context may or may not be altered on return; if
  //       altered, the time will be set to t and the continuous state will be
  //       indeterminate.
  void FreshenVelocityMatricesIfFullNewton(
      const T& t, const VectorX<T>& y, const VectorX<T>& qk,
      const VectorX<T>& qn, const T& h,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jy);

  // This helper method evaluates the Newton-Raphson residual R(y), defined as
  // the following:
  //     R(y)  = y - yⁿ - h l(y),
  //     l(y) = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),          (7)
  // with tⁿ⁺¹, y = (v, z), qₖ, qⁿ, yⁿ, and h passed in.
  // @param t refers to tⁿ⁺¹, the time at which to compute the residual R(y).
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate R(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn is qⁿ, the generalized position at the beginning of the step.
  // @param yn is yⁿ, the generalized velocity and miscellaneous states at the
  //        beginning of the step
  // @param h is the step size.
  // @param [in, out] qdot is a temporary BasicVector<T> of the same size as qⁿ
  //        allocated by the caller so that this method avoids unnecessary heap
  //        allocations. Its value is indeterminate upon return.
  // @param [out] result is set to R(y).
  // @post The context is set to (tⁿ⁺¹, qⁿ + h N(qₖ) v, y).
  VectorX<T> ComputeResidualR(const T& t, const VectorX<T>& y,
                              const VectorX<T>& qk, const VectorX<T>& qn,
                              const VectorX<T>& yn, const T& h,
                              BasicVector<T>* qdot);

  // This helper method evaluates l(y), defined as the following:
  //     l(y) = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),          (7)
  // with tⁿ⁺¹, y = (v, z), qₖ, qⁿ, yⁿ, and h passed in.
  // @param t refers to tⁿ⁺¹, the time at which to compute the residual R(y).
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate l(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn is qⁿ, the generalized position at the beginning of the step.
  // @param h is the step size.
  // @param [in, out] qdot is a temporary BasicVector<T> of the same size as qⁿ
  //        allocated by the caller so that this method avoids unnecessary heap
  //        allocations. Its value is indeterminate upon return.
  // @param [out] result is set to l(y).
  // @post The context is set to (tⁿ⁺¹, qⁿ + h N(qₖ) v, y).
  VectorX<T> ComputeLOfY(const T& t, const VectorX<T>& y, const VectorX<T>& qk,
                         const VectorX<T>& qn, const T& h,
                         BasicVector<T>* qdot);

  // The last computed iteration matrix and factorization.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_vie_;

  // Vector used in error estimate calculations. At the end of every step, we
  // set this to ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹, which is our estimate for ε = x̃ⁿ⁺¹ - xⁿ⁺¹,
  // the error of the propagated half-sized steps.
  VectorX<T> err_est_vec_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Variables to avoid heap allocations.
  VectorX<T> xn_, xdot_, xtplus_vie_, xtplus_hvie_;
  std::unique_ptr<BasicVector<T>> qdot_{nullptr};

  // The last computed velocity+misc Jacobian matrix.
  MatrixX<T> Jy_vie_;

  // Various statistics.
  int64_t num_nr_iterations_{0};

  // Half-sized-step-specific statistics, only updated when taking the half-
  // sized steps.
  int64_t num_half_vie_jacobian_reforms_{0};
  int64_t num_half_vie_iter_factorizations_{0};
  int64_t num_half_vie_function_evaluations_{0};
  int64_t num_half_vie_jacobian_function_evaluations_{0};
  int64_t num_half_vie_nr_iterations_{0};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::VelocityImplicitEulerIntegrator)
