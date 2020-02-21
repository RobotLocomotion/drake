#pragma once

#include <memory>
#include <stdexcept>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

namespace internal {
#ifndef DRAKE_DOXYGEN_CXX
__attribute__((noreturn)) inline void EmitNoErrorEstimatorStatAndMessage() {
  throw std::logic_error(
      "No error estimator is currently implemented, so "
      "query error estimator statistics is not yet supported.");
}
#endif
}  // namespace internal

/**
 * A first-order, fully implicit integrator optimized for second-order systems,
 * with a second-order error estimate.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
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
 * This integrator requires a system of ordinary differential equations in
 * state `x = (q,v,z)` to be expressible in the following form:
 *
 *     q̇ = N(q) v;                            (1)
 *     ẏ = f_y(t,q,y),                        (2)
 * where `q̇` and `v` are linearly related via the kinematic mapping `N(q)`, 
 * `y = (v,z)`, and `f_y` is a function that can depend on the time and state.
 *
 * Implicit Euler uses the following update rule at time step n:
 *
 *     qⁿ⁺¹ = qⁿ + h N(qⁿ⁺¹) vⁿ⁺¹;            (3)
 *     yⁿ⁺¹ = yⁿ + h f_y(tⁿ⁺¹,qⁿ⁺¹,yⁿ⁺¹).     (4)
 *
 * To solve the nonlinear system for `(qⁿ⁺¹,yⁿ⁺¹)`, the velocity-implicit Euler
 * integrator iterates with a modified Newton's method: At iteration `k`, it 
 * finds a `(qₖ₊₁,yₖ₊₁)` that attempts to satisfy
 *
 *     qₖ₊₁ = qⁿ + h N(qₖ) vₖ₊₁.              (5)
 *     yₖ₊₁ = yⁿ + h f_y(tⁿ⁺¹,qₖ₊₁,yₖ₊₁);     (6)
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
 *     l(y) = f_y(tⁿ⁺¹,qⁿ + h N(qₖ) v,y),     (7)
 *     Jₗ(y) = ∂l(y) / ∂y.                    (8)
 *
 * To advance the Newton step, the velocity-implicit Euler integrator solves
 * the following linear equation for `Δy`:
 *
 *     (I - h Jₗ) Δy = - R(yₖ),               (9)
 * where `R(y) = y - yⁿ - h l(y)` and `Δy = yₖ₊₁ - yₖ`. The `Δy` solution
 * directly gives us `yₖ₊₁`. It then substitutes the `vₖ₊₁` component of `yₖ₊₁`
 * in (5) to get `qₖ₊₁`.
 *
 * This implementation uses a Newton method and relies upon the obvious
 * convergence to a solution for `y` in `R(y) = 0` where
 * `R(y) = y - yⁿ - h l(y)` as `h` becomes sufficiently small.
 * General implementational details were gleaned from [Hairer, 1996].
 *
 * In this integrator, we simultaneously take a large step at the requested
 * step size of h as well as two half-sized steps each with step size h/2.
 * The solution at h/2 is propagated as the solution, while the difference
 * is used as the error estimate, which is accurate to second order.
 *
 * To see this, let x̅ⁿ⁺¹ be the computed solution from a large step, x̃ⁿ⁺¹ be
 * the computed solution from two small steps, and xⁿ⁺¹ be the true solution.
 * Upon Newton-Raphson convergence, the truncation error,
 * e(tⁿ+h, h) = x̅ⁿ⁺¹ - xⁿ⁺¹, from equations (3-4) can be expressed as
 *
 *     e(tⁿ+h, h) = c(tⁿ) h² + O(h³),         (10)
 * where c(tⁿ) depends on the higher-order time derivatives of the system. This 
 * expression stems from the well-known fact that implicit Euler has second-
 * order truncation error. With the half-sized steps, we have that
 *
 *     x̃ⁿ⁺¹ - xⁿ⁺¹ = e(tⁿ+h/2, h/2) + e(tⁿ+h, h/2) + O(h³),
 *                 = c(tⁿ) h²/4 + c(tⁿ+h/2) h²/4 + O(h³),
 *     x̃ⁿ⁺¹ - xⁿ⁺¹ = c(tⁿ) h²/2 + O(h³),
 * from the fact that c(tⁿ+h/2) = c(tⁿ) + O(h). Therefore, 
 *
 *     x̅ⁿ⁺¹ - x̃ⁿ⁺¹ = e(tⁿ+h, h) - (x̃ⁿ⁺¹ - xⁿ⁺¹),
 *                 = c(tⁿ) h²/2 + O(h³),
 *     x̅ⁿ⁺¹ - x̃ⁿ⁺¹ = x̃ⁿ⁺¹ - xⁿ⁺¹ + O(h³).     (11)
 *
 * Our error estimate, x̅ⁿ⁺¹ - x̃ⁿ⁺¹, therefore estimates the true error,
 * x̃ⁿ⁺¹ - xⁿ⁺¹, to within O(h³). Therefore we say that it is a second-order
 * error estimate.
 *
 * Note: In statistics reported by IntegratorBase, all statistics that deal
 * with "step_size_taken" or "steps_taken" refer to the small half-sized steps
 * (as these are propagated steps). This includes the statistic tracking the
 * number of steps taken: it tracks the number of small half-steps taken. On
 * the other hand, all the step-size limits are applied to the large step, and
 * all other step-size statistics refer to the large step. Furthermore, because
 * the large step is performed first, followed by the two small steps, all
 * statistics on error estimation correspond to the work incurred by taking the
 * small steps. We made this choice because most of the work in constructing
 * and factorizing matrices, which should not be counted toward error
 * estimation, is performed during the large steps.
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996, Section IV.8, p. 118–130.
 *
 * @note This integrator uses the integrator accuracy setting, even when run
 *       in fixed-step mode, to limit the error in the underlying Newton-Raphson
 *       process. See IntegratorBase::set_target_accuracy() for more info.
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
 * @see ImplicitEulerIntegrator class documentation for information about
 *      the "implicit Euler" integration method.
 */
template <class T>
class VelocityImplicitEulerIntegrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityImplicitEulerIntegrator)

  ~VelocityImplicitEulerIntegrator() override = default;

  explicit VelocityImplicitEulerIntegrator(const System<T>& system,
                                           Context<T>* context = nullptr)
      : ImplicitIntegrator<T>(system, context) {
    // Indicate that this integrator propagates half-sized steps.
    this->set_integrator_propagates_half_sized_steps(true);
  }

  /// The integrator supports error estimation.
  bool supports_error_estimation() const final { return true; }

  /// The asymptotic order of the difference between the large and small steps
  /// (from which the error estimate is computed) is O(h²).
  int get_error_estimate_order() const final { return 2; }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  // While these methods are labeled "error estimator", it in reality counts
  // the work performed by the half-sized steps.
  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return num_err_est_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return num_err_est_jacobian_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations() const final {
    return num_err_est_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return num_err_est_jacobian_reforms_;
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return num_err_est_iter_factorizations_;
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
  // velocity-implicit Euler method, and keeps track of statistics separately.
  // This method calls StepVelocityImplicitEuler() up to twice to perform the
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
  //     l(y)  = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)   (7)
  //     Jₗ(y) = ∂l(y)/∂y                       (8)
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

  // Uses first-order forward differencing to compute the Jacobian, Jₗ(y), of
  // the function l(y), used in this integrator's residual computation, with
  // respect to y, where y = (v,z). This Jacobian is then defined as:
  //     l(y)  = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)   (7)
  //     Jₗ(y) = ∂l(y)/∂y                       (8)
  //
  // In this method, we compute the Jacobian Jₗ(y) using a first-order forward
  // difference (i.e. numerical differentiation),
  //     Jₗ(y)ᵢⱼ = (l(y')ᵢ - l(y)ᵢ )/ δy(j),
  // where y' = y + δy(j) eⱼ, δy(j) = (√ε) max(1,|yⱼ|), and eⱼ is the j-th
  // standard Cartesian basis vector.
  // In the code we hereby refer to y as "the baseline" and y' as "prime".
  // @param t refers to tⁿ⁺¹, the time used in the definition of l(y).
  // @param h is the timestep size parameter, h, used in the definition of
  //        l(y).
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn refers to qⁿ, the initial position used in l(y).
  // @param [out] Jy is the Jacobian matrix, Jₗ(y).
  // @note The context's time will be set to t, and its continuous state will
  //       be indeterminate on return.
  void ComputeForwardDiffVelocityJacobian(const T& t, const T& h,
                                          const VectorX<T>& y,
                                          const VectorX<T>& qk,
                                          const VectorX<T>& qn,
                                          MatrixX<T>* Jy);

  // Computes necessary matrices (Jacobian and iteration matrix) for
  // Newton-Raphson (NR) iterations, as necessary. This method is based off of
  // ImplicitIntegrator<T>::MaybeFreshenMatrices. We implement our own version
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
  //     l(y) = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),    (7)
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
  //     l(y) = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),    (7)
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

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Variables to avoid heap allocations.
  VectorX<T> xn_, xdot_, xtplus_vie_, xtplus_hvie_;

  // Various statistics.
  int64_t num_nr_iterations_{0};

  // The last computed velocity+misc Jacobian matrix.
  MatrixX<T> Jy_vie_;

  // Half-sized-step-specific statistics.
  // Note: While these are labeled err_est, they count the statistics for
  // the half-sized steps. This is to be consistent with the IntegratorBase
  // interface.
  int64_t num_err_est_jacobian_reforms_{0};
  int64_t num_err_est_iter_factorizations_{0};
  int64_t num_err_est_function_evaluations_{0};
  int64_t num_err_est_jacobian_function_evaluations_{0};
  int64_t num_err_est_nr_iterations_{0};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::VelocityImplicitEulerIntegrator)

