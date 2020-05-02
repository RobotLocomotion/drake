#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

/**
 * A first-order, fully implicit integrator with second order error estimation.
 *
 * This integrator uses the following update rule:<pre>
 * x(t+h) = x(t) + h f(t+h,x(t+h))
 * </pre>
 * where x are the state variables, h is the integration step size, and
 * f() returns the time derivatives of the state variables. Contrast this
 * update rule to that of an explicit first-order integrator:<pre>
 * x(t+h) = x(t) + h f(t, x(t))
 * </pre>
 * Thus implicit first-order integration must solve a nonlinear system of
 * equations to determine *both* the state at t+h and the time derivatives
 * of that state at that time. Cast as a nonlinear system of equations,
 * we seek the solution to:<pre>
 * x(t+h) - x(t) - h f(t+h,x(t+h)) = 0
 * </pre>
 * given unknowns x(t+h).
 *
 * This "implicit Euler" method is known to be L-Stable, meaning both that
 * applying it at a fixed integration step to the  "test" equation `y(t) = eᵏᵗ`
 * yields zero (for `k < 0` and `t → ∞`) *and* that it is also A-Stable.
 * A-Stability, in turn, means that the method can integrate the linear constant
 * coefficient system `dx/dt = Ax` at any step size without the solution
 * becoming unstable (growing without bound). The practical effect of
 * L-Stability is that the integrator tends to be stable for any given step size
 * on an arbitrary system of ordinary differential equations. See
 * [Lambert, 1991], Ch. 6 for an approachable discussion on stiff differential
 * equations and L- and A-Stability.
 *
 * This implementation uses Newton-Raphson (NR) and relies upon the obvious
 * convergence to a solution for `g = 0` where
 * `g(x(t+h)) ≡ x(t+h) - x(t) - h f(t+h,x(t+h))` as `h` becomes sufficiently
 * small. General implementational details for the Newton method were gleaned
 * from Section IV.8 in [Hairer, 1996].
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
 * the true error vector as `ε = x̃ⁿ⁺¹ - xⁿ⁺¹`.ImplicitEulerIntegrator uses
 * `ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹`, the difference between the two solutions, as the
 * second-order error estimate, because for a smooth system, `‖ε*‖ = O(h²)`,
 * and `‖ε - ε*‖ = O(h³)`. See the notes in
 * ImplicitEulerIntegrator<T>::get_error_estimate_order() for a detailed
 * derivation of the error estimate's truncation error.
 *
 * In this implementation, ImplicitEulerIntegrator<T> attempts the large
 * full-sized step before attempting the two small half-sized steps, because
 * the large step is more likely to fail to converge, and if it is performed
 * first, convergence failures are detected early, avoiding the unnecessary
 * effort of computing potentially-successful small steps.
 *
 * If the user desires, ImplicitEulerIntegrator<T> can also use the implicit
 * trapezoid method for error estimation: it will feed the result from implicit
 * Euler for (hopefully) faster convergence- to compute the error estimate.
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996, Section IV.8, p. 118–130.
 * - [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
 *                    Equations. John Wiley & Sons, 1991.
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
 *       in fixed-step mode, to limit the error in the underlying Newton-Raphson
 *       process. See IntegratorBase::set_target_accuracy() for more info.
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
 *
 * @tparam_nonsymbolic_scalar
 * @ingroup integrators
 */
template <class T>
class ImplicitEulerIntegrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitEulerIntegrator)

  ~ImplicitEulerIntegrator() override = default;

  explicit ImplicitEulerIntegrator(const System<T>& system,
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
   * This derivation is based on the same derivation for
   * VelocityImplicitEulerIntegrator<T>, and so the equation numbers are from
   * there.
   *
   * To derive the second-order error estimate, let us first define the vector-
   * valued function `e(tⁿ, h, xⁿ) = x̅ⁿ⁺¹ - xⁿ⁺¹`, the local truncation error
   * for a single, full-sized implicit Euler integration step, with
   * initial conditions `(tⁿ, xⁿ)`, and a step size of `h`. Furthermore, use
   * `ẍ` to denote `df/dt`, and `∇f` and `∇ẍ` to denote the Jacobians `df/dx`
   * and `dẍ/dx` of the ODE system `ẋ = f(t, x)`. Note that `ẍ` uses a total
   * time derivative, i.e., `ẍ = ∂f/∂t + ∇f f`.
   *
   * Let us use `x*` to denote the true solution after a half-step, `x(tⁿ+½h)`,
   * and `x̃*` to denote the implicit Euler solution after a single
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
   * Let us look at a single implicit Euler step. Upon Newton-Raphson
   * convergence, the truncation error for implicit Euler is
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

  /* Set this to true to use implicit trapezoid, for error estimation;
   * otherwise this integrator will use step doubling, for error estimation.
   * By default, this integrator will use step doubling.
   */
  void set_use_implicit_trapezoid_error_estimation(bool flag) {
    use_implicit_trapezoid_error_estimation_ = flag;
  }

  /* Returns true if the integrator will use implicit trapezoid for error
   * estimation; otherwise it indicates the integrator will use step doubling
   * for error estimation.
   */
  bool get_use_implicit_trapezoid_error_estimation() {
    return use_implicit_trapezoid_error_estimation_;
  }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    // When implicit trapezoid is chosen, implicit trapezoid is the error
    // estimator, and statistics for it are directly reported; otherwise, the
    // small half-sized steps are propagated and the large step is the error
    // estimator, so we report error estimator stats by subtracting those of
    // the small half-sized steps from the total statistics.
    return use_implicit_trapezoid_error_estimation_
               ? num_itr_or_half_ie_function_evaluations_
               : (this->get_num_derivative_evaluations() -
                  num_itr_or_half_ie_function_evaluations_);
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    // When implicit trapezoid is chosen, implicit trapezoid is the error
    // estimator, and statistics for it are directly reported; otherwise, the
    // small half-sized steps are propagated and the large step is the error
    // estimator, so we report error estimator stats by subtracting those of
    // the small half-sized steps from the total statistics.
    return use_implicit_trapezoid_error_estimation_
               ? num_itr_or_half_ie_jacobian_function_evaluations_
               : (this->get_num_derivative_evaluations_for_jacobian() -
                  num_itr_or_half_ie_jacobian_function_evaluations_);
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations()
      const final {
    // When implicit trapezoid is chosen, implicit trapezoid is the error
    // estimator, and statistics for it are directly reported; otherwise, the
    // small half-sized steps are propagated and the large step is the error
    // estimator, so we report error estimator stats by subtracting those of
    // the small half-sized steps from the total statistics.
    return use_implicit_trapezoid_error_estimation_
               ? num_itr_or_half_ie_nr_iterations_
               : (this->get_num_newton_raphson_iterations() -
                  num_itr_or_half_ie_nr_iterations_);
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    // When implicit trapezoid is chosen, implicit trapezoid is the error
    // estimator, and statistics for it are directly reported; otherwise, the
    // small half-sized steps are propagated and the large step is the error
    // estimator, so we report error estimator stats by subtracting those of
    // the small half-sized steps from the total statistics.
    return use_implicit_trapezoid_error_estimation_
               ? num_itr_or_half_ie_jacobian_reforms_
               : (this->get_num_jacobian_evaluations() -
                  num_itr_or_half_ie_jacobian_reforms_);
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    // When implicit trapezoid is chosen, implicit trapezoid is the error
    // estimator, and statistics for it are directly reported; otherwise, the
    // small half-sized steps are propagated and the large step is the error
    // estimator, so we report error estimator stats by subtracting those of
    // the small half-sized steps from the total statistics.
    return use_implicit_trapezoid_error_estimation_
               ? num_itr_or_half_ie_iter_factorizations_
               : (this->get_num_iteration_matrix_factorizations() -
                  num_itr_or_half_ie_iter_factorizations_);
  }

  void DoResetImplicitIntegratorStatistics() final;
  static void ComputeAndFactorImplicitEulerIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);
  static void ComputeAndFactorImplicitTrapezoidIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);
  void DoInitialize() final;
  bool AttemptStepPaired(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus_ie, VectorX<T>* xtplus_hie);
  bool StepAbstract(const T& t0, const T& h, const VectorX<T>& xt0,
                    const std::function<VectorX<T>()>& g,
                    const std::function<
                        void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
                        compute_and_factor_iteration_matrix,
                    const VectorX<T>& xtplus_guess,
                    typename ImplicitIntegrator<T>::IterationMatrix*,
                    VectorX<T>* xtplus, int trial = 1);
  bool DoImplicitIntegratorStep(const T& h) final;
  bool StepImplicitEuler(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus);
  bool StepImplicitEulerWithGuess(const T& t0, const T& h,
      const VectorX<T>& xt0, const VectorX<T>& xtplus_guess,
      VectorX<T>* xtplus);
  bool StepHalfSizedImplicitEulers(const T& t0, const T& h,
      const VectorX<T>& xt0, const VectorX<T>& xtplus_ie, VectorX<T>* xtplus);
  bool StepImplicitTrapezoid(const T& t0, const T& h, const VectorX<T>& xt0,
      const VectorX<T>& dx0, const VectorX<T>& xtplus_ie, VectorX<T>* xtplus);

  // The last computed iteration matrix and factorization for implicit Euler.
  typename ImplicitIntegrator<T>::IterationMatrix ie_iteration_matrix_;

  // The last computed iteration matrix and factorization for implicit
  // trapezoid.
  typename ImplicitIntegrator<T>::IterationMatrix itr_iteration_matrix_;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Variables to avoid heap allocations.
  VectorX<T> xt0_, xdot_, xtplus_ie_, xtplus_hie_;

  // Various statistics.
  int64_t num_nr_iterations_{0};

  // Second order Runge-Kutta method for estimating the integration error when
  // the requested step size lies below the working step size.
  std::unique_ptr<RungeKutta2Integrator<T>> rk2_;

  // Statistics specific to implicit trapezoid or the two half-sized steps.
  int64_t num_itr_or_half_ie_jacobian_reforms_{0};
  int64_t num_itr_or_half_ie_iter_factorizations_{0};
  int64_t num_itr_or_half_ie_function_evaluations_{0};
  int64_t num_itr_or_half_ie_jacobian_function_evaluations_{0};
  int64_t num_itr_or_half_ie_nr_iterations_{0};

  // If set to true, the integrator uses implicit trapezoid instead of two
  // half-sized steps for error estimation.
  bool use_implicit_trapezoid_error_estimation_{false};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ImplicitEulerIntegrator)
