#pragma once

#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

/**
 * A first-order, fully implicit integrator with second order error estimation.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to https://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following kinds of T's are provided:
 *
 * - double
 *
 * TODO(antequ): support AutodiffXd
 * TODO(antequ): update this documentation to match the google doc.
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
 * small. It also uses the implicit trapezoid method- fed the result from
 * implicit Euler for (hopefully) faster convergence- to compute the error
 * estimate. General implementational details were gleaned from [Hairer, 1996].
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996.
 * - [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
 *                    Equations. John Wiley & Sons, 1991.
 *
 * @note This integrator uses the integrator accuracy setting, even when run
 *       in fixed-step mode, to limit the error in the underlying Newton-Raphson
 *       process. See IntegratorBase::set_target_accuracy() for more info.
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
 */
template <class T>
class SecondOrderImplicitEulerIntegrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SecondOrderImplicitEulerIntegrator)

  ~SecondOrderImplicitEulerIntegrator() override = default;

  explicit SecondOrderImplicitEulerIntegrator(const System<T>& system,
                                              Context<T>* context = nullptr)
      : ImplicitIntegrator<T>(system, context) {}

  /// The integrator supports error estimation.
  bool supports_error_estimation() const final { return true; }

  /// The asymptotic order of the difference between the large and small steps
  /// (from which the error estimate is computed) is O(h²).
  int get_error_estimate_order() const final { return 2; }

  /* Because the IntegratorBase dt is different from the half-step hs actually
     used, we provide modified statistics here. */
  // TODO(antequ): redesign this API in IntegratorBase so that the step size
  //    adjustment logic still works while returning the true sizes
  //    to the public
  T get_soie_actual_initial_step_size_taken() const {
    return 0.5 * IntegratorBase<T>::get_actual_initial_step_size_taken();
  }
  T get_soie_smallest_adapted_step_size_taken() const {
    return 0.5 * IntegratorBase<T>::get_smallest_adapted_step_size_taken();
  }
  T get_soie_largest_step_size_taken() const {
    return 0.5 * IntegratorBase<T>::get_largest_step_size_taken();
  }

  int get_soie_num_steps_taken() const {
    return 2 * IntegratorBase<T>::get_num_steps_taken();
  }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_newton_raphson_iterations_that_end_in_failure()
      const final {
    return num_nr_iterations_that_end_in_failure_;
  }

  int64_t do_get_num_newton_raphson_failures() const final {
    return num_nr_failures_;
  }

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

  int64_t
  do_get_num_error_estimator_newton_raphson_iterations_that_end_in_failure()
      const final {
    return num_err_est_nr_iterations_that_end_in_failure_;
  }

  int64_t do_get_num_error_estimator_newton_raphson_failures() const final {
    return num_err_est_nr_failures_;
  }

  void DoResetImplicitIntegratorStatistics() final;
  static void ComputeAndFactorImplicitEulerIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  void DoInitialize() final;
  bool AttemptStepPaired(const T& t0, const T& h, const VectorX<T>& xt0,
                         VectorX<T>* xtplus_ie, VectorX<T>* xtplus_hie);

  bool DoImplicitIntegratorStep(const T& h) final;

  /// Steps the system forward by a single step of at most h using the
  /// Second-Order Implicit Euler method.
  /// @param t0 the time at the left end of the integration interval.
  /// @param h the maximum time increment to step forward.
  /// @param xt0 the continuous state at t0.
  /// @param xtplus_guess the starting guess for x(t0+h).
  /// @param [out] xtplus the computed value for `x(t0+h)` on successful return.
  /// @param [in, out] iteration_matrix the cached iteration matrix
  /// @param [in, out] iteration_matrix the cached velocity Jacobian
  /// @param trial the attempt for this approach (1-4). StepImplicitEuler() uses
  ///        more computationally expensive methods as the trial numbers
  ///        increase.
  /// @returns `true` if the step of size `h` was successful, `false` otherwise.
  /// @note The time and continuous state in the context are indeterminate upon
  ///       exit.
  bool StepImplicitEuler(
      const T& t0, const T& h, const VectorX<T>& xt0,
      const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jv, int trial = 1);
  bool StepHalfImplicitEulers(
      const T& t0, const T& h, const VectorX<T>& xt0,
      const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jv);

  /// velocity Jacobians for implicit euler and half implicit euler
  MatrixX<T>& get_mutable_velocity_jacobian_implicit_euler() { return Jv_ie_; }
  MatrixX<T>& get_mutable_velocity_jacobian_half_implicit_euler() {
    return Jv_hie_;
  }

  /// Compute the partial derivative of the ordinary differential equations with
  /// respect to the y variables of a given x(t). In particular, we compute the
  /// Jacobian, Jₗₖ(y), of the function lₖ(y), used in this integrator's
  /// residual computation, with respect to y. As defined before, y = (v,z) and
  /// x = (q,v,z). This Jacobian is then defined as:
  ///     lₖ(y)  = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)    (9)
  ///     Jₗₖ(y) = ∂lₖ(y)/∂y                     (10)
  /// @param t refers to tⁿ⁺¹, the time used in the definition of lₖ(y)
  /// @param h is the timestep size parameter, h, used in the definition of
  ///        lₖ(y)
  /// @param x is (qₖ, y), the continuous state around which to evaluate Jₗₖ(y)
  /// @param qt0 refers to qⁿ, the initial position used in lₖ(y)
  /// @param [out] Jv is the Jacobian matrix, Jₗₖ(y).
  /// @post the context's time and continuous state will be temporarily set
  ///       during this call (and then reset to their original values) on
  ///       return.
  void CalcVelocityJacobian(const T& tf, const T& h, const VectorX<T>& xtplus,
                            const VectorX<T>& qt0, MatrixX<T>* Jv);

  /// Uses first-order forward differencing to compute the Jacobian, Jₗₖ(y), of
  /// the function lₖ(y), used in this integrator's residual computation, with
  /// respect to y. As defined before, y = (v, z). This Jacobian is then defined
  /// as:
  ///     lₖ(y)  = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)    (9)
  ///     Jₗₖ(y) = ∂lₖ(y)/∂y                     (10)
  //
  /// In this method, we compute the Jacobian Jₗₖ(y) using a first-order forward
  /// difference (i.e. numerical differentiation),
  ///   Jₗₖ(y)ᵢⱼ = (lₖ(y')ᵢ - lₖ(y)ᵢ )/ δy(j),
  ///      where y' = y + δy(j) eⱼ and δy(j) = (√ε) max(1,|yⱼ|).
  /// In the code we hereby refer to y as "the baseline" and y' as "prime".
  /// @param t refers to tⁿ⁺¹, the time used in the definition of lₖ(y)
  /// @param h is the timestep size parameter, h, used in the definition of
  ///        lₖ(y)
  /// @param x is (qₖ, y), the continuous state around which to evaluate Jₗₖ(y)
  /// @param qt0 refers to qⁿ, the initial position used in lₖ(y)
  /// @param context the Context of the system, at time and continuous state
  ///        unknown.
  /// @param [out] Jv is the Jacobian matrix, Jₗₖ(y).
  /// @note The continuous state will be indeterminate on return.
  /// For full Newton, we recommend this method to be evaluated at time t = t0 +
  /// h and x = xₖ; however, this recommendation is not necessary for the
  /// integrator and the logic in MaybeFreshenVelocityMatrices modifies this.
  void ComputeForwardDiffVelocityJacobian(const T& t, const T& h,
                                          const VectorX<T>& xt,
                                          const VectorX<T>& qt0,
                                          Context<T>* context, MatrixX<T>* Jv);

  /// Computes necessary matrices (Jacobian and iteration matrix) for
  /// Newton-Raphson (NR) iterations, as necessary. This method is based off of
  /// ImplicitIntegrator<T>::MaybeFreshenMatrices. We implement our own version
  /// here to use a specialized Velocity Jacobian. The aformentioned method was
  /// designed for use in DoImplicitIntegratorStep() processes that follow this
  /// model:
  /// 1. DoImplicitIntegratorStep(h) is called;
  /// 2. One or more NR iterations is performed until either (a) convergence is
  ///    identified, (b) the iteration is found to diverge, or (c) too many
  ///    iterations were taken. In the case of (a), DoImplicitIntegratorStep(h)
  ///    will return success. Otherwise, the Newton-Raphson process is attempted
  ///    again with (i) a recomputed and refactored iteration matrix and (ii) a
  ///    recomputed Jacobian and a recomputed an refactored iteration matrix, in
  ///    that order. The process stage of that NR algorithm is indicated by the
  ///    `trial` parameter below. In this model, DoImplicitIntegratorStep()
  ///    returns failure if the NR iterations reach a fourth trial.
  ///
  /// Note that the sophisticated logic above only applies when the Jacobian
  /// reuse is activated (default, see get_reuse()).
  ///
  /// @param t the time at which to compute the Jacobian.
  /// @param xt the continuous state at which the Jacobian is computed.
  /// @param qt0 the generalized position state at the beginning of the step
  /// @param h the integration step size
  /// @param trial which trial (1-4) the Newton-Raphson process is in when
  ///        calling this method.
  /// @param compute_and_factor_iteration_matrix a function pointer for
  ///        computing and factoring the iteration matrix.
  /// @param [out] iteration_matrix the updated and factored iteration matrix on
  ///             return.
  /// @param [out] Jv the updated and factored velocity Jacobian matrix on
  ///             return.
  /// @returns `false` if the calling stepping method should indicate failure;
  ///          `true` otherwise.
  /// @pre 1 <= `trial` <= 4.
  /// @post the state in the internal context may or may not be altered on
  ///       return; if altered, it will be set to (t, xt).
  bool MaybeFreshenVelocityMatrices(
      const T& t, const VectorX<T>& xt, const VectorX<T>& qt0, const T& h,
      int trial,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jv);

  /// This helper method evaluates l(y) with y from the context. The context
  /// should be at the time tf.
  /// @param qt0 the generalized position at the beginning of the step
  /// @param h the step size
  /// @param qk the generalized position to evaluate N in l(y)
  /// @param [out] result this is set to l(y) from the evaluation
  /// @post the context state is altered with q = qt0 + N(qk) v
  void eval_l_with_y_from_context_and_iterate_q(const VectorX<T>& qt0,
                                                const T& h,
                                                const VectorX<T>& qk,
                                                VectorX<T>* result);

  // The last computed iteration matrix and factorization; the _ie_ is for
  // the large step and the _hie_ is for the small step
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_ie_,
      iteration_matrix_hie_;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Variables to avoid heap allocations.
  VectorX<T> xt0_, xdot_, xtplus_ie_, xtplus_hie_;

  // Various statistics.
  int64_t num_nr_iterations_{0};
  int64_t num_nr_iterations_that_end_in_failure_{0};
  int64_t num_nr_failures_{0};

  // The last computed velocity+misc Jacobian matrices.
  MatrixX<T> Jv_ie_;
  MatrixX<T> Jv_hie_;

  // Second order Runge-Kutta method for estimating the integration error when
  // the requested step size lies below the working step size.
  std::unique_ptr<RungeKutta2Integrator<T>> rk2_;

  // Implicit trapezoid specific statistics.
  int64_t num_err_est_jacobian_reforms_{0};
  int64_t num_err_est_iter_factorizations_{0};
  int64_t num_err_est_function_evaluations_{0};
  int64_t num_err_est_jacobian_function_evaluations_{0};
  int64_t num_err_est_nr_iterations_{0};
  int64_t num_err_est_nr_iterations_that_end_in_failure_{0};
  int64_t num_err_est_nr_failures_{0};
};
}  // namespace systems
}  // namespace drake
