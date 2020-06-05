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

  /// The integrator supports error estimation.
  bool supports_error_estimation() const final { return true; }

  /// This first-order integrator uses embedded second order methods to compute
  /// estimates of the local truncation error. The order of the asymptotic
  /// difference between these two methods (from which the error estimate is
  /// computed) is O(h²).
  int get_error_estimate_order() const final { return 2; }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return num_err_est_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return num_err_est_jacobian_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations()
      const final {
    return num_err_est_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return num_err_est_jacobian_reforms_;
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return num_err_est_iter_factorizations_;
  }

  void DoResetCachedJacobianRelatedMatrices() final;

  void DoResetImplicitIntegratorStatistics() final;

  static void ComputeAndFactorImplicitEulerIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  static void ComputeAndFactorImplicitTrapezoidIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  void DoInitialize() final;

  // Steps both implicit Euler and implicit trapezoid forward by h, if possible.
  // @param t0 the time at the left end of the integration interval.
  // @param h the integration step size to attempt.
  // @param xt0 the continuous state at t0.
  // @param [out] xtplus_euler contains the Euler integrator solution (i.e.,
  //              `x(t0+h)`) on return.
  // @param [out] xtplus_trap contains the implicit trapezoid solution (i.e.,
  //              `x(t0+h)`) on return.
  // @returns `true` if the step of size `h` was successful.
  bool AttemptStepPaired(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus_euler, VectorX<T>* xtplus_trap);

  // Performs the bulk of the stepping computation for both implicit Euler and
  // implicit trapezoid method; all those methods need to do is provide a
  // residual function (`g`) and an iteration matrix computation and
  // factorization function (`compute_and_factor_iteration_matrix`) specific to
  // the particular integrator scheme and this method does the rest.
  // @param t0 the time at the left end of the integration interval.
  // @param h the integration step size (> 0) to attempt.
  // @param xt0 the continuous state at t0.
  // @param g the particular implicit function to compute the root of.
  // @param compute_and_factor_iteration_matrix the function for computing and
  //        factorizing the iteration matrix.
  // @param xtplus_guess the starting guess for x(t0+h) -- implicit Euler passes
  //        x(t0) since it has no better guess; implicit trapezoid passes
  //        implicit Euler's result for x(t0+h).
  // @param[out] iteration_matrix the iteration matrix to be used for the
  //             particular integration scheme (implicit Euler, implicit
  //             trapezoid), which will be computed and factored, if necessary.
  // @param[out] xtplus the value for x(t0+h) on return.
  // @param trial the attempt for this approach (1-4). StepAbstract() uses more
  //        computationally expensive methods as the trial numbers increase.
  // @returns `true` if the method was successfully able to take an integration
  //           step of size h.
  // @note The time and continuous state in the context are indeterminate upon
  //       exit.
  // TODO(edrumwri) Explicitly test this method's fallback logic (i.e., how it
  //                calls MaybeFreshenMatrices()) in a unit test).
  bool StepAbstract(const T& t0, const T& h, const VectorX<T>& xt0,
                    const std::function<VectorX<T>()>& g,
                    const std::function<
                        void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
                        compute_and_factor_iteration_matrix,
                    const VectorX<T>& xtplus_guess,
                    typename ImplicitIntegrator<T>::IterationMatrix*
                    iteration_matrix, VectorX<T>* xtplus, int trial = 1);

  // Takes a given step of the requested size, if possible.
  // @returns `true` if successful; on `true`, the time and continuous state
  //          will be advanced in the context (e.g., from t0 to t0 + h). On a
  //          `false` return, the time and continuous state in the context will
  //          be restored to its original value (at t0).
  bool DoImplicitIntegratorStep(const T& h) final;

  // Steps the system forward by a single step of at most h using the implicit
  // Euler method.
  // @param t0 the time at the left end of the integration interval.
  // @param h the maximum time increment to step forward.
  // @param xt0 the continuous state at t0.
  // @param[out] xtplus the computed value for `x(t0+h)` on successful return.
  // @returns `true` if the step of size `h` was successful.
  // @note The time and continuous state in the context are indeterminate upon
  //       exit.
  bool StepImplicitEuler(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus);

  // Steps forward by a single step of `h` using the implicit trapezoid
  // method, if possible.
  // @param t0 the time at the left end of the integration interval.
  // @param h the maximum time increment to step forward.
  // @param xt0 the continuous state at t0.
  // @param dx0 the time derivatives computed at time and state (t0, xt0).
  // @param xtplus_ie x(t0+h) computed by the implicit Euler method.
  // @param[out] xtplus x(t0+h) computed by the implicit trapezoid method on
  //             successful return.
  // @returns `true` if the step was successful.
  // @note The time and continuous state in the context are indeterminate upon
  //       exit.
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
  VectorX<T> xt0_, xdot_, xtplus_ie_, xtplus_tr_;

  // Various statistics.
  int64_t num_nr_iterations_{0};

  // Second order Runge-Kutta method for estimating the integration error when
  // the requested step size lies below the working step size.
  std::unique_ptr<RungeKutta2Integrator<T>> rk2_;

  // Implicit trapezoid specific statistics.
  int64_t num_err_est_jacobian_reforms_{0};
  int64_t num_err_est_iter_factorizations_{0};
  int64_t num_err_est_function_evaluations_{0};
  int64_t num_err_est_jacobian_function_evaluations_{0};
  int64_t num_err_est_nr_iterations_{0};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ImplicitEulerIntegrator)
