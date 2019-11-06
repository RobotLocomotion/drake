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

  // ComputeAndFactorImplicitTrapezoidIterationMatrix is NOT USED 
  //     (I will remove in final PR)
  static void ComputeAndFactorImplicitTrapezoidIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  void DoInitialize() final;
  bool AttemptStepPaired(const T& t0, const T& h, const VectorX<T>& xt0,
                         VectorX<T>* xtplus_ie, VectorX<T>* xtplus_hie);

  // StepAbstract is NOT USED (I will remove in final PR)
  bool StepAbstract(const T& t0, const T& h, const VectorX<T>& xt0,
                    const std::function<VectorX<T>()>& g,
                    const std::function<
                        void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
                        compute_and_factor_iteration_matrix,
                    VectorX<T>* xtplus, int trial = 1);

  bool DoImplicitIntegratorStep(const T& h) final;
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

  // velocity Jacobians for implicit euler and half implicit euler
  MatrixX<T>& get_mutable_velocity_jacobian_implicit_euler() { return Jv_ie_; }
  MatrixX<T>& get_mutable_velocity_jacobian_half_implicit_euler() {
    return Jv_hie_;
  }

  // Jacobian computation
  void CalcVelocityJacobian(const T& tf, const T& h, const VectorX<T>& xtplus,
                            const VectorX<T>& qt0, MatrixX<T>* Jv);
  void ComputeForwardDiffVelocityJacobian(const T& t, const T& h,
                                          const VectorX<T>& xt,
                                          const VectorX<T>& qt0,
                                          Context<T>* context, MatrixX<T>* Jv);

  // method containing the logic for sometimes refreshing the Jacobians
  bool MaybeFreshenVelocityMatrices(
      const T& t, const VectorX<T>& xt, const VectorX<T>& qt0, const T& h,
      int trial,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jv);

  // This function evaluates g(y) with y from the context. Context should be at
  // the time tf
  void eval_g_with_y_from_context(const VectorX<T>& qt0, const T& h,
                                  const VectorX<T>& qk, VectorX<T>* result);

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
