#pragma once

#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"

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
 * - AutoDiffXd
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
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
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
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides second order error estimates.
  int get_error_estimate_order() const override { return 2; }

  /// @name Cumulative statistics functions.
  /// The functions return statistics specific to the implicit integration
  /// process. Adds to the cumulative statistics functions of
  /// ImplicitIntegrator.
  /// @{

  /// Gets the number of iterations used in the Newton-Raphson nonlinear systems
  /// of equation solving process since the last call to ResetStatistics(). This
  /// count includes those Newton-Raphson iterations used during error
  /// estimation processes.
  int64_t get_num_newton_raphson_iterations() const {
    return num_nr_iterations_;
  }

  /// Gets the number of factorizations of the iteration matrix since the last
  /// call to ResetStatistics(). This count includes those refactorizations
  /// necessary during error estimation processes.
  int64_t get_num_iteration_matrix_factorizations() const {
    return num_iter_factorizations_;
  }

  /// @}

  /// Gets the number of ODE function evaluations
  /// (calls to CalcTimeDerivatives()) *used only for the error estimation
  /// process* since the last call to ResetStatistics(). This count
  /// includes *all* such calls including (1) those necessary to compute
  /// Jacobian matrices; and (2) calls that exhibit little
  /// cost (due to results being cached).
  int64_t get_num_error_estimator_derivative_evaluations() const {
    return num_err_est_function_evaluations_;
  }

  /// @name Error-estimation statistics functions.
  /// The functions return statistics specific to the error estimation
  /// process.
  /// @{
  /// Gets the number of ODE function evaluations (calls to
  /// CalcTimeDerivatives()) *used only for computing the Jacobian matrices
  /// needed by the error estimation process* since the last call to
  /// ResetStatistics().
  int64_t get_num_error_estimator_derivative_evaluations_for_jacobian() const {
    return num_err_est_jacobian_function_evaluations_;
  }

  /// Gets the number of iterations *used in the Newton-Raphson nonlinear
  /// systems of equation solving process for the error estimation process*
  /// since the last call to ResetStatistics().
  int64_t get_num_error_estimator_newton_raphson_iterations() const { return
        num_err_est_nr_iterations_;
  }

  /// Gets the number of Jacobian matrix evaluations *used only during
  /// the error estimation process* since the last call to ResetStatistics().
  int64_t get_num_error_estimator_jacobian_evaluations() const {
    return num_err_est_jacobian_reforms_;
  }

  /// Gets the number of factorizations of the iteration matrix *used only
  /// during the error estimation process* since the last call to
  /// ResetStatistics().
  int64_t get_num_error_estimator_iteration_matrix_factorizations() const {
    return num_err_est_iter_factorizations_;
  }

  /// @}

 private:
  void ComputeAndFactorIterationMatrix(
      const MatrixX<T>& J, const T& dt, int scale);
  void DoInitialize() override;
  void DoResetStatistics() override;
  bool AttemptStepPaired(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus_euler, VectorX<T>* xtplus_trap);
  bool StepAbstract(const T& t0, const T& h, const VectorX<T>& xt0,
      const std::function<VectorX<T>()>& g, int scale,
      VectorX<T>* xtplus, int trial = 1);
  bool CalcMatrices(const T& t, const T& h, const VectorX<T>& xt, int scale,
      int trial);
  bool DoStep(const T& h) override;
  bool StepImplicitEuler(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus);
  bool StepImplicitTrapezoid(const T& t0, const T& h, const VectorX<T>& xt0,
      const VectorX<T>& dx0, VectorX<T>* xtplus);
  MatrixX<T> ComputeForwardDiffJacobian(const System<T>&, Context<T>*);
  MatrixX<T> ComputeCentralDiffJacobian(const System<T>&, Context<T>*);
  MatrixX<T> ComputeAutoDiffJacobian(const System<T>& system,
                                     const Context<T>& context);

  // The last computed iteration matrix and factorization, used for both the
  // integrator and the error estimator.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Various statistics.
  int64_t num_nr_iterations_{0};
  int64_t num_iter_factorizations_{0};

  // Implicit trapezoid specific statistics.
  int64_t num_err_est_jacobian_reforms_{0};
  int64_t num_err_est_iter_factorizations_{0};
  int64_t num_err_est_function_evaluations_{0};
  int64_t num_err_est_jacobian_function_evaluations_{0};
  int64_t num_err_est_nr_iterations_{0};
};
}  // namespace systems
}  // namespace drake
