#pragma once

#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/math/jacobian.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, fully implicit integrator without error estimation.
 * @tparam T A double or autodiff type.
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to http://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following kinds of T's are provided:
 * - double
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
 * This "implicit Euler" method is known to be A-Stable, meaning that applying
 * it at a fixed integration step to the  "test" equation `y(t) = eᵏᵗ` yields
 * zero (for `k < 0` and `t → ∞`). The practical effect is that the integrator
 * tends to be stable for any given step size on a system of ordinary
 * differential equations, given that the nonlinear system of equations is
 * solvable (which typically depends on the step size, `h`).
 *
 * The time complexity of this method is often dominated by the time to form
 * the Jacobian matrix consisting of the partial derivatives of the nonlinear
 * system (of `n` dimensions, where `n` is the number of state variables) taken
 * with respect to the partial derivatives of the state variables at `x(t+h)`.
 * For typical numerical differentiation, `f` will be evaluated `n` times during
 * the Jacobian formation; if we liberally assume that the derivative function
 * evaluation code runs in `O(n)` time (e.g., as it would for multi-rigid
 * body dynamics without kinematic loops), the asymptotic complexity to form
 * the Jacobian will be `O(n²)`. This Jacobian matrix needs to be formed
 * repeatedly- every time the state variables are updated- during the solution
 * process, or one must cleverly attempt to reuse the Jacobian (repeatedly, if
 * possible). Using automatic differentiation replaces the `n` derivative
 * evaluations with what is hopefully a much less expensive process, though the
 * complexity to form the Jacobian matrix is still `O(n²)`. For large `n`, the
 * time complexity will be dominated by the `O(n³)` time required to
 * (repeatedly) solve least squares problems as part of the nonlinear system
 * solution process.
 *
 * The keys to efficiently solving the systems of nonlinear systems are (1)
 * accurate Jacobian matrices, (2) a "good" initial guess for `x(t+h)`, (3)
 * balancing efficient line search against likelihood of converging to an
 * undesirable solution.
 */
template <class T>
class ImplicitEulerIntegrator : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitEulerIntegrator)

  ~ImplicitEulerIntegrator() override = default;

  // TODO(edrumwri): consider making the convergence tolerance a parameter
  // to the constructor in ImplicitEulerIntegrator.
  explicit ImplicitEulerIntegrator(const System<T>& system,
                                   const T& max_step_size,
                                   Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs_ = system.AllocateTimeDerivatives();
  }

  /**
   * The integrator does not support error estimation.
   */
  bool supports_error_estimation() const override { return false; }

  /// Gets the tolerance below which the nonlinear system solving process will
  /// halt.
  double get_convergence_tolerance() const { return convergence_tol_; }

  /// Sets the tolerance below which the nonlinear system solving tolerance will
  /// halt.
  void set_convergence_tolerance(double tol) { convergence_tol_ = tol; }

  /// Gets the error estimate order (zero).
  int get_error_estimate_order() const override { return 0; }

  /// Gets the number of times that the integrator rejected a trial step due to
  /// misdirected descent steps (i.e., the Newton-Raphson step would increase
  /// the Euclidean norm of x(t+h) - x(t) - h*f(t+h, x(t+h)).
  int get_num_misdirected_descents() const {
    return num_misdirected_descents_;
  }

  /// Gets the number of times that the integrator rejected a trial step due to
  /// an increase in the Euclidean norm of x(t+h) - x(t) - h*f(t+h, x(t+h))
  /// upon an update to x(t+h).
  int get_num_objective_function_increases() const {
    return num_objective_function_increases_;
  }

  /// Gets the number of times that the integrator rejected a trial step due to
  /// the update vector in Newton-Raphson being too small.
  int get_num_overly_small_updates() const {
    return num_overly_small_updates_; }

 protected:
  std::pair<bool, T> DoStepOnceAtMost(const T& max_dt);
  void DoResetStatistics() override;

 private:
  void RestoreTimeAndState(const T& t, const VectorX<T>& x);
  void DoStepOnceFixedSize(const T& dt) override;
  bool DoTrialStep(const T& dt);
  VectorX<T> EvaluateNonlinearEquations(const VectorX<T>& xt,
                                        const VectorX<T>& xtplus,
                                        double h);
  MatrixX<T> ComputeNDiffJacobian(const VectorX<T>& xt,
                                  const VectorX<T>& xtplus,
                                  double h);
  MatrixX<T> ComputeN2DiffJacobian(const VectorX<T>& xt,
                                   const VectorX<T>& xtplus,
                                   double h);
  Eigen::MatrixXd ComputeADiffJacobian(const Eigen::VectorXd& xt,
                                       const Eigen::VectorXd& xtplus,
                                       double h);

  // The Euclidean norm tolerance at which the nonlinear system solving
  // process will halt.
  double convergence_tol_{std::sqrt(std::numeric_limits<double>::epsilon())};

  // This is a pre-allocated temporary for use by integration. It stores
  // the derivatives computed at x(t+h).
  std::unique_ptr<ContinuousState<T>> derivs_;

  // For LU factorization with full pivoting; this yields a solution to a
  // system of linear equations that is somewhat robust.
  Eigen::FullPivLU<MatrixX<T>> LU_;

  int num_overly_small_updates_{0};
  int num_objective_function_increases_{0};
  int num_misdirected_descents_{0};
};
}  // namespace systems
}  // namespace drake
