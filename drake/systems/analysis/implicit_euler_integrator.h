#pragma once

#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/math/autodiff_gradient.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, fully implicit integrator with second order error estimation.
 * @tparam T A double type.
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
 * The time complexity of this method is often dominated by the time to form
 * the Jacobian matrix consisting of the partial derivatives of the nonlinear
 * system (of `n` dimensions, where `n` is the number of state variables) taken
 * with respect to the partial derivatives of the state variables at `x(t+h)`.
 * For typical numerical differentiation, `f` will be evaluated `n` times during
 * the Jacobian formation; if we liberally assume that the derivative function
 * evaluation code runs in `O(n)` time (e.g., as it would for multi-rigid
 * body dynamics without kinematic loops), the asymptotic complexity to form
 * the Jacobian will be `O(n²)`. This Jacobian matrix needs to be formed
 * repeatedly- as often as every time the state variables are updated-
 * during the solution process. Using automatic differentiation replaces the
 * `n` derivative evaluations with what is hopefully a much less expensive
 * process, though the complexity to form the Jacobian matrix is still `O(n²)`.
 * For large `n`, the time complexity may be dominated by the `O(n³)` time
 * required to (repeatedly) solve least squares problems as part of the
 * nonlinear system solution process.
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
 */
template <class T>
class ImplicitEulerIntegrator : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitEulerIntegrator)

  ~ImplicitEulerIntegrator() override = default;

  explicit ImplicitEulerIntegrator(const System<T>& system,
                                   Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    derivs_ = system.AllocateTimeDerivatives();
  }

  ///
  /// @name Methods for getting and setting the scheme used to determine the
  ///       Jacobian matrix necessary for solving the requisite nonlinear system
  ///       if equations.
  /// @{
  /// Selecting the wrong such Jacobian determination scheme will slow (possibly
  /// critically) the implicit integration process. Automatic differentiation is
  /// recommended if the System supports it for reasons of both higher
  /// accuracy and increased speed. Forward differencing (i.e., numerical
  /// differentiation) yields the best numerically differenced Jacobian accuracy
  /// for a given unit of computational time: the total error in the
  /// forward-difference approximation is close to √ε, where ε is machine
  /// epsilon, from n forward dynamics calls (where n is the number of state
  /// variables). Central differencing yields the most accurate numerically
  /// differentiated Jacobian matrix, but expends double the computational
  /// effort for approximately 50% greater accuracy: the total error in the
  /// central-difference approximation is close to ε^(2/3), from 2n forward
  /// dynamics calls. See [Nocedal 2004, pp. 167-169].
  ///
  /// - [Nocedal 2004] J. Nocedal and S. Wright. Numerical Optimization.
  ///                  Springer, 2004.
  enum class JacobianComputationScheme {
    /// O(h) Forward differencing.
    kForwardDifference,

    /// O(h²) Central differencing.
    kCentralDifference,

    /// Automatic differentiation.
    kAutomatic
  };

  /// Sets the Jacobian computation scheme. This function can be safely called
  /// at any time (i.e., the integrator need not be re-initialized afterward).
  void set_jacobian_computation_scheme(JacobianComputationScheme scheme) {
    jacobian_scheme_ = scheme;
  }

  JacobianComputationScheme get_jacobian_computation_scheme() const {
    return jacobian_scheme_;
  }
  /// @}

  /**
   * The integrator supports error estimation.
   */
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides second order error estimates.
  int get_error_estimate_order() const override { return 2; }

  /// @name Statistics functions.
  /// The functions return statistics specific to the implicit integration
  /// process; much of the focus is on methods for returning statistics
  /// relevant to the nonlinear system of equations solution process.
  /// @{

  /// Gets the number of ODE function evaluations since the last call to
  /// ResetStatistics().
  int get_num_function_evaluations() const {
    return num_function_evaluations_;
  }

  /// Gets the number of ODE function evaluations *used only for the error
  /// estimation process* since the last call to ResetStatistics().
  int get_num_error_estimator_ode_evaluations() const {
    return num_itr_function_evaluations_;
  }

  /// Gets the number of implicit-Euler-only ODE function evaluations since
  /// the last call to ResetStatistics().
  int get_num_ieu_function_evaluations() const {
    return num_function_evaluations_ - num_itr_function_evaluations_;
  }

  /// Gets the number of ODE function evaluations *used only for computing
  /// the Jacobian matrices* since the last call to ResetStatistics().
  int get_num_jacobian_function_evaluations() const {
    return num_jacobian_function_evaluations_;
  }

  /// Gets the number of implicit-trapezoid-only ODE function evaluations *used
  /// only for computing the Jacobian matrices* since the last call to
  /// ResetStatistics().
  int get_num_itr_jacobian_function_evaluations() const {
    return num_itr_jacobian_function_evaluations_;
  }

  /// Gets the number of implicit-Euler-only ODE function evaluations *used
  /// only for computing the Jacobian matrices* since the last call to
  /// ResetStatistics().
  int get_num_ieu_jacobian_function_evaluations() const {
    return num_jacobian_function_evaluations_ -
        num_itr_jacobian_function_evaluations_;
  }

  /// Gets the number of loops used in the Newton-Raphson nonlinear systems of
  /// equation solving process since the last call to ResetStatistics().
  int get_num_newton_raphson_loops() const { return num_nr_loops_; }

  /// Gets the number of implicit-trapezoid-only loops used in the
  /// Newton-Raphson nonlinear systems of equation solving process since the
  /// last call to ResetStatistics().
  int get_num_itr_newton_raphson_loops() const { return num_itr_nr_loops_; }

  /// Gets the number of implicit-Euler-only loops used in the
  /// Newton-Raphson nonlinear systems of equation solving process since the
  /// last call to ResetStatistics().
  int get_num_ieu_newton_raphson_loops() const { return num_nr_loops_ -
        num_itr_nr_loops_; }

  /// Gets the number of Jacobian reformulations (i.e., the number of times
  /// that the Jacobian matrix was reformed) since the last call to
  /// ResetStatistics().
  int get_num_jacobian_reformulations() const { return num_jacobian_reforms_; }

  /// Gets the number of implicit-trapezoid-only Jacobian matrix reformulations
  /// since the last call to ResetStatistics().
  int get_num_itr_jacobian_reformulations() const {
    return num_itr_jacobian_reforms_; }

  /// Gets the number of implicit-Euler-only Jacobian matrix reformulations
  /// since the last call to ResetStatistics().
  int get_num_ieu_jacobian_reformulations() const {
    return num_jacobian_reforms_ - num_itr_jacobian_reforms_;
  }

  /// Gets the number of iteration matrix factorizations since the last
  /// call to ResetStatistics().
  int get_num_iteration_matrix_refactors() const {
    return num_iter_refactors_;
  }

  /// Gets the number of implicit-trapezoid-only iteration matrix factorizations
  /// since the last call to ResetStatistics().
  int get_num_itr_iter_refactors() const {
    return num_itr_iter_refactors_;
  }

  /// Gets the number of implicit-Euler-only iteration matrix factorizations
  /// since the last call to ResetStatistics().
  int get_num_ieu_iter_refactors() const {
    return num_iter_refactors_ - num_itr_iter_refactors_;
  }

  /// Gets the number of failed sub-steps (implying step halving was required
  /// to permit solving the necessary nonlinear system of equations).
  int get_num_substep_failures() const {
    return num_substep_failures_;
  }

  /// Gets the number of step size shrinkages due to sub-step failures.
  int get_num_step_shrinkages_from_substep_failures() const {
    return num_shrinkages_from_substep_failures_;
  }

  /// Gets the number of step size shrinkages due to error control.
  int get_num_step_shrinkages_from_error_control() const {
    return num_shrinkages_from_error_control_;
  }

  /// @}

  /// @name Implicit-integration-specific parameters.
  /// Parameters relevant to the nonlinear system solving process.
  /// @{

  /// Gets the tolerance below which changes to the state variables during the
  /// integration process should indicate that the Newton-Raphson process
  /// has converged. Convergence is only declared if the norms of the changes to
  /// position variables, velocity variables, and auxiliary variables are all
  /// within this tolerance. Norms are weighted as described in
  /// "Methods for weighting state variable errors" for IntegratorBase
  /// documentation; this tolerance is an absolute one.
  double get_delta_state_tolerance() const { return delta_update_tol_; }

  /// Sets the tolerance below which changes to the state variables during the
  /// integration process should indicate that the Newton-Raphson process
  /// has converged.
  /// @sa get_delta_state_tolerance()
  void set_delta_state_tolerance(double tol) { delta_update_tol_ = tol; }

  /// @}

 protected:
  void DoInitialize() override;
  std::pair<bool, T> DoStepOnceAtMost(const T& max_dt) override;
  void DoResetStatistics() override;

 private:
  T StepOnceAtMostPaired(const T& dt, VectorX<T>* xtplus_euler,
                         VectorX<T>* xtplus_trap);
  T StepAbstract(T dt,
                 const std::function<VectorX<T>(const VectorX<T>&)>& g,
                 double scale,
                 bool shrink_ok,
                 VectorX<T>* xtplus);
  MatrixX<T> CalcJacobian(const T& tf, const VectorX<T>& xtplus);
  void DoStepOnceFixedSize(const T& dt) override;
  T StepImplicitEuler(const T& dt);
  T StepImplicitTrapezoid(const T& dt, const VectorX<T>& dx0,
                          VectorX<T>* xtplus);
  MatrixX<T> ComputeFDiffJacobianF(const VectorX<T>& xtplus);
  MatrixX<T> ComputeCDiffJacobianF(const VectorX<T>& xtplus);
  Eigen::MatrixXd ComputeADiffJacobianF(const Eigen::VectorXd& xtplus);
  VectorX<T> CalcTimeDerivatives(const VectorX<T>& x);
  void CalcErrorNorms(const Context<T>& context, T* q_nrm, T* v_nrm, T* z_nrm);

  // The tolerance at which the updates to the state variables indicate that
  // no change is effectively occurring, thereby allowing the nonlinear system
  // solving process to complete.
  double delta_update_tol_{std::sqrt(std::numeric_limits<double>::epsilon())};

  // This is a pre-allocated temporary for use by integration. It stores
  // the derivatives computed at x(t+h).
  std::unique_ptr<ContinuousState<T>> derivs_;

  // A simple LU factorization is all that is needed; robustness in the solve
  // comes naturally as dt << 1.
  Eigen::PartialPivLU<MatrixX<T>> LU_;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // The pseudo-inverse of the matrix that converts time derivatives of
  // generalized coordinates to generalized velocities, multiplied by the
  // change in the generalized coordinates (used in update check tolerance
  // calculations).
  std::unique_ptr<VectorBase<T>> pinvN_ddq_;

  // Vectors used in update check tolerance calculations.
  VectorX<T> unweighted_delta_;
  std::unique_ptr<VectorBase<T>> weighted_dq_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // The scheme to be used for computing the Jacobian matrix during the
  // nonlinear system solve process.
  JacobianComputationScheme jacobian_scheme_{
      JacobianComputationScheme::kForwardDifference};

  // The Jacobian matrix.
  MatrixX<T> J_;

  // The computed iteration matrix.
  MatrixX<T> A_;

  // Statistics that indicate why step sizes decrease.
  int num_shrinkages_from_error_control_{0};
  int num_shrinkages_from_substep_failures_{0};
  int num_substep_failures_{0};

  // Various combined statistics.
  int num_jacobian_reforms_{0};
  int num_iter_refactors_{0};
  int num_function_evaluations_{0};
  int num_jacobian_function_evaluations_{0};
  int num_nr_loops_{0};

  // Implicit trapezoid specific statistics.
  int num_itr_jacobian_reforms_{0};
  int num_itr_iter_refactors_{0};
  int num_itr_function_evaluations_{0};
  int num_itr_jacobian_function_evaluations_{0};
  int num_itr_nr_loops_{0};
};
}  // namespace systems
}  // namespace drake
