#pragma once

#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, fully implicit integrator with second order error estimation.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to http://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following kinds of T's are provided:
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
 * required to (repeatedly) solve linear systems problems as part of the
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
class ImplicitEulerIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitEulerIntegrator)

  ~ImplicitEulerIntegrator() override = default;

  explicit ImplicitEulerIntegrator(const System<T>& system,
                                   Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    derivs_ = system.AllocateTimeDerivatives();
  }

  /// Selecting the wrong such Jacobian determination scheme will slow (possibly
  /// critically) the implicit integration process. Automatic differentiation is
  /// recommended if the System supports it for reasons of both higher
  /// accuracy and increased speed. Forward differencing (i.e., numerical
  /// differentiation) exhibits error in the approximation close to √ε, where
  /// ε is machine epsilon, from n forward dynamics calls (where n is the number
  /// of state variables). Central differencing yields the most accurate
  /// numerically differentiated Jacobian matrix, but expends double the
  /// computational effort for approximately three digits greater accuracy: the
  /// total error in the central-difference approximation is close to ε^(2/3),
  /// from 2n forward dynamics calls. See [Nocedal 2004, pp. 167-169].
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

  /// @name Methods for getting and setting the Jacobian scheme.
  ///
  /// Methods for getting and setting the scheme used to determine the
  /// Jacobian matrix necessary for solving the requisite nonlinear system
  /// if equations.
  /// @see JacobianComputationScheme
  /// @{

  /// Sets whether the integrator attempts to reuse Jacobian matrices and
  /// iteration matrix factorizations (default is `true`). Forming Jacobian
  /// matrices and factorizing iteration matrices are generally the two most
  /// expensive operations performed by this integrator. For small systems
  /// (those with on the order of ten state variables), the additional accuracy
  /// that using fresh Jacobians and factorizations buys- which can permit
  /// increased step sizes but should have no effect on solution accuracy- can
  /// outweigh the small factorization cost.
  /// @sa get_reuse
  void set_reuse(bool reuse) { reuse_ = reuse; }

  /// Gets whether the integrator attempts to reuse Jacobian matrices and
  /// iteration matrix factorizations.
  /// @sa set_reuse()
  bool get_reuse() const { return reuse_; }

  /// Sets the Jacobian computation scheme. This function can be safely called
  /// at any time (i.e., the integrator need not be re-initialized afterward).
  /// @note Discards any already-computed Jacobian matrices if the scheme
  ///       changes.
  void set_jacobian_computation_scheme(JacobianComputationScheme scheme) {
    if (jacobian_scheme_ != scheme)
      J_.resize(0, 0);
    jacobian_scheme_ = scheme;
  }

  JacobianComputationScheme get_jacobian_computation_scheme() const {
    return jacobian_scheme_;
  }
  /// @}

  /// The integrator supports error estimation.
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides second order error estimates.
  int get_error_estimate_order() const override { return 2; }

  /// @name Cumulative statistics functions.
  /// The functions return statistics specific to the implicit integration
  /// process.
  /// @{

  /// Gets the number of ODE function evaluations
  /// (calls to CalcTimeDerivatives()) *used only for the error estimation
  /// process* since the last call to ResetStatistics(). This count
  /// includes *all* such calls including (1) those necessary to compute
  /// Jacobian matrices; and (2) calls that exhibit little
  /// cost (due to results being cached).
  int64_t get_num_error_estimator_derivative_evaluations() const {
    return num_err_est_function_evaluations_;
  }

  /// Gets the number of ODE function evaluations
  /// (calls to CalcTimeDerivatives()) *used only for computing
  /// the Jacobian matrices* since the last call to ResetStatistics(). This
  /// count includes those derivative calculations necessary for computing
  /// Jacobian matrices during the error estimation process.
  int64_t get_num_derivative_evaluations_for_jacobian() const {
    return num_jacobian_function_evaluations_;
  }

  /// Gets the number of iterations used in the Newton-Raphson nonlinear systems
  /// of equation solving process since the last call to ResetStatistics(). This
  /// count includes those Newton-Raphson iterations used during the error
  /// estimation process.
  int64_t get_num_newton_raphson_iterations() const {
    return num_nr_iterations_;
  }

  /// Gets the number of Jacobian evaluations (i.e., the number of times
  /// that the Jacobian matrix was reformed) since the last call to
  /// ResetStatistics(). This count includes those evaluations necessary
  /// during the error estimation process.
  int64_t get_num_jacobian_evaluations() const { return
        num_jacobian_evaluations_;
  }

  /// Gets the number of factorizations of the iteration matrix since the last
  /// call to ResetStatistics(). This count includes those refactorizations
  /// necessary during the error estimation process.
  int64_t get_num_iteration_matrix_factorizations() const {
    return num_iter_factorizations_;
  }

  /// @}

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
  void DoInitialize() override;
  void DoResetStatistics() override;
  void Factor(const MatrixX<T>& A);
  VectorX<T> Solve(const VectorX<T>& rhs) const;
  bool AttemptStepPaired(const T& dt, VectorX<T>* xtplus_euler,
                         VectorX<T>* xtplus_trap);
  bool StepAbstract(const T& dt,
                    const std::function<VectorX<T>()>& g,
                    int scale,
                    VectorX<T>* xtplus, int trial = 1);
  bool CalcMatrices(const T& tf, const T& dt, int scale,
                    const VectorX<T>& xtplus, int trial);
  MatrixX<T> CalcJacobian(const T& tf, const VectorX<T>& xtplus);
  bool DoStep(const T& dt) override;
  bool StepImplicitEuler(const T& dt);
  bool StepImplicitTrapezoid(const T& dt, const VectorX<T>& dx0,
                             VectorX<T>* xtplus);
  MatrixX<T> ComputeForwardDiffJacobian(const System<T>&,
                                        const Context<T>&,
                                        ContinuousState<T>* state);
  MatrixX<T> ComputeCentralDiffJacobian(const System<T>&,
                                        const Context<T>&,
                                        ContinuousState<T>* state);
  MatrixX<T> ComputeAutoDiffJacobian(const System<T>& system,
                                     const Context<T>& context);
  VectorX<T> CalcTimeDerivativesUsingContext();

  // This is a pre-allocated temporary for use by integration. It stores
  // the derivatives computed at x(t+h).
  std::unique_ptr<ContinuousState<T>> derivs_;

  // A simple LU factorization is all that is needed; robustness in the solve
  // comes naturally as dt << 1. Keeping this data in the class definition
  // serves to minimize heap allocations and deallocations.
  Eigen::PartialPivLU<MatrixX<double>> LU_;

  // A QR factorization is necessary for automatic differentiation (current
  // Eigen requirement).
  Eigen::HouseholderQR<MatrixX<AutoDiffXd>> QR_;

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

  // The last computed Jacobian matrix. Keeping this data in the class
  // definitions serves to minimize heap allocations and deallocations.
  MatrixX<T> J_;

  // The last computed *negation* of the "iteration matrix", equivalent to
  // J_ * (dt / scale) - 1, where scale is either 1.0 or 2.0, depending on
  // whether the implicit Euler or implicit trapezoid method was used. Keeping
  // this data in the class definition serves to minimize heap allocations
  // and deallocations.
  MatrixX<T> neg_iteration_matrix_;

  // Whether the last call to StepAbstract() was a failure.
  bool last_call_failed_{false};

  // If set to `false`, Jacobian matrices and iteration matrix factorizations
  // will not be reused.
  bool reuse_{true};

  // Various combined statistics.
  int64_t num_jacobian_evaluations_{0};
  int64_t num_iter_factorizations_{0};
  int64_t num_jacobian_function_evaluations_{0};
  int64_t num_nr_iterations_{0};

  // Implicit trapezoid specific statistics.
  int64_t num_err_est_jacobian_reforms_{0};
  int64_t num_err_est_iter_factorizations_{0};
  int64_t num_err_est_function_evaluations_{0};
  int64_t num_err_est_jacobian_function_evaluations_{0};
  int64_t num_err_est_nr_iterations_{0};
};
}  // namespace systems
}  // namespace drake
