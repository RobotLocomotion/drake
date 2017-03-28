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
 * become unstable (growing without bound). The practical effect of L-Stability
 * is that the integrator tends to be stable for any given step size on an
 * arbitrary system of ordinary differential equations. See [Lambert, 1991],
 * Ch. 6 for an approachable discussion on stiff differential equations and
 * L- and A-Stability.
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
 * This implementation uses a novel solution process compared to methods
 * described in existing literature on implicit integration techniques. Those
 * techniques use "vanilla" Newton-Raphson (NR), relying upon obvious
 * convergence to a solution for `g = 0` where
 * `g(x(t+h)) ≡ x(t+h) - x(t) - h f(t+h,x(t+h))` as `h` becomes sufficiently
 * small. Those methods require logic to determine whether to keep taking NR
 * iterates or whether to decrease `h` and begin iterating anew.  It should be
 *  clear that for sufficiently small `h`, `g(x(t+h)) = 0` will be satisfiable.
 *
 * This novel solution process uses globally convergent NR with line search,
 * which is guaranteed to converge to a "local minimum", a point at which the
 * norm of `∇g` will be equal to zero. In other words, given a proposed update
 * Δx to the state variables, this method computes a scalar value α such that
 * `||g(x(t+h) + αΔx)||` is significantly reduced over `||g(x(t+h)||`.  `g = 0`
 * is *not* guaranteed at the end of this solution process, which seems to put
 * this method at a significant disadvantage compared to existing methods.
 * However, those methods generally terminate before `g = 0`! The danger in
 * the novel approach is that the determined `||g(x(t+h)||` is inaccurate for
 * large `h`. Use of error control and initializing `g(x(t+h))` to `g(x(t))`
 * both guard against (but do not, at least as of yet, provably prevent) that
 * possibility.
 *
 * - [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
 *                    Equations. John Wiley & Sons, 1991.
 */
template <class T>
class ImplicitEulerIntegrator : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitEulerIntegrator)

  ~ImplicitEulerIntegrator() override = default;

  explicit ImplicitEulerIntegrator(const System<T>& system,
                                   const T& max_step_size,
                                   Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
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
  /// differentiation) yields the best Jacobian accuracy for a given unit of
  /// computational time: the total error in the forward-difference
  /// approximation is close to √ε, where ε is machine epsilon, from n forward
  /// dynamics calls (where n is the number of state variables). Central
  /// differencing yields the most accurate numerically differentiated Jacobian
  /// matrix: the total error in the central-difference approximation is close
  /// to ε^(2/3), from 2n forward dynamics calls. See
  /// [Nocedal 2004, pp. 167-169].
  ///
  /// [Nocedal 2004] J. Nocedal and S. Wright. Numerical Optimization. Springer,
  ///                2004.
  enum class JacobianComputationScheme {
    /// O(h) Forward differencing.
    kForwardDifference,

    /// O(h²) Central differencing.
    kCentralDifference,

    /// Automatic differentiation.
    kAutomatic
  };

  /// Sets the Jacobian computation scheme. The integrator need not be
  /// re-initialized after setting the scheme.
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

  /// Gets the number of ODE function evaluations *used only for computing
  /// the Jacobian matrices* since the last call to ResetStatistics().
  int get_num_jacobian_function_evaluations() const {
    return num_jacobian_function_evaluations_;
  }

  /// Gets the number of loops used in the Newton-Raphson nonlinear systems of
  /// equation solving process since the last call to ResetStatistics().
  int get_num_newton_raphson_loops() const { return num_nr_loops_; }

  /// Gets the mean scaling factor applied to each state update since the last
  /// call to ResetStatistics(). The ideal (and maximum) value is 1.0. Minimum
  /// value is 0.0.
  double get_mean_scaling_factor() const { return alpha_sum_ / num_nr_loops_; }
  /// @}

  /// @name Tunable parameters.
  /// Parameters for tuning the speed of the implicit integration process;
  /// the delta zero tolerance may affect the accuracy of the solution, while
  /// other parameters- those that determine the frequency with
  /// which the Jacobian matrix is reformulated and refactorized- affect only
  /// the speed that the solution will be found.
  /// @{

  /// Gets the tolerance below which changes to the state variables during the
  /// integration process should indicate that the Newton-Raphson process
  /// has converged.
  double get_delta_state_tolerance() const { return delta_update_tol_; }

  /// Sets the tolerance below which changes to the state variables during the
  /// integration process should indicate that the Newton-Raphson process
  /// has converged.
  void set_delta_state_tolerance(double tol) { delta_update_tol_ = tol; }

  /// Gets the exponent on the decrease in error in the objective function
  /// above which the Jacobian matrix will be reformulated and refactorized.
  double get_jacobian_reformulation_exponent() const {
    return reformulation_exponent_; }

  /// Sets the exponent on the decrease in error in the objective function
  /// above which the Jacobian matrix will be reformulated and refactorized.
  /// The Jacobian matrix is "freshened" if `f(x+α dx)`, the error in the
  /// objective function- which is zero if the objective function is perfectly
  /// satisfied- is reduced _less than:<pre>
  /// f(x)ᵝ  for f(x) > 1
  /// f(x)⁻ᵝ for f(x) ≤ 1</pre>
  /// The closer this value is to one, the less aggressively the Jacobian
  /// matrix will be freshened. The Newton-Raphson procedure is expected to
  /// achieve quadratic convergence, which would indicate a setting of β = 2.
  /// Recommended values lie in the range (1, 2].
  /// @throws std::logic_error if beta is less than 1.0.
  /// @param beta the reformulation exponent (default = 1.5). Set this to a
  ///             very large value to reformulate the Jacobian matrix on every
  ///             Newton-Raphson iteration.
  void set_jacobian_reformulation_exponent(double beta) {
    if (beta <= 1.0)
      throw std::logic_error("Jacobian reformulation exponent too small.");
    reformulation_exponent_ = beta;
  }

  /// @}

 protected:
  void DoInitialize() override;
  std::pair<bool, T> DoStepOnceAtMost(const T& max_dt) override;
  void DoResetStatistics() override;

 private:
  void StepAbstract(const T& dt,
                    const std::function<VectorX<T>(const VectorX<T>&)>& g,
                    double scale,
                    VectorX<T>* xtplus);
  void CalcIterationMatrix(const VectorX<T>& xtplus, double scale);
  MatrixX<T> CalcJacobian(const VectorX<T>& xtplus);
  void DoStepOnceFixedSize(const T& dt) override;
  void StepImplicitEuler(const T& dt);
  void StepImplicitTrapezoid(const T& dt, const VectorX<T>& dx0,
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

  // The exponent for recomputing and refactorizing the Jacobian matrix.
  double reformulation_exponent_{1.5};

  // This is a pre-allocated temporary for use by integration. It stores
  // the derivatives computed at x(t+h).
  std::unique_ptr<ContinuousState<T>> derivs_;

  // For LU factorization with full pivoting; this yields a solution to a
  // system of linear equations that is somewhat robust.
  Eigen::FullPivLU<MatrixX<T>> LU_;

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

  // The computed iteration matrix.
  MatrixX<T> Jg_;

  // Various statistics
  int num_function_evaluations_{0};
  int num_jacobian_function_evaluations_{0};
  int num_nr_loops_{0};
  double alpha_sum_{0.0};
};
}  // namespace systems
}  // namespace drake
