#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * An abstract class providing methods shared by implicit integrators.
 * @tparam_nonsymbolic_scalar
 */
template <class T>
class ImplicitIntegrator : public IntegratorBase<T> {
 public:
  virtual ~ImplicitIntegrator() {}

  explicit ImplicitIntegrator(const System<T>& system,
                                   Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {}

  /** The maximum number of Newton-Raphson iterations to take before the
   Newton-Raphson process decides that convergence will not be attained. This
   number affects the speed with which a solution is found. If the number is
   too small, Jacobian/iteration matrix reformations and refactorizations will
   be performed unnecessarily. If the number is too large, the Newton-Raphson
   process will waste time evaluating derivatives when convergence is
   infeasible. [Hairer, 1996] states, "It is our experience that the code
   becomes more efficient when we allow a relatively high number of iterations
   (e.g., [7 or 10])", p. 121.  Note that the focus of that quote is a 5th order
   integrator that uses a quasi-Newton approach.
   */
  int max_newton_raphson_iterations() const {
    return do_max_newton_raphson_iterations();
  }

  enum class JacobianComputationScheme {
    /// Forward differencing.
    kForwardDifference,

    /// Central differencing.
    kCentralDifference,

    /// Automatic differentiation.
    kAutomatic
  };

  /// @name Methods for getting and setting the Jacobian scheme.
  ///
  /// Methods for getting and setting the scheme used to determine the
  /// Jacobian matrix necessary for solving the requisite nonlinear system
  /// if equations.
  ///
  /// Selecting the wrong Jacobian determination scheme will slow (possibly
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
  /// @{

  /// Sets whether the integrator attempts to reuse Jacobian matrices and
  /// iteration matrix factorizations (default is `true`). Forming Jacobian
  /// matrices and factorizing iteration matrices are generally the two most
  /// expensive operations performed by this integrator. For small systems
  /// (those with on the order of ten state variables), the additional accuracy
  /// that using fresh Jacobians and factorizations buys- which can permit
  /// increased step sizes but should have no effect on solution accuracy- can
  /// outweigh the small factorization cost.
  /// @note The reuse setting will have no effect when
  ///       get_use_full_newton() `== true`.
  /// @see get_reuse()
  /// @see set_use_full_newton()
  void set_reuse(bool reuse) { reuse_ = reuse; }

  /// Gets whether the integrator attempts to reuse Jacobian matrices and
  /// iteration matrix factorizations.
  /// @see set_reuse()
  /// @note This method always returns `false` when full-Newton mode is on.
  bool get_reuse() const { return !use_full_newton_ && reuse_; }

  /// Sets whether the method operates in "full Newton" mode, in which case
  /// Jacobian and iteration matrices are freshly computed on every
  /// Newton-Raphson iteration. When set to `true`, this mode overrides
  /// the reuse mode.
  /// @see set_reuse()
  void set_use_full_newton(bool flag) { use_full_newton_ = flag; }

  /// Gets whether this method is operating in "full Newton" mode.
  /// @see set_use_full_newton()
  bool get_use_full_newton() const { return use_full_newton_; }

  /// Sets the Jacobian computation scheme. This function can be safely called
  /// at any time (i.e., the integrator need not be re-initialized afterward).
  /// @note Discards any already-computed Jacobian matrices if the scheme
  ///       changes.
  void set_jacobian_computation_scheme(JacobianComputationScheme scheme) {
    if (jacobian_scheme_ != scheme) {
      J_.resize(0, 0);
      // Reset the Jacobian and any matrices cached by child integrators.
      DoResetCachedJacobianRelatedMatrices();
    }
    jacobian_scheme_ = scheme;
  }

  JacobianComputationScheme get_jacobian_computation_scheme() const {
    return jacobian_scheme_;
  }
  /// @}

  /// @name Cumulative statistics functions.
  /// The functions return statistics specific to the implicit integration
  /// process.
  /// @{

  /// Gets the number of ODE function evaluations
  /// (calls to EvalTimeDerivatives()) *used only for computing
  /// the Jacobian matrices* since the last call to ResetStatistics(). This
  /// count includes those derivative calculations necessary for computing
  /// Jacobian matrices during error estimation processes.
  int64_t get_num_derivative_evaluations_for_jacobian() const {
    return num_jacobian_function_evaluations_;
  }

  /// Gets the number of Jacobian computations (i.e., the number of times
  /// that the Jacobian matrix was reformed) since the last call to
  /// ResetStatistics(). This count includes those evaluations necessary
  /// during error estimation processes.
  int64_t get_num_jacobian_evaluations() const { return
        num_jacobian_evaluations_;
  }

  /// Gets the number of iterations used in the Newton-Raphson nonlinear systems
  /// of equation solving process since the last call to ResetStatistics(). This
  /// count includes those Newton-Raphson iterations used during error
  /// estimation processes.
  int64_t get_num_newton_raphson_iterations() const {
    return do_get_num_newton_raphson_iterations();
  }

  /// Gets the number of factorizations of the iteration matrix since the last
  /// call to ResetStatistics(). This count includes those refactorizations
  /// necessary during error estimation processes.
  int64_t get_num_iteration_matrix_factorizations() const {
    return num_iter_factorizations_;
  }

  /// Gets the number of ODE function evaluations
  /// (calls to EvalTimeDerivatives()) *used only for the error estimation
  /// process* since the last call to ResetStatistics(). This count
  /// includes those needed to compute Jacobian matrices.
  int64_t get_num_error_estimator_derivative_evaluations() const {
    return do_get_num_error_estimator_derivative_evaluations();
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
    return do_get_num_error_estimator_derivative_evaluations_for_jacobian();
  }

  /// Gets the number of iterations *used in the Newton-Raphson nonlinear
  /// systems of equation solving process for the error estimation process*
  /// since the last call to ResetStatistics().
  int64_t get_num_error_estimator_newton_raphson_iterations() const { return
        do_get_num_error_estimator_newton_raphson_iterations();
  }

  /// Gets the number of Jacobian matrix computations *used only during
  /// the error estimation process* since the last call to ResetStatistics().
  int64_t get_num_error_estimator_jacobian_evaluations() const {
    return do_get_num_error_estimator_jacobian_evaluations();
  }

  /// Gets the number of factorizations of the iteration matrix *used only
  /// during the error estimation process* since the last call to
  /// ResetStatistics().
  int64_t get_num_error_estimator_iteration_matrix_factorizations() const {
    return do_get_num_error_estimator_iteration_matrix_factorizations();
  }
  /// @}

 protected:
  /// Derived classes can override this method to change the number of
  /// Newton-Raphson iterations (10 by default) to take before the
  /// Newton-Raphson process decides that convergence will not be attained.
  virtual int do_max_newton_raphson_iterations() const { return 10; }

  /// A class for storing the factorization of an iteration matrix and using it
  /// to solve linear systems of equations. This class exists simply because
  /// Eigen AutoDiff puts limitations on what kinds of factorizations can be
  /// used; encapsulating the iteration matrix factorizations like this frees
  /// the implementer of these kinds of details.
  class IterationMatrix {
   public:
    /// Factors a dense matrix (the iteration matrix) using LU factorization,
    /// which should be faster than the QR factorization used in the specialized
    /// template method for AutoDiffXd below.
    void SetAndFactorIterationMatrix(const MatrixX<T>& iteration_matrix);

    /// Solves a linear system Ax = b for x using the iteration matrix (A)
    /// factored using LU decomposition.
    /// @see Factor()
    VectorX<T> Solve(const VectorX<T>& b) const;

    /// Returns whether the iteration matrix has been set and factored.
    bool matrix_factored() const { return matrix_factored_; }

   private:
    bool matrix_factored_{false};

    // A simple LU factorization is all that is needed for ImplicitIntegrator
    // templated on scalar type `double`; robustness in the solve
    // comes naturally as h << 1. Keeping this data in the class definition
    // serves to minimize heap allocations and deallocations.
    Eigen::PartialPivLU<MatrixX<double>> LU_;

    // The only factorization supported by automatic differentiation in Eigen is
    // currently QR. When ImplicitIntegrator is templated on type AutoDiffXd,
    // this will be the factorization that is used.
    Eigen::HouseholderQR<MatrixX<AutoDiffXd>> QR_;
  };

  /// Computes necessary matrices (Jacobian and iteration matrix) for
  /// Newton-Raphson (NR) iterations, as necessary. This method has been
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
  /// @param h the integration step size (for computing iteration matrices).
  /// @param trial which trial (1-4) the Newton-Raphson process is in when
  ///        calling this method.
  /// @param compute_and_factor_iteration_matrix a function pointer for
  ///        computing and factoring the iteration matrix.
  /// @param[out] iteration_matrix the updated and factored iteration matrix on
  ///             return.
  /// @returns `false` if the calling stepping method should indicate failure;
  ///          `true` otherwise.
  /// @pre 1 <= `trial` <= 4.
  /// @post the state in the internal context may or may not be altered on
  ///       return; if altered, it will be set to (t, xt).
  bool MaybeFreshenMatrices(const T& t, const VectorX<T>& xt, const T& h,
      int trial,
      const std::function<void(const MatrixX<T>& J, const T& h,
          typename ImplicitIntegrator<T>::IterationMatrix*)>&
      compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  /// Computes necessary matrices (Jacobian and iteration matrix) for full
  /// Newton-Raphson (NR) iterations, if full Newton-Raphson method is activated
  /// (if it's not activated, this method is a no-op).
  /// @param t the time at which to compute the Jacobian.
  /// @param xt the continuous state at which the Jacobian is computed.
  /// @param h the integration step size (for computing iteration matrices).
  /// @param compute_and_factor_iteration_matrix a function pointer for
  ///        computing and factoring the iteration matrix.
  /// @param[out] iteration_matrix the updated and factored iteration matrix on
  ///             return.
  /// @post the state in the internal context will be set to (t, xt) and this
  ///       will store the updated Jacobian matrix, on return.
  void FreshenMatricesIfFullNewton(const T& t, const VectorX<T>& xt, const T& h,
      const std::function<void(const MatrixX<T>& J, const T& h,
          typename ImplicitIntegrator<T>::IterationMatrix*)>&
      compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  /// Checks whether a proposed update is effectively zero, indicating that the
  /// Newton-Raphson process converged.
  /// @param xc the continuous state.
  /// @param dxc the update to the continuous state.
  /// @param eps the tolerance that will be used to determine whether the
  ///        change in any dimension of the state is nonzero. `eps` will
  ///        be treated as an absolute tolerance when the magnitude of a
  ///        particular dimension of the state is no greater than unity and as
  ///        a relative tolerance otherwise. For non-positive `eps` (default),
  ///        an appropriate tolerance will be computed.
  /// @return `true` if the update is effectively zero.
  bool IsUpdateZero(
      const VectorX<T>& xc, const VectorX<T>& dxc, double eps = -1.0) const {
    using std::abs;
    using std::max;

    // Reset the tolerance, if necessary, by backing off slightly from the
    // tightest tolerance (machine epsilon).
    if (eps <= 0)
      eps = 10 * std::numeric_limits<double>::epsilon();

    for (int i = 0; i < xc.size(); ++i) {
      // We do not want the presence of a NaN to cause this function to
      // spuriously return `true`, so indicate the update is not zero when a NaN
      // is detected. This will make the Newton-Raphson process in the caller
      // continue iterating until its inevitable failure.
      using std::isnan;
      if (isnan(dxc[i]) || isnan(xc[i])) return false;

      const T tol = max(abs(xc[i]), T(1)) * eps;
      if (abs(dxc[i]) > tol)
        return false;
    }

    return true;
  }

  enum class ConvergenceStatus {
    kDiverged,
    kConverged,
    kNotConverged,
  };

  /// Checks a Newton-Raphson iteration process for convergence. The logic
  /// is based on the description on p. 121 from
  /// [Hairer, 1996] E. Hairer and G. Wanner. Solving Ordinary Differential
  ///                Equations II (Stiff and Differential-Algebraic Problems).
  ///                Springer, 1996.
  /// This function is called after the dx is computed in an iteration, to
  /// determine if the Newton process converged, diverged, or needs further
  /// iterations.
  /// @param iteration the iteration index, starting at 0 for the first
  ///           iteration.
  /// @param xtplus the state x at the current iteration.
  /// @param dx the state change dx the difference between xtplus at the
  ///           current and the previous iteration.
  /// @param dx_norm the weighted norm of dx
  /// @param last_dx_norm the weighted norm of dx from the previous iteration.
  ///           This parameter is ignored during the first iteration.
  /// @return `kConverged` for convergence, `kDiverged` for divergence,
  ///         otherwise `kNotConverged` if Newton-Raphson should simply
  ///         continue.
  ConvergenceStatus CheckNewtonConvergence(int iteration,
      const VectorX<T>& xtplus, const VectorX<T>& dx, const T& dx_norm,
      const T& last_dx_norm) const;

  /// Resets any statistics particular to a specific implicit integrator. The
  /// default implementation of this function does nothing. If your integrator
  /// collects its own statistics, you should re-implement this method and
  /// reset them there.
  virtual void DoResetImplicitIntegratorStatistics() {}


  /// @copydoc IntegratorBase::DoReset()
  virtual void DoImplicitIntegratorReset() {}

  /// Resets any cached Jacobian or iteration matrices owned by child classes.
  /// This is called when the user changes the Jacobian computation scheme;
  /// the child class should use this to reset its cached matrices.
  virtual void DoResetCachedJacobianRelatedMatrices() {}

  /// Checks to see whether a Jacobian matrix is "bad" (has any NaN or
  /// Inf values) and needs to be recomputed. A divergent Newton-Raphson
  /// iteration can cause the state to overflow, which is how the Jacobian can
  /// become "bad". This is an O(n²) operation, where n is the state dimension.
  bool IsBadJacobian(const MatrixX<T>& J) const;

  // TODO(edrumwri) Document the functions below.
  virtual int64_t do_get_num_newton_raphson_iterations() const = 0;
  virtual int64_t do_get_num_error_estimator_derivative_evaluations() const = 0;
  virtual int64_t
      do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const = 0;
  virtual int64_t do_get_num_error_estimator_newton_raphson_iterations()
      const = 0;
  virtual int64_t do_get_num_error_estimator_jacobian_evaluations() const = 0;
  virtual int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const = 0;
  MatrixX<T>& get_mutable_jacobian() { return J_; }
  void DoResetStatistics() override;
  void DoReset() final;

  // Compute the partial derivative of the ordinary differential equations with
  // respect to the state variables for a given x(t).
  // @param t the time around which to compute the Jacobian matrix.
  // @param x the continuous state around which to compute the Jacobian matrix.
  // @post the context's time and continuous state will be temporarily set
  //       during this call (and then reset to their original values) on return.
  //       Furthermore, the jacobian_is_fresh_ flag is set to "true", indicating
  //       that the Jacobian was computed from the most recent time t.
  const MatrixX<T>& CalcJacobian(const T& t, const VectorX<T>& x);

  // Computes the Jacobian of the ordinary differential equations around time
  // and continuous state `(t, xt)` using a first-order forward difference
  // (i.e., numerical differentiation).
  // @param system The dynamical system.
  // @param t the time around which to compute the Jacobian matrix.
  // @param xt the continuous state around which to compute the Jacobian matrix.
  // @param context the Context of the system, at time and continuous state
  //        unknown.
  // @param[out] J the Jacobian matrix around time and state `(t, xt)`.
  // @post The continuous state will be indeterminate on return.
  void ComputeForwardDiffJacobian(const System<T>& system, const T& t,
      const VectorX<T>& xt, Context<T>* context, MatrixX<T>* J);

  // Computes the Jacobian of the ordinary differential equations around time
  // and continuous state `(t, xt)` using a second-order central difference
  // (i.e., numerical differentiation).
  // @param system The dynamical system.
  // @param t the time around which to compute the Jacobian matrix.
  // @param xt the continuous state around which to compute the Jacobian matrix.
  // @param context the Context of the system, at time and continuous state
  //        unknown.
  // @param[out] J the Jacobian matrix around time and state `(t, xt)`.
  // @post The continuous state will be indeterminate on return.
  void ComputeCentralDiffJacobian(const System<T>& system, const T& t,
      const VectorX<T>& xt, Context<T>* context, MatrixX<T>* J);

  // Computes the Jacobian of the ordinary differential equations around time
  // and continuous state `(t, xt)` using automatic differentiation.
  // @param system The dynamical system.
  // @param t the time around which to compute the Jacobian matrix.
  // @param xt the continuous state around which to compute the Jacobian matrix.
  // @param context the Context of the system, at time and continuous state
  //        unknown.
  // @param[out] J the Jacobian matrix around time and state `(t, xt)`.
  // @post The continuous state will be indeterminate on return.
  void ComputeAutoDiffJacobian(const System<T>& system, const T& t,
      const VectorX<T>& xt, const Context<T>& context, MatrixX<T>* J);

  /// @copydoc IntegratorBase::DoStep()
  virtual bool DoImplicitIntegratorStep(const T& h) = 0;

  // Methods for derived classes to increment the factorization and Jacobian
  // evaluation counts.
  void increment_num_iter_factorizations() {
    ++num_iter_factorizations_;
  }

  void increment_jacobian_computation_derivative_evaluations(int count) {
    num_jacobian_function_evaluations_ += count;
  }

  void increment_jacobian_evaluations() {
    ++num_jacobian_evaluations_;
  }

  void set_jacobian_is_fresh(bool flag) {
    jacobian_is_fresh_ = flag;
  }

 private:
  bool DoStep(const T& h) final {
    bool result = DoImplicitIntegratorStep(h);
    // If the implicit step is successful (result is true), we need a new
    // Jacobian (fresh is false). Otherwise, a failed step (result is false)
    // means we can keep the Jacobian (fresh is true). Therefore fresh =
    // !result, almost always.

    // The exception is when the implicit step fails during the second half-
    // step of ImplicitEulerIntegrator, in which case the Jacobian is not from
    // the beginning of the step, and so fresh should be false. We leave it
    // untouched here to keep the design of ImplicitIntegrator<T> simple, and
    // let ImplicitEulerIntegrator<T> handle this flag on its own at the
    // beginning of ImplicitEulerIntegrator<T>::DoImplicitIntegratorStep().
    jacobian_is_fresh_ = !result;

    return result;
  }

  // The scheme to be used for computing the Jacobian matrix during the
  // nonlinear system solve process.
  JacobianComputationScheme jacobian_scheme_{
      JacobianComputationScheme::kForwardDifference};

  // The last computed Jacobian matrix.
  MatrixX<T> J_;

  // Indicates whether the Jacobian matrix is fresh. We say the Jacobian is
  // "fresh" if it was last computed at a state (t0, x0) from the beginning of
  // the current step. This indicates to MaybeFreshenMatrices that it should
  // not recompute the Jacobian, but rather it should fail immediately. This
  // is only used when use_full_newton_ and reuse_ are set to false.
  bool jacobian_is_fresh_{false};

  // If set to `false`, Jacobian matrices and iteration matrix factorizations
  // will not be reused.
  bool reuse_{true};

  // If set to `true`, Jacobian matrices and iteration matrix factorizations
  // will be freshly computed on every Newton-Raphson iteration. This should
  // only ever be useful in debugging.
  bool use_full_newton_{false};

  // Various combined statistics.
  int64_t num_iter_factorizations_{0};
  int64_t num_jacobian_evaluations_{0};
  int64_t num_jacobian_function_evaluations_{0};
};

// We do not support computing the Jacobian matrix using automatic
// differentiation when the scalar is already an AutoDiff type.
// Note: must be declared inline because it's specialized and located in the
// header file (to avoid multiple definition errors).
template <>
inline void ImplicitIntegrator<AutoDiffXd>::
    ComputeAutoDiffJacobian(const System<AutoDiffXd>&,
      const AutoDiffXd&, const VectorX<AutoDiffXd>&,
      const Context<AutoDiffXd>&, MatrixX<AutoDiffXd>*) {
        throw std::runtime_error("AutoDiff'd Jacobian not supported from "
                                     "AutoDiff'd ImplicitIntegrator");
}

// Factors a dense matrix (the iteration matrix). This
// AutoDiff-specialized method is necessary because Eigen's LU factorization,
// which should be faster than the QR factorization used here, is not currently
// AutoDiff-able (while the QR factorization *is* AutoDiff-able).
// Note: must be declared inline because it's specialized and located in the
// header file (to avoid multiple definition errors).
template <>
inline void ImplicitIntegrator<AutoDiffXd>::IterationMatrix::
    SetAndFactorIterationMatrix(const MatrixX<AutoDiffXd>& iteration_matrix) {
  QR_.compute(iteration_matrix);
  matrix_factored_ = true;
}

// Solves the linear system Ax = b for x using the iteration matrix (A)
// factored using QR decomposition.
// @see Factor()
// Note: must be declared inline because it's specialized and located in the
// header file (to avoid multiple definition errors).
template <>
inline VectorX<AutoDiffXd>
ImplicitIntegrator<AutoDiffXd>::IterationMatrix::Solve(
    const VectorX<AutoDiffXd>& b) const {
  return QR_.solve(b);
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ImplicitIntegrator)
