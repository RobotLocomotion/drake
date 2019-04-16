#pragma once

#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A class providing methods shared by implicit integrators.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * @tparam T The scalar type. Must be a valid Eigen scalar.
 */
template <class T>
class ImplicitIntegrator : public IntegratorBase<T> {
 public:
  virtual ~ImplicitIntegrator() {}

  explicit ImplicitIntegrator(const System<T>& system,
                                   Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {}

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

  /// @name Cumulative statistics functions.
  /// The functions return statistics specific to the implicit integration
  /// process.
  /// @{

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
    return do_get_num_iteration_matrix_factorizations();
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
  /// A class for storing the factorization of an iteration matrix and using it
  /// to solve linear systems of equations. This class exists simply because
  /// Eigen AutoDiff puts limitations on what kinds of factorizations can be
  /// used; encapsulating the iteration matrix factorizations like this frees
  /// the implementer of these kinds of details.
  class IterationMatrix {
   public:
    void SetAndFactorIterationMatrix(const MatrixX<T>& iteration_matrix);
    VectorX<T> Solve(const VectorX<T>& b) const;

    /// Returns whether the iteration matrix has been set and factored.
    bool matrix_factored() const { return matrix_factored_; }

   private:
    bool matrix_factored_{false};

    // A simple LU factorization is all that is needed for ImplicitIntegrator
    // templated on scalar type `double`; robustness in the solve
    // comes naturally as dt << 1. Keeping this data in the class definition
    // serves to minimize heap allocations and deallocations.
    Eigen::PartialPivLU<MatrixX<double>> LU_;

    // The only factorization supported by automatic differentiation in Eigen is
    // currently QR. When ImplicitIntegrator is templated on type AutoDiffXd,
    // this will be the factorization that is used.
    Eigen::HouseholderQR<MatrixX<AutoDiffXd>> QR_;
  };

  /**
   * Resets any statistics particular to a specific implicit integrator. The
   * default implementation of this function does nothing. If your integrator
   * collects its own statistics, you should re-implement this method and
   * reset them there.
   */
  virtual void DoResetImplicitIntegratorStatistics() {}

  // TODO(edrumwri) Document the functions below.
  virtual int64_t do_get_num_newton_raphson_iterations() const = 0;
  virtual int64_t do_get_num_iteration_matrix_factorizations() const = 0;
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
  void set_last_call_succeeded(bool success) { last_call_succeeded_ = success; }
  bool last_call_succeeded() const { return last_call_succeeded_; }
  bool IsBadJacobian(const MatrixX<T>& J) const;
  void DoResetStatistics() override;
  const MatrixX<T>& CalcJacobian(const T& tf, const VectorX<T>& xtplus);
  void ComputeForwardDiffJacobian(const System<T>&, const T& t,
      const VectorX<T>& xc, Context<T>*, MatrixX<T>* J);
  void ComputeCentralDiffJacobian(const System<T>&, const T& t,
      const VectorX<T>& xc, Context<T>*, MatrixX<T>* J);
  void ComputeAutoDiffJacobian(const System<T>& system, const T& t,
      const VectorX<T>& xc, const Context<T>& context, MatrixX<T>* J);
  VectorX<T> EvalTimeDerivativesUsingContext();

 private:
  // The scheme to be used for computing the Jacobian matrix during the
  // nonlinear system solve process.
  JacobianComputationScheme jacobian_scheme_{
      JacobianComputationScheme::kForwardDifference};

  // The last computed Jacobian matrix.
  MatrixX<T> J_;

  // Whether the last stepping call was successful.
  bool last_call_succeeded_{true};

  // If set to `false`, Jacobian matrices and iteration matrix factorizations
  // will not be reused.
  bool reuse_{true};

  // Various combined statistics.
  int64_t num_jacobian_evaluations_{0};
  int64_t num_jacobian_function_evaluations_{0};
};

template <class T>
void ImplicitIntegrator<T>::DoResetStatistics() {
  num_jacobian_function_evaluations_ = 0;
  num_jacobian_evaluations_ = 0;
}

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

// Computes the Jacobian of the ordinary differential equations around time
// and continuous state `(t, xt)` using automatic differentiation.
// @param system The dynamical system.
// @param t the time around which to compute the Jacobian matrix.
// @param xt the continuous state around which to compute the Jacobian matrix.
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix around time and state `(t, xt)`.
// @note The continuous state will be indeterminate on return.
template <class T>
void ImplicitIntegrator<T>::ComputeAutoDiffJacobian(
    const System<T>& system, const T& t, const VectorX<T>& xt,
    const Context<T>& context, MatrixX<T>* J) {
  SPDLOG_DEBUG(drake::log(), "  ImplicitIntegrator Compute Autodiff Jacobian "
               "t={}", t);
  // Create AutoDiff versions of the state vector.
  VectorX<AutoDiffXd> a_xt = xt;

  // Set the size of the derivatives and prepare for Jacobian calculation.
  const int n_state_dim = a_xt.size();
  for (int i = 0; i < n_state_dim; ++i)
    a_xt[i].derivatives() = VectorX<T>::Unit(n_state_dim, i);

  // Get the system and the context in AutoDiffable format. Inputs must also
  // be copied to the context used by the AutoDiff'd system (which is
  // accomplished using FixInputPortsFrom()).
  // TODO(edrumwri): Investigate means for moving as many of the operations
  //                 below offline (or with lower frequency than once-per-
  //                 Jacobian calculation) as is possible. These operations
  //                 are likely to be expensive.
  const auto adiff_system = system.ToAutoDiffXd();
  std::unique_ptr<Context<AutoDiffXd>> adiff_context = adiff_system->
      AllocateContext();
  adiff_context->SetTimeStateAndParametersFrom(context);
  adiff_system->FixInputPortsFrom(system, context, adiff_context.get());
  adiff_context->SetTime(t);

  // Set the continuous state in the context.
  adiff_context->SetContinuousState(a_xt);

  // Evaluate the derivatives at that state.
  const VectorX<AutoDiffXd> result =
      this->EvalTimeDerivatives(*adiff_system, *adiff_context).CopyToVector();

  *J = math::autoDiffToGradientMatrix(result);
}

// Evaluates the ordinary differential equations at the time and state in
// the system's context (stored by the integrator).
template <class T>
VectorX<T> ImplicitIntegrator<T>::EvalTimeDerivativesUsingContext() {
    return this->EvalTimeDerivatives(this->get_context()).CopyToVector();
}

// Computes the Jacobian of the ordinary differential equations around time
// and continuous state `(t, xt)` using a first-order forward difference (i.e.,
// numerical differentiation).
// @param system The dynamical system.
// @param t the time around which to compute the Jacobian matrix.
// @param xt the continuous state around which to compute the Jacobian matrix.
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix around time and state `(t, xt)`.
// @note The continuous state will be indeterminate on return.
template <class T>
void ImplicitIntegrator<T>::ComputeForwardDiffJacobian(
    const System<T>&, const T& t, const VectorX<T>& xt, Context<T>* context,
    MatrixX<T>* J) {
  using std::abs;

  // Set epsilon to the square root of machine precision.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of continuous state variables xt.
  const int n = context->num_continuous_states();

  SPDLOG_DEBUG(drake::log(), "  ImplicitIntegrator Compute Forwarddiff "
               "{}-Jacobian t={}", n, t);
  SPDLOG_DEBUG(drake::log(), "  computing from state {}", xt.transpose());

  // Initialize the Jacobian.
  J->resize(n, n);

  // Evaluate f(t,xt).
  context->SetTimeAndContinuousState(t, xt);
  const VectorX<T> f = EvalTimeDerivativesUsingContext();

  // Compute the Jacobian.
  VectorX<T> xt_prime = xt;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xt| is large, the increment will
    // be large as well. If |xt| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xt(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xt[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xt[i]| not small; make increment a fraction of |xt[i]|.
      dxi = eps * abs_xi;
    }

    // Update xt', minimizing the effect of roundoff error by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xt_prime(i) = xt(i) + dxi;
    dxi = xt_prime(i) - xt(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f' and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, xt_prime);
    J->col(i) = (EvalTimeDerivativesUsingContext() - f) / dxi;

    // Reset xt' to xt.
    xt_prime(i) = xt(i);
  }
}

// Computes the Jacobian of the ordinary differential equations around time
// and continuous state `(t, xt)` using a second-order central difference (i.e.,
// numerical differentiation).
// @param system The dynamical system.
// @param t the time around which to compute the Jacobian matrix.
// @param xt the continuous state around which to compute the Jacobian matrix.
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix around time and state `(t, xt)`.
// @note The continuous state will be indeterminate on return.
template <class T>
void ImplicitIntegrator<T>::ComputeCentralDiffJacobian(
    const System<T>&, const T& t, const VectorX<T>& xt, Context<T>* context,
    MatrixX<T>* J) {
  using std::abs;

  // Cube root of machine precision (indicated by theory) seems a bit coarse.
  // Pick power of eps halfway between 6/12 (i.e., 1/2) and 4/12 (i.e., 1/3).
  const double eps = std::pow(std::numeric_limits<double>::epsilon(), 5.0/12);

  // Get the number of continuous state variables xt.
  const int n = context->num_continuous_states();

  SPDLOG_DEBUG(drake::log(), "  ImplicitIntegrator Compute ",
               "Centraldiff {}-Jacobian t={}", n, t);

  // Initialize the Jacobian.
  J->resize(n, n);

  // Evaluate f(t,xt).
  context->SetTimeAndContinuousState(t, xt);
  const VectorX<T> f = EvalTimeDerivativesUsingContext();

  // Compute the Jacobian.
  VectorX<T> xt_prime = xt;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xt| is large, the increment will
    // be large as well. If |xt| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xt(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xt[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xt[i]| not small; make increment a fraction of |xt[i]|.
      dxi = eps * abs_xi;
    }

    // Update xt', minimizing the effect of roundoff error, by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xt_prime(i) = xt(i) + dxi;
    const T dxi_plus = xt_prime(i) - xt(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f(x+dx).
    context->SetContinuousState(xt_prime);
    VectorX<T> fprime_plus = EvalTimeDerivativesUsingContext();

    // Update xt' again, minimizing the effect of roundoff error.
    xt_prime(i) = xt(i) - dxi;
    const T dxi_minus = xt(i) - xt_prime(i);

    // Compute f(x-dx).
    context->SetContinuousState(xt_prime);
    VectorX<T> fprime_minus = EvalTimeDerivativesUsingContext();

    // Set the Jacobian column.
    J->col(i) = (fprime_plus - fprime_minus) / (dxi_plus + dxi_minus);

    // Reset xt' to xt.
    xt_prime(i) = xt(i);
  }
}

// Factors a dense matrix (the iteration matrix) using LU factorization,
// which should be faster than the QR factorization used in the specialized
// template method immediately below.
template <class T>
void ImplicitIntegrator<T>::IterationMatrix::SetAndFactorIterationMatrix(
    const MatrixX<T>& iteration_matrix) {
  LU_.compute(iteration_matrix);
  matrix_factored_ = true;
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

// Solves a linear system Ax = b for x using the iteration matrix (A)
// factored using LU decomposition.
// @sa Factor()
template <class T>
VectorX<T> ImplicitIntegrator<T>::IterationMatrix::Solve(
    const VectorX<T>& b) const {
  return LU_.solve(b);
}

// Solves the linear system Ax = b for x using the iteration matrix (A)
// factored using QR decomposition.
// @sa Factor()
// Note: must be declared inline because it's specialized and located in the
// header file (to avoid multiple definition errors).
template <>
inline VectorX<AutoDiffXd>
ImplicitIntegrator<AutoDiffXd>::IterationMatrix::Solve(
    const VectorX<AutoDiffXd>& b) const {
  return QR_.solve(b);
}

// Checks to see whether a Jacobian matrix has "become bad" and needs to be
// recomputed.
template <class T>
bool ImplicitIntegrator<T>::IsBadJacobian(const MatrixX<T>& J) const {
  return !J.allFinite();
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the state variables for a given x(t).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
const MatrixX<T>& ImplicitIntegrator<T>::CalcJacobian(const T& t,
    const VectorX<T>& x) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  const T t_current = context->get_time();
  const VectorX<T> x_current = context->get_continuous_state_vector().
      CopyToVector();

  // Update the time and state.
  context->SetTimeAndContinuousState(t, x);
  num_jacobian_evaluations_++;

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  // Get a the system.
  const System<T>& system = this->get_system();

  // TODO(edrumwri): Give the caller the option to provide their own Jacobian.
  [this, context, &system, &t, &x]() {
    switch (jacobian_scheme_) {
      case JacobianComputationScheme::kForwardDifference:
        ComputeForwardDiffJacobian(system, t, x, &*context, &J_);
        break;

      case JacobianComputationScheme::kCentralDifference:
        ComputeCentralDiffJacobian(system, t, x, &*context, &J_);
        break;

      case JacobianComputationScheme::kAutomatic:
        ComputeAutoDiffJacobian(system, t, x, *context, &J_);
        break;
    }
  }();

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  num_jacobian_function_evaluations_ += this->get_num_derivative_evaluations()
      - current_ODE_evals;

  // Reset the time and state.
  context->SetTimeAndContinuousState(t_current, x_current);

  return J_;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::ImplicitIntegrator)
