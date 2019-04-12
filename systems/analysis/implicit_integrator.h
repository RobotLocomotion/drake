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
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to https://drake.mit.edu/cxx_inl.html.
 *
 * @tparam T The scalar type. Must be a valid Eigen scalar.
 */
template <class T>
class ImplicitIntegrator : public IntegratorBase<T> {
 public:
  ~ImplicitIntegrator() override = default;

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
  /// (calls to CalcTimeDerivatives()) *used only for the error estimation
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

  // TODO(edrumwri) Document the functions below (or move documentation from
  // the -inl file to here).
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
}  // namespace systems
}  // namespace drake

// TODO(edrumwri) Get the DECLARE_CLASS_TEMPLATE_... macro working.
