#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, explicit Euler integrator. State is updated in the following
 * manner:
 * <pre>
 * x(t+h) = x(t) + dx/dt * h
 * </pre>
 */
template <class T>
class FixedStepImplicitEulerIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedStepImplicitEulerIntegrator)

  ~FixedStepImplicitEulerIntegrator() override = default;

  /**
   * Constructs a fixed-step integrator for a given system using the given
   * context for initial conditions.
   * @param system A reference to the system to be simulated
   * @param max_step_size The maximum (fixed) step size; the integrator will
   *                      not take larger step sizes than this.
   * @param context Pointer to the context (nullptr is ok, but the caller
   *                must set a non-null context before Initialize()-ing the
   *                integrator).
   * @sa Initialize()
   */
  FixedStepImplicitEulerIntegrator(const System<T>& system, const T& max_step_size,
                          Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
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

  /**
   * Explicit Euler integrator does not support error estimation.
   */
  bool supports_error_estimation() const override { return false; }

  /// Integrator does not provide an error estimate.
  int get_error_estimate_order() const override { return 0; }

 private:
  // IntegratorBase overrides:
  void DoResetStatistics() override;
  void DoInitialize() override;
  bool DoStep(const T& dt) override;

  // Helpers to compute the Newton-Raphson iteration Jacobian.
  MatrixX<T> ComputeForwardDiffJacobian(const System<T>&,
                                        const Context<T>&,
                                        ContinuousState<T>* state);
  MatrixX<T> ComputeCentralDiffJacobian(const System<T>&,
                                        const Context<T>&,
                                        ContinuousState<T>* state);
  MatrixX<T> ComputeAutoDiffJacobian(const System<T>& system,
                                     const Context<T>& context);
  VectorX<T> CalcTimeDerivativesUsingContext();

  //
  MatrixX<T> CalcJacobian(const T& tf, const VectorX<T>& xtplus);

  bool StepAbstract(const T& dt,
                    const std::function<VectorX<T>()>& g,
                    int scale,
                    VectorX<T>* xtplus);

  bool StepImplicitEuler(const T& dt);

  // Helpers to factor and solve the Newton-Raphson iteration Jacobian.
  void Factor(const MatrixX<T>& A);
  VectorX<T> Solve(const VectorX<T>& rhs) const;

  // Returns true if any coefficients went inf or nan.
  bool IsBadJacobian(const MatrixX<T>& J) const;

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

  // Various combined statistics.
  int64_t num_jacobian_evaluations_{0};
  int64_t num_factorizations_{0};
  int64_t num_jacobian_function_evaluations_{0};
  int64_t num_nr_iterations_{0};
};

}  // namespace systems
}  // namespace drake

