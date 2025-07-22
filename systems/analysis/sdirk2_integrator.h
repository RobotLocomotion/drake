#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/implicit_integrator.h"

namespace drake {
namespace systems {

/**
 A two-stage, second-order, stiffly accurate, L-stable, Singly Diagonally
 Implicit Runge Kutta (SDIRK) integrator with embedded error estimation.

 The Butcher tableau for this integrator is given by
 <pre>
  γ  | γ
  1  | (1-γ)        γ
 -----------------------------------------------------------------------------
       (1-γ)        γ
       (1-α)        α
 </pre>
 where γ = 1 + √2/2 and α = 2 - 5√2 / 4. The final row provides an embedded
 first-order accurate solution for error control.

 This method is described in [Kennedy, 2016], with error estimation coefficients
 from Table 4 of [Blom, 2016].

 - [Kennedy, 2016] C. Kennedy and M. Carpenter. "Diagonally implicit Runge-Kutta
   methods for ordinary differential equations. A review." Sec. 4.1.2, 2016
 - [Blom, 2016] D. Blom et al. "A comparison of Rosenbrock and ESDIRK methods
   combined with iterative solvers for unsteady compressible flows." Advances in
   Computational Mathematics 42 (2016): 1401-1426.

 @tparam_nonsymbolic_scalar
 @ingroup integrators
 */
template <class T>
class Sdirk2Integrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Sdirk2Integrator);

  ~Sdirk2Integrator() override;

  explicit Sdirk2Integrator(const System<T>& system,
                            Context<T>* context = nullptr)
      : ImplicitIntegrator<T>(system, context) {}

  bool supports_error_estimation() const final { return true; }

  int get_error_estimate_order() const final { return 2; }

 private:
  // Implicit integrator statistics
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  // The embedded error estimate means that no extra computations are devoted to
  // the error estimate.
  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_newton_raphson_iterations() const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return 0;
  }

  // Implicit integrator virtual function implementations
  void DoInitialize() final;

  std::unique_ptr<ImplicitIntegrator<T>> DoImplicitIntegratorClone()
      const final;

  void DoResetCachedJacobianRelatedMatrices() final;

  void DoResetImplicitIntegratorStatistics() final;

  // The main step function.
  // @returns `true` if successful; on `true`, the time and continuous state
  //          will be advanced in the context (e.g., from t0 to t0 + h). On a
  //          `false` return, the time and continuous state in the context will
  //          be restored to its original value (at t0).
  bool DoImplicitIntegratorStep(const T& h) final;

  // Use Newton-Raphson to solve k = f(t, x₀ + γ h k).
  //
  // This function will recursively call itself with escalating `trial` values,
  // with higher values corresponding to more computationally expensive (but
  // more likely to succeed) methods.
  //
  // Returns true if the Newton-Raphson process converged. Returning false
  // indicates convergence failure and will trigger a reduction in h.
  bool NewtonSolve(const T& t, const T& h, const VectorX<T>& x0, VectorX<T>* k,
                   int trial = 1);

  // Compute and factor the iteration matrix A = [I - γhJ] for the Newton steps.
  // Note that the the "S" in SDIRK means that the iteration matrix has the same
  // structure across all stages.
  static void ComputeAndFactorIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  // The iteration matrix A = [I - γhJ] for the Newton steps.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_;

  // Intermediate variables to avoid heap allocations
  VectorX<T> x0_, x1_, x_, k1_, k2_, g_, dk_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Storage for the error estimate x - x̂
  VectorX<T> err_est_vec_;

  // Tracks the number of Newton-Raphson iterations
  int64_t num_nr_iterations_{0};

  // Constants defined in the Butcher tableau
  static constexpr double gamma_ = 0.29289321881345247559915563789515096;
  static constexpr double alpha_ = 0.23223304703363118899788909473787740;
};

}  // namespace systems
}  // namespace drake
