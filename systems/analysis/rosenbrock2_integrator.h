#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/implicit_integrator.h"

namespace drake {
namespace systems {

/**
 * A two-stage Rosenbrock integrator with an embedded error estimate.
 *
 * This integrator is second-order and L-stable.
 *
 * TODO(vincekurtz): add documentation.
 */
template <class T>
class Rosenbrock2Integrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Rosenbrock2Integrator);

  ~Rosenbrock2Integrator() override;

  explicit Rosenbrock2Integrator(const System<T>& system,
                                 Context<T>* context = nullptr)
      : ImplicitIntegrator<T>(system, context) {}

  /**
   * Returns true, because this integrator supports error estimation.
   */
  bool supports_error_estimation() const final { return true; }

  /**
   * The error estimate order is 2, because the integrator uses a second-order
   * Rosenbrock method with an embedded first-order error estimate.
   */
  int get_error_estimate_order() const final { return 2; }

 private:
  // Integrator parameters adopted from MATLODE, and simplified where possible.
  static struct Rosenbrock2Parameters {
    static constexpr double gamma = 0.29289321881345254;  // 1 - 1/√2
    static constexpr double m1 = 3.0 / (2.0 * gamma);
    static constexpr double m2 = 1.0 / (2.0 * gamma);
    static constexpr double c = -2.0 / gamma;
    static constexpr double a = 1.0 / gamma;
    static constexpr double m_hat1 = 1.0 / gamma;
  } params_;

  // Compute and factor the iteration matrix G = [I/(h*γ) - J]. The same
  // iteration matrix is used in both stages.
  static void ComputeAndFactorIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  // Rosenbrock integrators take one NR iteration for each stage, at each step.
  int64_t do_get_num_newton_raphson_iterations() const final {
    return 2 * this->get_num_steps_taken();
  }

  // The embedded error estimate means that no additional derivative
  // evaluations, NR iterations, matrix factorizations, etc are needed for the
  // error estimate.
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

  void DoResetImplicitIntegratorStatistics() final {};

  void DoResetCachedJacobianRelatedMatrices() final {};

  void DoInitialize() final;

  std::unique_ptr<ImplicitIntegrator<T>> DoImplicitIntegratorClone()
      const final;

  // Takes a given step of the requested size, if possible.
  // @returns `true` if successful; on `true`, the time and continuous state
  //          will be advanced in the context (e.g., from t0 to t0 + h). On a
  //          `false` return, the time and continuous state in the context will
  //          be restored to its original value (at t0).
  bool DoImplicitIntegratorStep(const T& h) final;

  // For very small h, we'll fall back to an explicit Euler scheme.
  bool ExplicitEulerFallbackStep(const T& h);

  // The iteration matrix G = [I/(h*γ) - J] and its factorization.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_;

  // Vector used for error estimation
  VectorX<T> error_est_vec_;

  // Intermediate variables to avoid heap allocations
  VectorX<T> x0_, x_, xdot_, k1_, k2_;
};

}  // namespace systems
}  // namespace drake
