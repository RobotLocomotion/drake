#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

/**
 * A selectable order (third- or first-order), fully implicit integrator with
 * error estimation.
 *
 * @tparam_nonsymbolic_scalar
 * @tparam num_stages The number of stages used in this integrator, which must
 *         be either 1 or 2. Set this to 1 for the integrator to be implicit
 *         Euler and 2 for it to Radau3 (default).
 *
 * A two-stage Radau IIa (see [Hairer, 1996], Ch. 5) method is used for
 * propagating the state forward, by default. The state can also be propagated
 * using a single-stage method, in which case it is equivalent to an implicit
 * Euler method, by setting num_stages=1. Regardless of the order of propagating
 * state, the local (truncation) error is estimated through the implicit
 * trapezoid rule.
 *
 * Radau IIa methods are known to be L-Stable, meaning both that
 * applying it at a fixed integration step to the  "test" equation `y(t) = eᵏᵗ`
 * yields zero (for `k < 0` and `t → ∞`) *and* that it is also A-Stable.
 * A-Stability, in turn, means that the method can integrate the linear constant
 * coefficient system `dx/dt = Ax` at any step size without the solution
 * becoming unstable (growing without bound). The practical effect of
 * L-Stability is that the integrator tends to be stable for any given step size
 * on an arbitrary system of ordinary differential equations. Note that the
 * implicit trapezoid rule used for error estimation is "only" A-Stable; whether
 * this lesser stability has some practical effect on the efficiency of this
 * integrator is currently unknown. See [Lambert, 1991], Ch. 6 for an
 * approachable discussion on stiff differential equations and L- and
 * A-Stability.
 *
 * This implementation uses Newton-Raphson (NR). General implementation
 * details were taken from [Hairer, 1996] Ch. 8.
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996.
 * - [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
 *                    Equations. John Wiley & Sons, 1991.
 *
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
 * @see Radau3Integrator and Radau1Integrator alises for third- and first-order
 *      templates with num_stages already specified.
 * @note This integrator uses the integrator accuracy setting, even when run
 *       in fixed-step mode, to limit the error in the underlying Newton-Raphson
 *       process. See IntegratorBase::set_target_accuracy() for more info.
 * @ingroup integrators
 */
template <typename T, int num_stages = 2>
class RadauIntegrator final : public ImplicitIntegrator<T> {
  static_assert(num_stages == 1 || num_stages == 2,
      "Only 1-stage and 2-stage Radau are supported.");

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RadauIntegrator)

  explicit RadauIntegrator(const System<T>& system,
      Context<T>* context = nullptr);
  ~RadauIntegrator() final = default;

  bool supports_error_estimation() const final { return true; }

  /// This integrator uses embedded second order methods to compute estimates of
  /// the local truncation error. The order of the asymptotic difference between
  /// the third-order Radau method and an embedded second order method is O(h³).
  /// The order of the asymptotic difference between the first-order Radau
  /// method and an embedded second order method is O(h²).
  int get_error_estimate_order() const final {
    if (num_stages == 2) {
      return 3;
    } else {
      DRAKE_DEMAND(num_stages == 1);
      return 2;
    }
  }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return num_err_est_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return num_err_est_jacobian_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations()
      const final {
    return num_err_est_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return num_err_est_jacobian_reforms_;
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return num_err_est_iter_factorizations_;
  }

  void ComputeSolutionFromIterate(
      const VectorX<T>& xt0, const VectorX<T>& Z, VectorX<T>* xtplus) const;
  void ComputeAndSetErrorEstimate(
      const VectorX<T>& xtplus_prop, const VectorX<T>& xtplus_embed);
  bool AttemptStepPaired(const T& t0, const T& h,
      const VectorX<T>& xt0, VectorX<T>* xtplus_radau, VectorX<T>* xtplus_tr);
  const VectorX<T>& ComputeFofZ(
      const T& t0, const T& h, const VectorX<T>& xt0, const VectorX<T>& Z);
  void DoInitialize() final;
  void DoResetImplicitIntegratorStatistics() final;
  bool DoImplicitIntegratorStep(const T& h) final;
  bool StepRadau(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus, int trial = 1);
  bool StepImplicitTrapezoid(const T& t0, const T& h, const VectorX<T>& xt0,
      const VectorX<T>& dx0, const VectorX<T>& xtplus_radau,
      VectorX<T>* xtplus);
  static MatrixX<T> CalcTensorProduct(const MatrixX<T>& A, const MatrixX<T>& B);
  static void ComputeImplicitTrapezoidIterationMatrix(const MatrixX<T>& J,
      const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);
  static void ComputeRadauIterationMatrix(const MatrixX<T>& J, const T& h,
      const MatrixX<double>& A,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);
  bool StepImplicitTrapezoidDetail(const T& t0, const T& h,
      const VectorX<T>& xt0, const std::function<VectorX<T>()>& g,
      const VectorX<T>& xtplus_radau, VectorX<T>* xtplus, int trial = 1);

  // The num_stages-dimensional (constant) vector of time-scaling coefficients
  // common to Runge-Kutta-type integrators.
  std::vector<double> c_;

  // The num_stages x num_stages-dimensional (constant) matrix of stage-scaling
  // coefficients that are standard with Runge-Kutta-type integrators.
  MatrixX<double> A_;

  // The iteration matrix for the Radau method.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_radau_;

  // The iteration matrix for the implicit trapezoid method.
  typename ImplicitIntegrator<T>::IterationMatrix
      iteration_matrix_implicit_trapezoid_;

  // The (constant) tensor product between A_ and an identity matrix. This
  // product is computed only at initialization.
  MatrixX<T> A_tp_eye_;

  // The num_stages-dimensional (constant) solution propagation coefficients
  // (that also scales the stages) common to Runge-Kutta-type integrators.
  std::vector<double> b_;

  // The num_stages-dimensional (constant) scaling coefficients for Z (IV.8.2b)
  // in [Hairer, 1996].
  std::vector<double> d_;

  // A num_stages * |xc|-dimensional vector of the current iterate for the
  // Newton-Raphson process.
  VectorX<T> Z_;

  // The num_stages dimensional vector of derivative evaluations at every stage.
  VectorX<T> F_of_Z_;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Continuous state at the beginning of an integration step (stored to avoid
  // heap allocations).
  VectorX<T> xt0_;

  // Time-derivative of continuous state (stored to avoid heap allocations).
  VectorX<T> xdot_;

  // "Propagated" solution computed by the integrator (stored to avoid heap
  // allocations)- this is the solution that will be propagated forward in time.
  VectorX<T> xtplus_prop_;

  // "Error estimate" solution computed by the embedded method (stored to avoid
  // allocations)- this is the solution that will be used in concert with the
  // propagated solution to compute the error estimate.
  VectorX<T> xtplus_embed_;

  // 3/2 Bogacki-Shampine integrator used for propagation and error estimation
  // when the step size becomes smaller than the working minimum step size.
  std::unique_ptr<BogackiShampine3Integrator<T>> bs3_;

  // Second order Runge-Kutta integrator used for error estimation when the
  // step size becomes smaller than the working minimum step size.
  std::unique_ptr<RungeKutta2Integrator<T>> rk2_;

  // Statistics specific to this integrator.
  int64_t num_nr_iterations_{0};

  // Implicit trapezoid specific statistics.
  int64_t num_err_est_jacobian_reforms_{0};
  int64_t num_err_est_jacobian_function_evaluations_{0};
  int64_t num_err_est_iter_factorizations_{0};
  int64_t num_err_est_function_evaluations_{0};
  int64_t num_err_est_nr_iterations_{0};
};

/** A third-order fully implicit integrator with error estimation.
See RadauIntegrator with `num_stages == 2` for details.
@tparam_nonsymbolic_scalar */
template <typename T>
using Radau3Integrator = RadauIntegrator<T, 2>;

/** A first-order fully implicit integrator with error estimation.
See RadauIntegrator with `num_stages == 1` for details.
@tparam_nonsymbolic_scalar */
template <typename T>
using Radau1Integrator = RadauIntegrator<T, 1>;

}  // namespace systems
}  // namespace drake

// Declare class template initializations for double and AutoDiffXd.
// Note: We don't use the macros in drake/common/default_scalars.h because
// those macros are designed for functions with only one template argument, and
// we need to instantiate both scalar types for both the Radau1 and Radau3
// integrators, which have num_stages set 1 and 2, respectively.
extern template class drake::systems::RadauIntegrator<double, 1>;
extern template class drake::systems::RadauIntegrator<drake::AutoDiffXd, 1>;

extern template class drake::systems::RadauIntegrator<double, 2>;
extern template class drake::systems::RadauIntegrator<drake::AutoDiffXd, 2>;
