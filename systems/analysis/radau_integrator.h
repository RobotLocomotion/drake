#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/LU>

#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

/**
 * A selectable order (third- or first-order), fully implicit integrator with
 * error estimation.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 * @tparam num_stages the number of stages used in this integrator. Set this to
 *         1 for the integrator to be implicit Euler and 2 for it to
 *         Radau3 (default). Other values are invalid.
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
 * @note This integrator uses the integrator accuracy setting, even when run
 *       in fixed-step mode, to limit the error in the underlying Newton-Raphson
 *       process. See IntegratorBase::set_target_accuracy() for more info.
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
    return num_err_est_iter_factorizations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return num_err_est_function_evaluations_;
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

  enum class ConvergenceStatus {
    kDiverged,
    kConverged,
    kNotConverged,
  };
  ConvergenceStatus CheckConvergence(int iteration, const VectorX<T>& xtplus,
      const VectorX<T>& dx, const T& dx_norm, const T& last_dx_norm) const;
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

template <typename T, int num_stages>
void RadauIntegrator<T, num_stages>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
  num_err_est_jacobian_reforms_ = 0;
  num_err_est_jacobian_function_evaluations_ = 0;
  num_err_est_iter_factorizations_ = 0;
  num_err_est_function_evaluations_ = 0;
  num_err_est_nr_iterations_ = 0;
}

template <typename T, int num_stages>
RadauIntegrator<T, num_stages>::RadauIntegrator(const System<T>& system,
    Context<T>* context) : ImplicitIntegrator<T>(system, context) {
  A_.resize(num_stages, num_stages);

  // TODO(edrumwri) Convert A_, c_, b_, and d_ to fixed-size when Drake supports
  // "if constexpr" (C++17 feature). Lack of partial template function
  // specialization makes turning those into fixed-sizes painful at the moment.

  if (num_stages == 2) {
    // Set the matrix coefficients (from [Hairer, 1996] Table 5.5).
    A_(0, 0) = 5.0/12;     A_(0, 1) = -1.0/12;
    A_(1, 0) = 3.0/4;      A_(1, 1) = 1.0/4;

    // Set the time coefficients (from the same table).
    c_ = { 1.0/3, 1.0 };

    // Set the propagation constants (again, from the same table).
    b_ = { 3.0/4, 1.0/4 };

    // Set the scaling constants for the solution using (8.2b) and Table 5.6.
    d_ = { 0.0, 1.0 };
  } else {
    // For implicit Euler integration.
    A_(0, 0) = 1.0;
    c_ = { 1.0 };
    b_ = { 1.0 };
    d_ = { 1.0 };
  }
}

template <typename T, int num_stages>
void RadauIntegrator<T, num_stages>::DoInitialize() {
  using std::isnan;

  // Compute the tensor product of A with the identity matrix. A is a
  // num_stages x num_stages matrix. We need the tensor product to be a
  // m x m-dimensional matrix, where m = num_stages * state_dim. Thus the
  // number of rows/columns of the identity matrix is state_dim.

  const int state_dim =
      this->get_context().get_continuous_state_vector().size();

  // Compute A ⊗ I.
  // TODO(edrumwri) The resulting matrix only has s²n non-zeros out of s²n²
  // elements (where s is the number of stages)- take advantage of this.
  A_tp_eye_ = CalcTensorProduct(A_, MatrixX<T>::Identity(state_dim, state_dim));

  F_of_Z_.resize(state_dim * num_stages);

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  // TODO(edrumwri): Find the best values for the method.
  // These values are expected to be good for the two-stage and one-stage
  // methods, respectively.
  const double kDefaultAccuracy = (num_stages == 2) ? 1e-3 : 1e-1;
  const double kLoosestAccuracy = (num_stages == 2) ? 1e-2 : 5e-1;

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size())) {
      throw std::logic_error("Neither initial step size target nor maximum "
                                 "step size has been set!");
    }

    this->request_initial_step_size_target(
        this->get_maximum_step_size());
  }

  // If the user asks for accuracy that is looser than the loosest this
  // integrator can provide, use the integrator's loosest accuracy setting
  // instead.
  double working_accuracy = this->get_target_accuracy();
  if (isnan(working_accuracy))
    working_accuracy = kDefaultAccuracy;
  else if (working_accuracy > kLoosestAccuracy)
    working_accuracy = kLoosestAccuracy;
  this->set_accuracy_in_use(working_accuracy);

  // Reset the Jacobian matrix (so that recomputation is forced).
  this->get_mutable_jacobian().resize(0, 0);

  // Instantiate the embedded third order Bogacki-Shampine3 integrator. Note
  // that we do not worry about setting the initial step size, since that code
  // will never be triggered (the integrator will always be used in fixed-step
  // mode).
  bs3_ = std::make_unique<BogackiShampine3Integrator<T>>(
      this->get_system(),
      this->get_mutable_context());

  // Instantiate the embedded second-order Runge-Kutta integrator.
  rk2_ = std::make_unique<RungeKutta2Integrator<T>>(
      this->get_system(),
      std::numeric_limits<double>::max() /* no maximum step size */,
      this->get_mutable_context());

  // Maximum step size is not to be a constraint.
  bs3_->set_maximum_step_size(std::numeric_limits<double>::max());

  bs3_->Initialize();
  rk2_->Initialize();
  bs3_->set_fixed_step_mode(true);
  // Note: RK2 only operates in fixed step mode.
}

// Computes F(Z) used in [Hairer, 1996], (IV.8.4). This method evaluates
// the time derivatives of the system given the current iterate Z.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param Z the current iterate, of dimension state_dim * num_stages.
// @post the state of the internal context will be set to (t0, xt0) on return.
// @return a (state_dim * num_stages)-dimensional vector.
template <typename T, int num_stages>
const VectorX<T>& RadauIntegrator<T, num_stages>::ComputeFofZ(
      const T& t0, const T& h, const VectorX<T>& xt0, const VectorX<T>& Z) {
  Context<T>* context = this->get_mutable_context();
  const int state_dim = xt0.size();

  // Evaluate the derivative at each stage.
  for (int i = 0, j = 0; i < num_stages; ++i, j += state_dim) {
    const auto Z_i = Z.segment(j, state_dim);
    context->SetTimeAndContinuousState(t0 + c_[i] * h, xt0 + Z_i);
    auto F_i = F_of_Z_.segment(j, state_dim);
    F_i = this->EvalTimeDerivatives(*context).CopyToVector();
  }

  return F_of_Z_;
}

// Computes the solution xtplus (i.e., the continuous state at t0 + h) from the
// continuous state at t0 and the current Newton-Raphson iterate (Z).
template <typename T, int num_stages>
void RadauIntegrator<T, num_stages>::ComputeSolutionFromIterate(
    const VectorX<T>& xt0, const VectorX<T>& Z, VectorX<T>* xtplus) const {
  const int state_dim = xt0.size();

  // Compute the solution using (IV.8.2b) in [Hairer, 1996].
  xtplus->setZero();
  for (int i = 0, j = 0; i < num_stages; ++i, j += state_dim) {
    if (d_[i] != 0.0)
      *xtplus += d_[i] * Z.segment(j, state_dim);
  }
  *xtplus += xt0;
}

// Checks the Newton-Raphson iteration process for convergence.
template <typename T, int num_stages>
typename RadauIntegrator<T, num_stages>::ConvergenceStatus
RadauIntegrator<T, num_stages>::CheckConvergence(
    int iteration, const VectorX<T>& xtplus, const VectorX<T>& dx,
    const T& dx_norm, const T& last_dx_norm) const {
  // The check below looks for convergence by identifying cases where the
  // update to the state results in no change. We do this check only after
  // at least one Newton-Raphson update has been applied to ensure that there
  // is at least some change to the state, no matter how small, on a
  // non-stationary system.
  if (iteration > 0 && this->IsUpdateZero(xtplus, dx)) {
    SPDLOG_DEBUG(
        drake::log(), "magnitude of state update indicates convergence");
    return ConvergenceStatus::kConverged;
  }

  // Compute the convergence rate and check convergence.
  // [Hairer, 1996] notes that this convergence strategy should only be
  // applied after *at least* two iterations (p. 121).
  if (iteration > 1) {
    // TODO(edrumwri) Hairer's RADAU5 implementation (allegedly) uses
    // theta = sqrt(dx[k] / dx[k-2]) while DASSL uses
    // theta = pow(dx[k] / dx[0], 1/k), so investigate setting
    // theta to these alternative values for minimizing convergence failures.
    const T theta = dx_norm / last_dx_norm;
    const T eta = theta / (1 - theta);
    SPDLOG_DEBUG(drake::log(), "Newton-Raphson loop {} theta: {}, eta: {}",
                 iteration, theta, eta);

    // Look for divergence.
    if (theta > 1) {
      SPDLOG_DEBUG(drake::log(), "Newton-Raphson divergence detected");
      return ConvergenceStatus::kDiverged;
    }

    // Look for convergence using Equation IV.8.10 from [Hairer, 1996].
    // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
    // efficiently on a number of test problems with *Radau5* (a fifth order
    // implicit integrator), p. 121. We select a value halfway in-between.
    const double kappa = 0.05;
    const double k_dot_tol = kappa * this->get_accuracy_in_use();
    if (eta * dx_norm < k_dot_tol) {
      SPDLOG_DEBUG(drake::log(), "Newton-Raphson converged; η = {}", eta);
      return ConvergenceStatus::kConverged;
    }
  }

  return ConvergenceStatus::kNotConverged;
}

// Computes the next continuous state (at t0 + h) using the Radau method,
// assuming that the method is able to converge at that step size.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param[out] xtplus the value for x(t+h) on return.
// @param trial the attempt for this approach (1-4). StepRadau() uses more
//        computationally expensive methods as the trial numbers increase.
// @post the internal context will be in an indeterminate state on returning
//       `false`.
// @returns `true` if the method was successfully able to take an integration
//           step of size `h`.
template <typename T, int num_stages>
bool RadauIntegrator<T, num_stages>::StepRadau(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(1 <= trial && trial <= 4);

  // Set the state.
  Context<T>* context = this->get_mutable_context();
  context->SetTimeAndContinuousState(t0, xt0);

  const int state_dim = xt0.size();

  // Verify xtplus
  DRAKE_ASSERT(xtplus && xtplus->size() == state_dim);

  SPDLOG_DEBUG(drake::log(), "StepRadau() entered for t={}, h={}, trial={}",
               t0, h, trial);

  // TODO(edrumwri) Experiment with setting this as recommended in
  // [Hairer, 1996], p. 120.
  // Initialize the z iterate using (IV.8.5) in [Hairer, 1996], p. 120 (and
  // the corresponding xt+).
  Z_.setZero(state_dim * num_stages);
  *xtplus = xt0;

  // Set the iteration matrix construction method.
  auto construct_iteration_matrix = [this](const MatrixX<T>& J, const T& dt,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
    ComputeRadauIterationMatrix(J, dt, this->A_, iteration_matrix);
  };

  // Calculate Jacobian and iteration matrices (and factorizations), as needed.
  if (!this->MaybeFreshenMatrices(t0, xt0, h, trial, construct_iteration_matrix,
      &iteration_matrix_radau_)) {
    return false;
  }

  // Initialize the "last" norm of dx; this will be used to detect convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Do the Newton-Raphson iterations.
  for (int iter = 0; iter < this->max_newton_raphson_iterations(); ++iter) {
    SPDLOG_DEBUG(drake::log(), "Newton-Raphson iteration {}", iter);

    // Update the number of Newton-Raphson iterations.
    ++num_nr_iterations_;

    // Evaluate the derivatives using the current iterate.
    const VectorX<T>& F_of_Z = ComputeFofZ(t0, h, xt0, Z_);

    // Compute the state update using (IV.8.4) in [Hairer, 1996], p. 119, i.e.:
    // Solve (I − hA⊗J) ΔZᵏ = h (A⊗I) F(Zᵏ) - Zᵏ for ΔZᵏ, where:
    // A_tp_eye ≡ (A⊗I) and (I − hA⊗J) is the iteration matrix.
    SPDLOG_DEBUG(drake::log(), "residual: {}",
        (A_tp_eye_ * (h * F_of_Z) - Z_).transpose());
    VectorX<T> dZ = iteration_matrix_radau_.Solve(
        A_tp_eye_ * (h * F_of_Z) - Z_);

    // Update the iterate.
    Z_ += dZ;

    // Compute the update to the actual continuous state (i.e., x not Z) using
    // (IV.8.2b) in [Hairer, 1996], which gives the relationship between x(t0+h)
    // and Z:
    // x(t0+h) = x(t0) + Σ dᵢ Zᵢ
    // Therefore, we can get the relationship between dZ and dx as:
    // x* = x(t0) + Σ dᵢ Zᵢ                   (1)
    // x+ = x(t0) + Σ dᵢ (Zᵢ + dZᵢ)           (2)
    // Subtracting (1) from (2) yields
    // dx = Σ dᵢ Zᵢ
    // where dx ≡ x+ - x*
    VectorX<T> dx = VectorX<T>::Zero(state_dim);
    for (int i = 0, j = 0; i < num_stages; ++i, j += state_dim) {
      if (d_[i] != 0.0)
        dx += d_[i] * dZ.segment(j, state_dim);
    }

    dx_state_->SetFromVector(dx);
    SPDLOG_DEBUG(drake::log(), "dx: {}", dx.transpose());

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

    // Compute the update.
    ComputeSolutionFromIterate(xt0, Z_, &(*xtplus));

    // Check for convergence.
    ConvergenceStatus status =
        CheckConvergence(iter, *xtplus, dx, dx_norm, last_dx_norm);
    if (status == ConvergenceStatus::kConverged) return true;  // We win.
    if (status == ConvergenceStatus::kDiverged) break;  // Try something else.
    DRAKE_DEMAND(status == ConvergenceStatus::kNotConverged);

    // Update the norm of the state update.
    last_dx_norm = dx_norm;
  }

  SPDLOG_DEBUG(drake::log(), "StepRadau() convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try; otherwise, the following code will recurse
  // into this function again, and freshen computations as helpful.
  if (!this->get_reuse())
    return false;

  // Try StepRadau again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepRadau(t0, h, xt0, xtplus, trial+1);
}

// Computes the next continuous state (at t0 + h) using the implicit trapezoid
// method, assuming that the method is able to converge at that step size.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param radau_xtplus the Radau solution for x(t+h).
// @param[out] xtplus the value for x(t+h) on return.
// @param trial the attempt for this approach (1-4). The method uses more
//        computationally expensive methods as the trial numbers increase.
// @returns `true` if the method was successfully able to take an integration
//           step of size `h` (or `false` otherwise).
template <typename T, int num_stages>
bool RadauIntegrator<T, num_stages>::StepImplicitTrapezoid(const T& t0,
    const T& h, const VectorX<T>& xt0, const VectorX<T>& dx0,
    const VectorX<T>& radau_xtplus, VectorX<T>* xtplus) {
  using std::abs;

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoid(h={}) t={}",
               h, t0);

  // Define g(x(t+h)) ≡ x(t+h) - x(t) - h/2 (f(t,x(t)) + f(t+h,x(t+h)) and
  // evaluate it at the current x(t+h).
  Context<T>* context = this->get_mutable_context();
  std::function<VectorX<T>()> g =
      [&xt0, h, &dx0, context, this]() {
        return (context->get_continuous_state().CopyToVector() - xt0 - h/2 *
            (dx0 + this->EvalTimeDerivatives(
                this->get_context()).CopyToVector())).eval();
      };

  // Store statistics before calling StepAbstract(). The difference between
  // the modified statistics and the stored statistics will be used to compute
  // the trapezoid method-specific statistics.
  int stored_num_jacobian_evaluations = this->get_num_jacobian_evaluations();
  int stored_num_iter_factorizations =
      this->get_num_iteration_matrix_factorizations();
  int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  int64_t stored_num_jacobian_function_evaluations =
      this->get_num_derivative_evaluations_for_jacobian();
  int stored_num_nr_iterations = this->get_num_newton_raphson_iterations();

  // Step.
  bool success = StepImplicitTrapezoidDetail(
      t0, h, xt0, g, radau_xtplus, xtplus);

  // Move statistics to implicit trapezoid-specific.
  num_err_est_jacobian_reforms_ +=
      this->get_num_jacobian_evaluations() - stored_num_jacobian_evaluations;
  num_err_est_iter_factorizations_ +=
      this->get_num_iteration_matrix_factorizations() -
      stored_num_iter_factorizations;
  num_err_est_function_evaluations_ +=
      this->get_num_derivative_evaluations() - stored_num_function_evaluations;
  num_err_est_jacobian_function_evaluations_ +=
      this->get_num_derivative_evaluations_for_jacobian() -
      stored_num_jacobian_function_evaluations;
  num_err_est_nr_iterations_ += this->get_num_newton_raphson_iterations() -
      stored_num_nr_iterations;

  return success;
}

// Does all of the real work for the implicit trapezoid method.
template <typename T, int num_stages>
bool RadauIntegrator<T, num_stages>::StepImplicitTrapezoidDetail(
    const T& t0, const T& h,
    const VectorX<T>& xt0, const std::function<VectorX<T>()>& g,
    const VectorX<T>& radau_xtplus, VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);

  // Verify xtplus.
  Context<T>* context = this->get_mutable_context();
  DRAKE_ASSERT(xtplus &&
               xtplus->size() == context->get_continuous_state_vector().size());

  // Start from the Radau solution, which is close (either O(h³) accurate or
  // O(h) accurate, depending on the number of stages) to the true solution and
  // hence should be an excellent starting point.
  *xtplus = radau_xtplus;

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoidDetail() entered for t={}, "
      "h={}, trial={}", t0, h, trial);

  // Advance the context time; this means that all derivatives will be computed
  // at t+dt.
  const T tf = t0 + h;
  context->SetTimeAndContinuousState(tf, *xtplus);

  // Evaluate the residual error using the current x(t+h).
  VectorX<T> goutput = g();

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Calculate Jacobian and iteration matrices (and factorizations), as needed.
  // Note that this method computes the Jacobian matrix around (tf, *xtplus),
  // where *xtplus is the solution computed by the Radau method, whereas the
  // Radau3 method computes it around (t0, xt0).
  if (!this->MaybeFreshenMatrices(t0, *xtplus, h, trial,
      ComputeImplicitTrapezoidIterationMatrix,
      &iteration_matrix_implicit_trapezoid_)) {
    return false;
  }

  for (int iter = 0; iter < this->max_newton_raphson_iterations(); ++iter) {
    SPDLOG_DEBUG(drake::log(), "Newton-Raphson iteration {}", iter);
    ++num_nr_iterations_;

    // Compute the state update using the equation A*x = -g(), where A is the
    // iteration matrix.
    // TODO(edrumwri): Allow caller to provide their own solver.
    VectorX<T> dx = iteration_matrix_implicit_trapezoid_.Solve(-goutput);
    SPDLOG_DEBUG(drake::log(), "dx: {}", dx.transpose());

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

    // Update the state vector.
    *xtplus += dx;
    context->SetTimeAndContinuousState(tf, *xtplus);

    // Check for convergence.
    ConvergenceStatus status =
        CheckConvergence(iter, *xtplus, dx, dx_norm, last_dx_norm);
    if (status == ConvergenceStatus::kConverged) return true;  // We win.
    if (status == ConvergenceStatus::kDiverged) break;  // Try something else.
    DRAKE_DEMAND(status == ConvergenceStatus::kNotConverged);

    // Update the norm of the state update.
    last_dx_norm = dx_norm;

    // Update the state in the context and compute g(xⁱ⁺¹).
    goutput = g();
  }

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoidDetail() convergence "
      "failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!this->get_reuse())
    return false;

  // Try the step again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepImplicitTrapezoidDetail(
      t0, h, xt0, g, radau_xtplus, xtplus, trial + 1);
}

// Steps Radau forward by h, if possible.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param[out] xtplus_radau contains the Radau integrator solution on return.
// @param[out] xtplus_itr contains the implicit trapezoid solution on return.
// @returns `true` if the integration was successful at the requested step size.
// @pre The time and state in the system's context (stored by the integrator)
//      are set to (t0, xt0) on entry.
// @post The time and state of the system's context (stored by the integrator)
//       will be set to t0+h and `xtplus_radau` on successful exit (indicated
//       by this function returning `true`) and will be indeterminate on
//       unsuccessful exit (indicated by this function returning `false`).
template <typename T, int num_stages>
bool RadauIntegrator<T, num_stages>::AttemptStepPaired(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus_radau, VectorX<T>* xtplus_itr) {
  using std::abs;
  DRAKE_ASSERT(xtplus_radau);
  DRAKE_ASSERT(xtplus_itr);
  DRAKE_ASSERT(xtplus_radau->size() == xt0.size());
  DRAKE_ASSERT(xtplus_itr->size() == xt0.size());

  // Set the time and state in the context.
  this->get_mutable_context()->SetTimeAndContinuousState(t0, xt0);

  // Compute the derivative at xt0. NOTE: the derivative is calculated at this
  // point (early on in the integration process) in order to reuse the
  // derivative evaluation, via the cache, from the last integration step (if
  // possible).
  const VectorX<T> dx0 = this->EvalTimeDerivatives(
      this->get_context()).CopyToVector();

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  *xtplus_radau = xt0;

  // Do the Radau step.
  if (!StepRadau(t0, h, xt0, xtplus_radau)) {
    SPDLOG_DEBUG(drake::log(), "Radau approach did not converge for "
        "step size {}", h);
    return false;
  }

  // The error estimation process uses the implicit trapezoid method, which
  // is defined as:
  // x(t+h) = x(t) + h/2 (f(t, x(t) + f(t+h, x(t+h))
  // x(t+h) from the Radau method is presumably a good starting point.

  // The error estimate for 2-stage (3rd order) Radau is derived as follows
  // (thanks to Michael Sherman):
  // x*(t+h) = xᵣ₃(t+h) + O(h⁴)      [Radau3]
  //         = xₜ(t+h) + O(h³)       [implicit trapezoid]
  // where x*(t+h) is the true (generally unknown) answer that we seek.
  // This implies:
  // xᵣ₃(t+h) + O(h⁴) = xₜ(t+h) + O(h³)
  // Given that the third order term subsumes the fourth order one:
  // xᵣ₃(t+h) - xₜ(t+h) = O(h³)
  // Therefore the asymptotic term is third order.

  // For 1-stage (1st order) Radau, the error estimate is derived analogously:
  // x*(t+h) = xᵣ₁(t+h) + O(h²)      [Radau1]
  //         = xₜ(t+h) + O(h³)       [implicit trapezoid]
  // By the same reasoning as above, this implies that:
  // xᵣ₁(t+h) - xₜ(t+h) = O(h²)
  // In this case, the asymptotic term is second order.

  // One subtlety in this analysis is that the first case (with Radau3) gives
  // an error estimate for the implicit trapezoid method, while the second case
  // gives an error estimate for the Radau1 method. Put another way: the higher
  // order result is propagated in the 3rd order method while the lower order
  // result is propagated in the 1st order method.

  // Attempt to compute the implicit trapezoid solution.
  if (StepImplicitTrapezoid(t0, h, xt0, dx0, *xtplus_radau, xtplus_itr)) {
    // Reset the state to that computed by Radau3.
    this->get_mutable_context()->SetTimeAndContinuousState(
        t0 + h, *xtplus_radau);
    return true;
  } else {
    SPDLOG_DEBUG(drake::log(), "Implicit trapezoid approach FAILED with a step"
        "size that succeeded on Radau3.");
    return false;
  }

  return true;
}

// Updates the error estimate from the propagated solution and the embedded
// solution.
template <typename T, int num_stages>
void RadauIntegrator<T, num_stages>::ComputeAndSetErrorEstimate(
    const VectorX<T>& xtplus_prop, const VectorX<T>& xtplus_embed) {
  err_est_vec_ = xtplus_prop - xtplus_embed;
  err_est_vec_ = err_est_vec_.cwiseAbs();

  // Compute and set the error estimate.
  SPDLOG_DEBUG(drake::log(), "Error estimate: {}", err_est_vec_.transpose());
  this->get_mutable_error_estimate()->get_mutable_vector().
      SetFromVector(err_est_vec_);
}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful.
/// @post the time and continuous state will be advanced only if `true` is
///       returned (if `false` is returned, the time and state will be reset
///       to their values on entry).
template <typename T, int num_stages>
bool RadauIntegrator<T, num_stages>::DoImplicitIntegratorStep(const T& h) {
  Context<T>* context = this->get_mutable_context();

  // Save the current time and state.
  const T t0 = context->get_time();
  SPDLOG_DEBUG(drake::log(), "Radau DoStep(h={}) t={}", h, t0);

  xt0_ = context->get_continuous_state().CopyToVector();
  xtplus_prop_.resize(xt0_.size());
  xtplus_embed_.resize(xt0_.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Bogacki-Shampine/explicit Euler step, depending on the
  // number of stages in use.
  if (h < this->get_working_minimum_step_size()) {
    SPDLOG_DEBUG(drake::log(), "-- requested step too small, taking explicit "
        "step instead");

    // We want to maintain the order of the error estimation process even as we
    // take this very small step.
    if (num_stages == 2) {
      // The BS3 integrator provides exactly the same order as 2-stage
      // Radau + embedded implicit trapezoid.
      const int evals_before_bs3 = bs3_->get_num_derivative_evaluations();
      DRAKE_DEMAND(bs3_->IntegrateWithSingleFixedStepToTime(t0 + h));
      const int evals_after_bs3 = bs3_->get_num_derivative_evaluations();
      this->get_mutable_error_estimate()->SetFrom(*bs3_->get_error_estimate());
      this->add_derivative_evaluations(evals_after_bs3 - evals_before_bs3);
    } else {
      // First-order Euler + RK2 provides exactly the same order as 1-stage
      // Radau + embedded implicit trapezoid.
      DRAKE_DEMAND(num_stages == 1);

      // Compute the Euler step.
      xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
      xtplus_prop_ = xt0_ + h * xdot_;

      // Compute the RK2 step.
      const int evals_before_rk2 = rk2_->get_num_derivative_evaluations();
      DRAKE_DEMAND(rk2_->IntegrateWithSingleFixedStepToTime(t0 + h));
      const int evals_after_rk2 = rk2_->get_num_derivative_evaluations();

      // Update the error estimation ODE counts.
      num_err_est_function_evaluations_ += (evals_after_rk2 - evals_before_rk2);

      // Store the embedded solution.
      xtplus_embed_ = context->get_continuous_state().CopyToVector();

      // Reset the state to the propagated solution.
      context->SetTimeAndContinuousState(t0 + h, xtplus_prop_);

      // Update the error estimate.
      ComputeAndSetErrorEstimate(xtplus_prop_, xtplus_embed_);
    }
  } else {
    // Try taking the requested step.
    bool success = AttemptStepPaired(
        t0, h, xt0_, &xtplus_prop_, &xtplus_embed_);

    // If the step was not successful, reset the time and state.
    if (!success) {
      context->SetTimeAndContinuousState(t0, xt0_);
      return false;
    }

    // Update the error estimate.
    ComputeAndSetErrorEstimate(xtplus_prop_, xtplus_embed_);
  }

  return true;
}

// Function for computing the iteration matrix for the Implicit Trapezoid
// method.
template <typename T, int num_stages>
void RadauIntegrator<T, num_stages>::ComputeImplicitTrapezoidIterationMatrix(
    const MatrixX<T>& J,
    const T& h,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows();
  // TODO(edrumwri) Investigate how to do the below operation with a move.
  iteration_matrix->SetAndFactorIterationMatrix(J * (-h / 2.0) +
      MatrixX<T>::Identity(n, n));
}

// Function for computing the iteration matrix for the Radau method. This
// is the matrix in [Hairer, 1996] (IV.8.4) on p.119.
template <typename T, int num_stages>
void RadauIntegrator<T, num_stages>::ComputeRadauIterationMatrix(
    const MatrixX<T>& J,
    const T& h,
    const MatrixX<double>& A,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows() * num_stages;
  // TODO(edrumwri) Investigate how to do the below operation with a move.
  // Computes I - h A ⊗ J.
  iteration_matrix->SetAndFactorIterationMatrix(
      CalcTensorProduct(A * -h, J) + MatrixX<T>::Identity(n , n));
}

// Computes the tensor product between two matrices. Given
// A = | a11 ... a1m |
//     | ...     ... |
//     | an1 ... anm |
// and some matrix B, the tensor product is:
// A ⊗ B = | a11B ... a1mB |
//         | ...      ...  |
//         | an1B ... anmB |
template <typename T, int num_stages>
MatrixX<T> RadauIntegrator<T, num_stages>::CalcTensorProduct(
    const MatrixX<T>& A, const MatrixX<T>& B) {
  const int rows_A = A.rows();
  const int cols_A = A.cols();
  const int rows_B = B.rows();
  const int cols_B = B.cols();
  MatrixX<T> AB(rows_A * rows_B, cols_A * cols_B);
  for (int i = 0, ii = 0; i < rows_A; ++i, ii += rows_B) {
    for (int j = 0, jj = 0; j < cols_A; ++j, jj += cols_B) {
      AB.block(ii, jj, rows_B, cols_B) = A(i, j) * B;
    }
  }

  return AB;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::RadauIntegrator)
