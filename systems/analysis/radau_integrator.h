#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/LU>

#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"

namespace drake {
namespace systems {

namespace internal {
#ifndef DRAKE_DOXYGEN_CXX
__attribute__((noreturn))
inline void EmitNoErrorEstimatorStatAndMessage() {
  throw std::logic_error("No error estimator is currently implemented, so "
      "query error estimator statistics is not yet supported.");
}
#endif
}  // namespace internal

/**
 * A third-order, fully implicit integrator without error estimation.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 * @tparam num_stages the number of stages used in this integrator. Set this to
 *         1 for the integrator to be implicit Euler and 2 for it to
 *         Radau3 (default). Other values are invalid.
 *
 * A two-stage Radau IIa (see [Hairer, 1996], Ch. 5) method is used for
 * propagating the state forward, by default. The integrator can also be
 * constructed using a single-stage method, in which case it is equivalent to
 * an implicit Euler method, by setting num_stages=1.
 *
 * Radau IIa methods are known to be L-Stable, meaning both that
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
template <class T, int num_stages = 2>
class RadauIntegrator final : public ImplicitIntegrator<T> {
  static_assert(num_stages == 1 || num_stages == 2,
      "Only 1-stage and 2-stage Radau are supported.");

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RadauIntegrator)

  explicit RadauIntegrator(const System<T>& system,
      Context<T>* context = nullptr);
  ~RadauIntegrator() final = default;

  bool supports_error_estimation() const final { return false; }

  int get_error_estimate_order() const final { return 0; }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations()
      const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  bool AttemptStep(const T& t0, const T& h,
      const VectorX<T>& xt0, VectorX<T>* xtplus_radau3);
  const VectorX<T>& ComputeFofZ(
      const T& t0, const T& h, const VectorX<T>& xt0, const VectorX<T>& Z);
  void DoInitialize() final;
  void DoResetImplicitIntegratorStatistics() final;
  bool DoImplicitIntegratorStep(const T& h) final;
  bool StepRadau(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus, int trial = 1);
  static MatrixX<T> CalcTensorProduct(const MatrixX<T>& A, const MatrixX<T>& B);
  static void ComputeRadauIterationMatrix(const MatrixX<T>& J, const T& h,
      const MatrixX<double>& A,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  // The num_stages-dimensional (constant) vector of time-scaling coefficients
  // common to Runge-Kutta-type integrators.
  std::vector<double> c_;

  // The num_stages x num_stages-dimensional (constant) matrix of stage-scaling
  // coefficients that are standard with Runge-Kutta-type integrators.
  MatrixX<double> A_;

  // The iteration matrix for the Radau method.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_radau3_;

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

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Statistics specific to this integrator.
  int64_t num_nr_iterations_{0};
};

template <class T, int num_stages>
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

template <class T, int num_stages>
void RadauIntegrator<T, num_stages>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
}

template <class T, int num_stages>
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

  // Verify that the maximum step size has been set.
  if (isnan(this->get_maximum_step_size()))
    throw std::logic_error("Maximum step size has not been set!");

  // Reset the Jacobian matrix (so that recomputation is forced).
  this->get_mutable_jacobian().resize(0, 0);
}

// Computes F(Z) used in [Hairer, 1996], (IV.8.4). This method evaluates
// the time derivatives of the system given the current iterate Z.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param Z the current iterate, of dimension state_dim * num_stages.
// @post the state of the internal context will be set to (t0, xt0) on return.
// @return a (state_dim * num_stages)-dimensional vector.
template <class T, int num_stages>
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

// Computes the next continuous state (at t0 + h) using the Radau method,
// assuming that the method is able to converge at that step size.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param[in,out] xtplus the starting guess for x(t+h); the value for x(t+h) on
//        return (assuming that h > 0).
// @param trial the attempt for this approach (1-4). StepRadau() uses more
//        computationally expensive methods as the trial numbers increase.
// @post the internal context will be in an indeterminate state on returning
//       `false`.
// @returns `true` if the method was successfully able to take an integration
//           step of size `h`.
template <class T, int num_stages>
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
      &iteration_matrix_radau3_)) {
    return false;
  }

  // Initialize the "last" norm of dx; this will be used to detect convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // The maximum number of Newton-Raphson iterations to take before declaring
  // failure. [Hairer, 1996] states, "It is our experience that the code becomes
  // more efficient when we allow a relatively high number of iterations (e.g.,
  // [7 or 10])", p. 121.
  // TODO(edrumwri): Consider making this a settable parameter. Not doing so
  //                 to avoid parameter overload.
  const int max_iterations = 10;

  // Do the Newton-Raphson iterations.
  for (int iter = 0; iter < max_iterations; ++iter) {
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
    VectorX<T> dZ = iteration_matrix_radau3_.Solve(
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

    // TODO(edrumwri): Replace this with CalcStateChangeNorm() when error
    // control has been implemented.
    // Get the norm of the update vector.
    T dx_norm = dx_state_->CopyToVector().norm();

    bool converged = (iter > 0 && this->IsUpdateZero(*xtplus, dx));
    SPDLOG_DEBUG(drake::log(), "norm(dx) indicates convergence? {}", converged);

    // Compute the convergence rate and check convergence.
    // [Hairer, 1996] notes that this convergence strategy should only be
    // applied after *at least* two iterations (p. 121).
    if (!converged && iter >= 1) {
      // TODO(edrumwri) Hairer's RADAU5 implementation (allegedly) uses
      // theta = sqrt(dx[k] / dx[k-2]) while DASSL uses
      // theta = pow(dx[k] / dx[0], 1/k), so investigate setting
      // theta to these alternative values for minimizing convergence failures.
      const T theta = dx_norm / last_dx_norm;
      const T eta = theta / (1 - theta);
      SPDLOG_DEBUG(drake::log(), "Newton-Raphson loop {} theta: {}, eta: {}",
                   iter, theta, eta);

      // Look for divergence.
      if (theta > 1) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson divergence detected for "
            "h={}", h);
        break;
      }

      // Look for convergence using Equation IV.8.10 from [Hairer, 1996].
      // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
      // efficiently on a number of test problems with *Radau5* (a fifth order
      // implicit integrator), p. 121. We select a value halfway in-between.
      const double kappa = 0.05;
      const double k_dot_tol = kappa * this->get_accuracy_in_use();
      if (eta * dx_norm < k_dot_tol) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson converged; η = {}, h = {}",
                     eta, h);
        converged = true;
      }
    }

    if (converged) {
      // Set the solution using (IV.8.2b) in [Hairer, 1996].
      xtplus->setZero();
      for (int i = 0, j = 0; i < num_stages; ++i, j += state_dim) {
        if (d_[i] != 0.0)
          *xtplus += d_[i] * Z_.segment(j, state_dim);
      }
      *xtplus += xt0;

      SPDLOG_DEBUG(drake::log(), "Final state: {}", xtplus->transpose());
      return true;
    }

    // Update the norm of the state update.
    last_dx_norm = dx_norm;
  }

  SPDLOG_DEBUG(drake::log(), "StepRadau() convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!this->get_reuse())
    return false;

  // Try StepRadau again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepRadau(t0, h, xt0, xtplus, trial+1);
}

// Steps Radau forward by h, if possible.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param[out] xtplus_radau3 contains the Radau integrator solution on return.
// @returns `true` if the integration was successful at the requested step size.
// @pre The time and state in the system's context (stored by the integrator)
//      are set to (t0, xt0) on entry.
// @post The time and state of the system's context (stored by the integrator)
//       will be set to t0+h and `xtplus_radau3` on successful exit (indicated
//       by this function returning `true`) and will be indeterminate on
//       unsuccessful exit (indicated by this function returning `false`).
template <class T, int num_stages>
bool RadauIntegrator<T, num_stages>::AttemptStep(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus_radau3) {
  using std::abs;
  DRAKE_ASSERT(xtplus_radau3);
  DRAKE_ASSERT(xtplus_radau3->size() == xt0.size());

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
  *xtplus_radau3 = xt0;

  // Do the Radau step.
  if (!StepRadau(t0, h, xt0, xtplus_radau3)) {
    SPDLOG_DEBUG(drake::log(), "Radau approach did not converge for "
        "step size {}", h);
    return false;
  }

  return true;
}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful.
/// @post the time and continuous state will be advanced only if `true` is
///       returned (if `false` is returned, the time and state will be reset
///       to their values on entry).
template <class T, int num_stages>
bool RadauIntegrator<T, num_stages>::DoImplicitIntegratorStep(const T& h) {
  Context<T>* context = this->get_mutable_context();

  // Save the current time and state.
  const T t0 = context->get_time();
  SPDLOG_DEBUG(drake::log(), "Radau DoStep(h={}) t={}", h, t0);

  // TODO(sherm1) Heap allocation here; consider mutable temporaries instead.
  const VectorX<T> xt0 = context->get_continuous_state().CopyToVector();
  VectorX<T> xtplus_radau3(xt0.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Euler step.
  if (h < this->get_working_minimum_step_size()) {
    SPDLOG_DEBUG(drake::log(), "-- requested step too small, taking explicit "
        "step instead");

    // Compute the Euler step.
    // TODO(sherm1) Heap allocation here; consider mutable temporary instead.
    VectorX<T> xdot = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_radau3 = xt0 + h * xdot;
    context->SetTimeAndContinuousState(t0 + h, xtplus_radau3);
  } else {
    // Try taking the requested step.
    bool success = AttemptStep(t0, h, xt0, &xtplus_radau3);

    // If the step was not successful, reset the time and state.
    if (!success) {
      context->SetTimeAndContinuousState(t0, xt0);
      return false;
    }
  }

  return true;
}

// Function for computing the iteration matrix for the Radau method. This
// is the matrix in [Hairer, 1996] (IV.8.4) on p.119.
template <class T, int num_stages>
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
template <class T, int num_stages>
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
