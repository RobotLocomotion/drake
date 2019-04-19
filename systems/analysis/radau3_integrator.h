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

/**
 * A third-order, fully implicit integrator without error estimation.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 * @tparam NumStages the number of stages used in this integrator. Set this to 1
 *                   for the integrator to be implicit Euler and 2 for it to be
 *                   Radau3 (default). Other values are invalid.
 *
 * A two-stage Radau IIa (see [Hairer, 1996], Ch. 5) method is used for
 * propagating the state forward, by default. The integrator can also be
 * constructed using a single-stage method, in which case it is equivalent to
 * an implicit Euler method, by setting NumStages=1.
 *
 * The Radau3 method is known to be L-Stable, meaning both that
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
 * This implementation uses Newton-Raphson (NR). General implementational
 * details were taken from [Hairer, 1996] Ch. 8.
 *
 * See ImplicitIntegrator class documentation for further details.
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996.
 * - [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
 *                    Equations. John Wiley & Sons, 1991. * 
 */
template <class T, int NumStages = 2>
class Radau3Integrator final : public ImplicitIntegrator<T> {
  static_assert(NumStages == 1 || NumStages == 2,
      "Only 1-stage and 2-stage Radau are supported.");

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Radau3Integrator)

  explicit Radau3Integrator(const System<T>& system,
      Context<T>* context = nullptr);
  ~Radau3Integrator() override = default;

  bool supports_error_estimation() const override { return false; }

  int get_error_estimate_order() const override { return 0; }

 private:
  int64_t EmitNoErrorEstimatorStatAndMessage() const {
    throw std::logic_error("No error estimator is currently implemented, so "
        "query error estimator statistics is not yet supported.");
    return 0;
  }

  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_iteration_matrix_factorizations() const final {
    return num_iter_factorizations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations()
      const final {
    return EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return EmitNoErrorEstimatorStatAndMessage();
  }

  bool AttemptStep(const T& t0, const T& h,
      const VectorX<T>& xt0, VectorX<T>* xtplus_radau3);
  const VectorX<T>& ComputeFofg(
      const T& t0, const T& h, const VectorX<T>& xt0, const VectorX<T>& Z);
  void DoInitialize() override;
  void DoResetImplicitIntegratorStatistics() override;
  bool DoStep(const T& h) override;
  bool StepRadau3(const T& t0, const T& h, const VectorX<T>& xt0,
      VectorX<T>* xtplus, int trial = 1);
  static MatrixX<T> CalcTensorProduct(const MatrixX<T>& A, const MatrixX<T>& B);
  static void ComputeRadau3IterationMatrix(const MatrixX<T>& J, const T& h,
      const MatrixX<double>& A,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);
  bool CalcMatrices(const T& t, const VectorX<T>& xt, const T& h,
      const std::function<void(const MatrixX<T>&, const T&,
          typename ImplicitIntegrator<T>::IterationMatrix*)>&
      recompute_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      int trial);

  // The time-scaling coefficients standard with Runge-Kutta-type integrators.
  std::vector<double> c_;

  // The stage-scaling coefficients standard with Runge-Kutta-type integrators.
  MatrixX<double> A_;

  // The iteration matrix for the Radau3 method.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_radau3_;

  // The tensor product between A_ and an identity matrix. This product is
  // computed on initialization and then stored.
  MatrixX<T> A_tp_eye_;

  // The solution propagation coefficients (that also scales the stages),
  // standard with Runge-Kutta-type integrators.
  std::vector<double> b_;

  // The scaling coefficients for Z (8.2b) in [Hairer, 1996].
  std::vector<double> d_;

  // A NumStages * |xc|-dimensional vector of the current iterate for the
  // Newton-Raphson process.
  VectorX<T> Z_;

  // The derivative evaluations at t0 and every stage.
  VectorX<T> F_of_g_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Statistics specific to this integrator.
  int64_t num_nr_iterations_{0};
  int64_t num_iter_factorizations_{0};
};

template <class T, int NumStages>
Radau3Integrator<T, NumStages>::Radau3Integrator(const System<T>& system,
    Context<T>* context) : ImplicitIntegrator<T>(system, context) {
  A_.resize(NumStages, NumStages);

  if (NumStages == 2) {
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
    // For Euler integration.
    A_(0, 0) = 1.0;
    c_ = { 1.0 };
    b_ = { 1.0 };
    d_ = { 1.0 };
  }
}

template <class T, int NumStages>
void Radau3Integrator<T, NumStages>::DoResetImplicitIntegratorStatistics() {
  num_iter_factorizations_ = 0;
  num_nr_iterations_ = 0;
}

template <class T, int NumStages>
void Radau3Integrator<T, NumStages>::DoInitialize() {
  using std::isnan;

  // Compute the tensor product of A with the identity matrix. A is a
  // NumStages x NumStages matrix. We need the tensor product to be a
  // m x m-dimensional matrix, where m = NumStages * state_dim. Thus the
  // number of rows/columns of the identity matrix is state_dim.

  const int state_dim =
      this->get_context().get_continuous_state_vector().size();
  A_tp_eye_ = CalcTensorProduct(A_, MatrixX<T>::Identity(state_dim, state_dim));

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  // Verify that the maximum step size has been set.
  if (isnan(this->get_maximum_step_size()))
    throw std::logic_error("Maximum step size has not been set!");

  // Reset the Jacobian matrix (so that recomputation is forced).
  this->get_mutable_jacobian().resize(0, 0);
}

// Computes F(Z) used in [Hairer, 1996], (8.4). This method evaluates
// the time derivatives of the system given the current iterate Z.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param Z the current iterate.
// @post the state of the internal context will be set to (t0, xt0) on return.
template <class T, int NumStages>
const VectorX<T>& Radau3Integrator<T, NumStages>::ComputeFofg(
      const T& t0, const T& h, const VectorX<T>& xt0, const VectorX<T>& Z) {
  Context<T>* context = this->get_mutable_context();
  const int state_dim = xt0.size();
  F_of_g_.resize(state_dim * NumStages);

  // Evaluate the derivative at each stage.
  for (int i = 0, j = 0; i < NumStages; ++i, j += state_dim) {
    const auto Z_i = Z.segment(j, state_dim);
    context->SetTimeAndContinuousState(t0 + c_[i] * h, xt0 + Z_i);
    auto F_i = F_of_g_.segment(j, state_dim);
    F_i = this->EvalTimeDerivativesUsingContext();
  }

  return F_of_g_;
}

// Computes the next continuous state (at t0 + h) using the Radau3 method,
// assuming that the method is able to converge at that step size.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param [in,out] the starting guess for x(t+h); the value for x(t+h) on
//        return (assuming that h > 0).
// @param trial the attempt for this approach (1-4). StepRadau3() uses more
//        computationally expensive methods as the trial numbers increase.
// @post the internal context will be in an indeterminate state on returning
//       `false`.
// @returns `true` if the method was successfully able to take an integration
//           step of size `h` (or `false` otherwise).
template <class T, int NumStages>
bool Radau3Integrator<T, NumStages>::StepRadau3(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);

  // Set the state.
  Context<T>* context = this->get_mutable_context();
  context->SetTimeAndContinuousState(t0, xt0);

  const int state_dim = xt0.size();

  // Verify xtplus
  DRAKE_ASSERT(xtplus && xtplus->size() == state_dim);

  SPDLOG_DEBUG(drake::log(), "StepRadau3() entered for t={}, h={}, trial={}",
               t0, h, trial);

  // Initialize the z iterate using (8.5) in [Hairer, 1996], p. 120.
  Z_.setZero(state_dim * NumStages);

  // TODO(edrumwri) Experiment with setting this as recommended in
  // [Hairer, 1996], p. 120.
  // Initialize xt+.
  *xtplus = xt0;

  // Set the iteration matrix construction method.
  auto construct_iteration_matrix = [this](const MatrixX<T>& J, const T& dt,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
    ComputeRadau3IterationMatrix(J, dt, this->A_, iteration_matrix);
  };

  // Calculate Jacobian and iteration matrices (and factorizations), as needed.
  if (!CalcMatrices(t0, xt0, h, construct_iteration_matrix,
      &iteration_matrix_radau3_, trial)) {
    this->set_last_call_succeeded(false);
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
    const VectorX<T>& F_of_g = ComputeFofg(t0, h, xt0, Z_);

    // Compute the state update using (8.4) in [Hairer, 1996], p. 119.
    SPDLOG_DEBUG(drake::log(), "residual: {}",
        (A_tp_eye_ * (h * F_of_g) - Z_).transpose());
    VectorX<T> dZ = iteration_matrix_radau3_.Solve(
        A_tp_eye_ * (h * F_of_g) - Z_);

    // Update the iterate.
    Z_ += dZ;

    // Compute the update to the actual continuous state (i.e., x not Z) using
    // (8.2b) in [Hairer, 1996], which gives the relationship between x(t0+h)
    // and Z:
    // x(t0+h) = x(t0) + Σ dᵢ Zᵢ
    // Therefore, we can get the relationship between dZ and dx as:
    // x* = x(t0) + Σ dᵢ Zᵢ                   (1)
    // x+ = x(t0) + Σ dᵢ (Zᵢ + dZᵢ)           (2)
    // Subtracting (1) from (2) yields
    // dx = Σ dᵢ Zᵢ
    // where dx ≡ x+ - x*
    VectorX<T> dx = VectorX<T>::Zero(state_dim);
    for (int i = 0, j = 0; i < NumStages; ++i, j += state_dim)
      dx += d_[i] * dZ.segment(j, state_dim);

    dx_state_->SetFromVector(dx);
    SPDLOG_DEBUG(drake::log(), "dx: {}", dx.transpose());

    // TODO(edrumwri): Replace this with CalcStateChangeNorm() when error
    // control has been implemented.
    // Get the norm of the update vector.
    T dx_norm = dx_state_->CopyToVector().norm();

    // The check below looks for convergence using machine epsilon. Without
    // this check, the convergence criteria can be applied when
    // |dZ_norm| ~ 1e-22 (one example taken from practice), which does not
    // allow the norm to be reduced further. What happens: dx_norm will become
    // equivalent to last_dZ_norm, making theta = 1, and eta = infinity. Thus,
    // convergence would never be identified.
    bool converged = (dx_norm < 10 * std::numeric_limits<double>::epsilon());
    SPDLOG_DEBUG(drake::log(), "norm(dx) indicates convergence? {}", converged);

    // Compute the convergence rate and check convergence.
    // [Hairer, 1996] notes that this convergence strategy should only be
    // applied after *at least* two iterations (p. 121).
    if (!converged && iter >= 1) {
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

      // Look for convergence using Equation 8.10 from [Hairer, 1996].
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
      // Set the solution using (8.2b) in [Hairer, 1996].
      xtplus->setZero();
      for (int i = 0, j = 0; i < NumStages; ++i, j += state_dim)
        *xtplus += d_[i] * Z_.segment(j, state_dim);
      *xtplus += xt0;

      SPDLOG_DEBUG(drake::log(), "Final state: {}", xtplus->transpose());
      this->set_last_call_succeeded(true);
      return true;
    }

    // Update the norm of the state update.
    last_dx_norm = dx_norm;
  }

  SPDLOG_DEBUG(drake::log(), "StepRadau3() convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!this->get_reuse()) {
    this->set_last_call_succeeded(false);
    return false;
  }

  // Try StepRadau3 again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepRadau3(t0, h, xt0, xtplus, trial+1);
}

// Computes necessary matrices for the Newton-Raphson iteration.
// @param t the time at which to compute the Jacobian.
// @param xt the continuous state at which the Jacobian is computed.
// @param h the integration step size (for computing iteration matrices).
// @param compute_and_factor_iteration_matrix a function pointer for computing
//        and factoring the iteration matrix.
// @param[out] iteration_matrix the updated and factored iteration matrix on
//             return.
// @returns `false` if the calling stepping method should indicate failure;
//          `true` otherwise.
// @post the state in the internal context may or may not be altered on return;
//       if altered, it will be set to (t, xt).
template <class T, int NumStages>
bool Radau3Integrator<T, NumStages>::CalcMatrices(
    const T& t, const VectorX<T>& xt, const T& h,
    const std::function<void(const MatrixX<T>&, const T&,
        typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    int trial) {
  // Compute the initial Jacobian and negated iteration matrices (see
  // rationale for the negation below) and factor them, if necessary.
  MatrixX<T>& J = this->get_mutable_jacobian();
  if (!this->get_reuse() || J.rows() == 0 || this->IsBadJacobian(J)) {
    // Note that the Jacobian can become bad through a divergent Newton-Raphson
    // iteration, which causes the state to overflow, which then causes the
    // Jacobian to overflow. If the state overflows, recomputing the Jacobian
    // using this bad state will result in another bad Jacobian, eventually
    // causing DoStep() to return indicating failure (but not before resetting
    // the continuous state to its previous, good value). DoStep() will then
    // be called again with a smaller step size and the good state; the
    // bad Jacobian will then be corrected.
    J = this->CalcJacobian(t, xt);
    ++num_iter_factorizations_;
    compute_and_factor_iteration_matrix(J, h, iteration_matrix);
    return true;
  } else {
    // Reuse is activated, Jacobian is fully sized, and Jacobian is not "bad".
    // Verify that the iteration matrix has been set and factored.
    if (!iteration_matrix->matrix_factored()) {
      ++num_iter_factorizations_;
      compute_and_factor_iteration_matrix(J, h, iteration_matrix);
      return true;
    }
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing special.
      return true;

    case 2: {
      // For the second trial, re-construct and factor the iteration matrix.
      compute_and_factor_iteration_matrix(J, h, iteration_matrix);
      return true;
    }

    case 3: {
      // If the last call to StepAbstract() ended in failure, we know that
      // the Jacobian matrix is fresh and the iteration matrix has been newly
      // formed and factored (on Trial #2), so there is nothing more to be
      // done.
      if (!this->last_call_succeeded()) {
        return false;
      } else {
        // Reform the Jacobian matrix and refactor the negation of
        // the iteration matrix. The idea of using the negation of this matrix
        // is that an O(n^2) subtraction is not necessary as would
        // be the case with MatrixX<T>::Identity(n, n) - J * (h / scale).
        J = this->CalcJacobian(t, xt);
        ++num_iter_factorizations_;
        compute_and_factor_iteration_matrix(J, h, iteration_matrix);
      }
      return true;

      case 4: {
        // Trial #4 indicates failure.
        return false;
      }

      default:
        throw std::domain_error("Unexpected trial number.");
    }
  }
}

// Steps Radau3 forward by h, if possible.
// @param t0 the initial time.
// @param h the integration step size to attempt.
// @param xt0 the continuous state at time t0.
// @param[out] xtplus_radau3 contains the Radau3 integrator solution on return.
// @returns `true` if the integration was successful at the requested step size
//          and `false` otherwise.
// @pre The time and state in the system's context (stored by the integrator)
//      are set to (t0, xt0) on entry.
// @post The time and state of the system's context (stored by the integrator)
//       will be set to t0+h and `xtplus_radau3` on successful exit (indicated
//       by this function returning `true`) and will be indeterminate on
//       unsuccessful exit (indicated by this function returning `false`).
template <class T, int NumStages>
bool Radau3Integrator<T, NumStages>::AttemptStep(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus_radau3) {
  using std::abs;
  DRAKE_ASSERT(xtplus_radau3);

  // Set the time and state in the context.
  this->get_mutable_context()->SetTimeAndContinuousState(t0, xt0);

  // Compute the derivative at xt0. NOTE: the derivative is calculated at this
  // point (early on in the integration process) in order to reuse the
  // derivative evaluation, via the cache, from the last integration step (if
  // possible).
  const VectorX<T> dx0 = this->EvalTimeDerivativesUsingContext();

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  *xtplus_radau3 = xt0;

  // Do the Radau3 step.
  if (!StepRadau3(t0, h, xt0, xtplus_radau3)) {
    SPDLOG_DEBUG(drake::log(), "Radau3 approach did not converge for "
        "step size {}", h);
    return false;
  }

  return true;
}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful and `false` otherwise.
/// @post the time and continuous state will be advanced only if `true` is
///       returned (if `false` is returned, the time and state will be reset
///       to their values on entry).
template <class T, int NumStages>
bool Radau3Integrator<T, NumStages>::DoStep(const T& h) {
  Context<T>* context = this->get_mutable_context();

  // Save the current time and state.
  const T t0 = context->get_time();
  SPDLOG_DEBUG(drake::log(), "Radau3 DoStep(h={}) t={}", h, t0);

  // TODO(sherm1) Heap allocation here; consider mutable temporaries instead.
  const VectorX<T> xt0 = context->get_continuous_state().CopyToVector();
  VectorX<T> xtplus_radau3(xt0.size()), xtplus_itr(xt0.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using two explicit Euler steps of size h/2. For error estimation, we also
  // take a single step of size h. We can estimate the error in the larger
  // step that way, but note that we propagate the two half-steps on the
  // assumption the result will be better despite not having an error estimate
  // for them (that's called "local extrapolation").
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

// Function for computing the iteration matrix for the Radau3 method. This
// is the matrix in [Hairer, 1996] (8.4) on p.119.
template <class T, int NumStages>
void Radau3Integrator<T, NumStages>::ComputeRadau3IterationMatrix(
    const MatrixX<T>& J,
    const T& h,
    const MatrixX<double>& A,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows() * NumStages;
  // TODO(edrumwri) Investigate how to do the below operation with a move.
  iteration_matrix->SetAndFactorIterationMatrix(
      CalcTensorProduct(A * -h, J) + MatrixX<T>::Identity(n , n));
}

// Computes the tensor product between two matrices. Given
// A = | a11 ... a1m |
//     | ...     ... |
//     | an1 ... anm |
// and some matrix B, the tensor product is is:
// A x B = | a11B ... a1mB |
//         | ...      ...  |
//         | an1B ... anmB |
template <class T, int NumStages>
MatrixX<T> Radau3Integrator<T, NumStages>::CalcTensorProduct(
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
    class ::drake::systems::Radau3Integrator)
