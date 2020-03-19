#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <class T>
void VelocityImplicitEulerIntegrator<T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::DoInitialize() {
  using std::isnan;

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  // Verify that the maximum step size has been set.
  if (isnan(this->get_maximum_step_size()))
    throw std::logic_error("Maximum step size has not been set!");

  // Reset the Jacobian matrix (so that recomputation is forced).
  this->Jy_ie_.resize(0, 0);

  // TODO(antequ): Change this to the recommended default accuracy after error
  // control is implemented.
  // Set an initial working accuracy so that the integrator doesn't take too
  // long.
  if (isnan(this->get_accuracy_in_use()))
    this->set_accuracy_in_use(1e-6);
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::
    ComputeAndFactorImplicitEulerIterationMatrix(
        const MatrixX<T>& J, const T& h,
        typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows();
  // TODO(edrumwri) Investigate using a move-type operation below.
  // We form the iteration matrix in this particular way to avoid an O(n^2)
  // subtraction as would be the case with:
  // MatrixX<T>::Identity(n, n) - h * J.
  iteration_matrix->SetAndFactorIterationMatrix(-h * J  +
                                                MatrixX<T>::Identity(n, n));
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::ComputeForwardDiffVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& y, const VectorX<T>& qk,
    const VectorX<T>& qn, MatrixX<T>* Jy) {
  DRAKE_LOGGER_DEBUG(
      "VelocityImplicitEulerIntegrator ComputeForwardDiffVelocityJacobian "
      "{}-Jacobian t={}",
      y.size(), t);
  DRAKE_LOGGER_DEBUG("  computing from qk {}, y {}", qk.transpose(),
                     y.transpose());

  BasicVector<T> qdot(qn.size());
  // Define the lambda l_of_y to evaluate l(y).
  std::function<void(const VectorX<T>&, VectorX<T>*)> l_of_y =
      [&qk, &t, &qn, &h, &qdot, this](const VectorX<T>& y_state,
                               VectorX<T>* l_result) {
        *l_result = this->ComputeLOfY(t, y_state, qk, qn, h, &qdot);
      };

  // Compute Jy by passing l(y) to math::ComputeNumericalGradient.
  // TODO(antequ): Right now we modify the context twice each time we call
  // l(y): once when we calculate qⁿ + h N(qₖ) v (SetTimeAndContinuousState),
  // and once when we calculate l(y) (get_mutable_generalized_position).
  // However, this is only necessary for each y that modifies a velocity (v).
  // For all but one of the miscellaneous states (z), we can reuse the position
  // so that the context needs only one modification. Investigate how to
  // refactor this logic to achieve this performance benefit while maintaining
  // code readability.
  *Jy = math::ComputeNumericalGradient(l_of_y, y,
      math::NumericalGradientOption{math::NumericalGradientMethod::kForward});
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::CalcVelocityJacobian(const T& t,
    const T& h, const VectorX<T>& y, const VectorX<T>& qk,
    const VectorX<T>& qn, MatrixX<T>* Jy) {
  // Note: Unlike ImplicitIntegrator<T>::CalcJacobian, we neither save the
  // context or change it back, because our implementation of
  // StepVelocityImplicitEuler does not require the context to be restored.
  this->increment_jacobian_evaluations();

  // Get the existing number of ODE evaluations.
  int64_t existing_ODE_evals = this->get_num_derivative_evaluations();

  // Compute the Jacobian using the selected computation scheme.
  switch (this->get_jacobian_computation_scheme()) {
    case ImplicitIntegrator<T>::JacobianComputationScheme::kForwardDifference:
      ComputeForwardDiffVelocityJacobian(t, h, y, qk, qn, Jy);
      break;
    case ImplicitIntegrator<T>::JacobianComputationScheme::kCentralDifference:
      throw std::runtime_error("Central difference not supported yet!");
      break;
    case ImplicitIntegrator<T>::JacobianComputationScheme::kAutomatic:
      throw std::runtime_error("AutoDiff'd Jacobian not supported yet!");
      break;
    default:
      throw new std::logic_error("Invalid Jacobian computation scheme!");
  }

  // Use the new number of ODE evaluations to determine the number of ODE
  // evaluations used in computing Jacobians.
  this->increment_jacobian_computation_derivative_evaluations(
      this->get_num_derivative_evaluations() - existing_ODE_evals);
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::MaybeFreshenVelocityMatrices(
    const T& t, const VectorX<T>& y, const VectorX<T>& qk,
    const VectorX<T>& qn, const T& h, int trial,
    const std::function<void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jy) {
  DRAKE_DEMAND(Jy != nullptr);
  DRAKE_DEMAND(iteration_matrix != nullptr);
  // Compute the initial Jacobian and iteration matrices and factor them, if
  // necessary.
  if (!this->get_reuse() || Jy->rows() == 0 || this->IsBadJacobian(*Jy)) {
    CalcVelocityJacobian(t, h, y, qk, qn, Jy);
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jy, h, iteration_matrix);
    return true;  // Indicate success.
  }

  // Reuse is activated, Jacobian is fully sized, and Jacobian is not "bad".
  // If the iteration matrix has not been set and factored, do only that.
  if (!iteration_matrix->matrix_factored()) {
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jy, h, iteration_matrix);
    return true;  // Indicate success.
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing: this will cause the Newton-Raphson
      // process to use the last computed (and already factored) iteration
      // matrix. This matrix may be from a previous time-step or a previously-
      // attempted step size.
      return true;  // Indicate success.

    case 2: {
      // For the second trial, we know the first trial, which uses the last
      // computed iteration matrix, has already failed. We perform the (likely)
      // next least expensive operation, which is re-constructing and factoring
      // the iteration matrix, using the last computed Jacobian. The last
      // computed Jacobian may be from a previous time-step or a previously-
      // attempted step size.
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jy, h, iteration_matrix);
      return true;
    }

    case 3: {
      // For the third trial, we know that the first two trials, which
      // exhausted all our options short of recomputing the Jacobian, have
      // failed. We recompute the Jacobian matrix and refactor the iteration
      // matrix.

      // Note: Based on a few simple experimental tests, we found that the
      // optimization to abort this trial when matrices are already fresh in
      // ImplicitIntegrator<T>::MaybeFreshenMatrices does not significantly help
      // here, especially because our Jacobian depends on step size h.
      CalcVelocityJacobian(t, h, y, qk, qn, Jy);
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jy, h, iteration_matrix);
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

template <class T>
void VelocityImplicitEulerIntegrator<T>::FreshenVelocityMatricesIfFullNewton(
    const T& t, const VectorX<T>& y, const VectorX<T>& qk,
    const VectorX<T>& qn, const T& h,
    const std::function<void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jy) {
  DRAKE_DEMAND(iteration_matrix != nullptr);
  DRAKE_DEMAND(Jy != nullptr);

  // Return immediately if full-Newton is not in use.
  if (!this->get_use_full_newton()) return;

  // Compute the initial Jacobian and iteration matrices and factor them.
  CalcVelocityJacobian(t, h, y, qk, qn, Jy);
  this->increment_num_iter_factorizations();
  compute_and_factor_iteration_matrix(*Jy, h, iteration_matrix);
}

template <class T>
VectorX<T> VelocityImplicitEulerIntegrator<T>::ComputeResidualR(
    const T& t, const VectorX<T>& y, const VectorX<T>& qk, const VectorX<T>& qn,
    const VectorX<T>& yn, const T& h, BasicVector<T>* qdot) {
  // Compute l(y), which also sets the time and y states of the context.
  const VectorX<T> l_of_y = ComputeLOfY(t, y, qk, qn, h, qdot);

  // Evaluate R(y).
  return y - yn - h * l_of_y;
}


template <class T>
VectorX<T> VelocityImplicitEulerIntegrator<T>::ComputeLOfY(const T& t,
    const VectorX<T>& y, const VectorX<T>& qk, const VectorX<T>& qn,
    const T& h, BasicVector<T>* qdot) {
  Context<T>* context = this->get_mutable_context();
  int nq = qn.size();
  int ny = y.size();

  // Set the context to (t, qₖ, y)
  // TODO(antequ): Optimize this procedure to both (1) remove unnecessary heap
  // allocations, like in the VectorX<T> constructions of x and q and the return
  // statement, and (2) reduce unnecessary cache invalidations since
  // MapVelocityToQDot() doesn't set any caches.
  VectorX<T> x(nq+ny);
  x.head(nq) = qk;
  x.tail(ny) = y;
  context->SetTimeAndContinuousState(t, x);

  // Compute q = qⁿ + h N(qₖ) v.
  this->get_system().MapVelocityToQDot(*context,
      context->get_continuous_state().get_generalized_velocity(), &*qdot);
  const VectorX<T> q = qn + h * qdot->get_value();

  // Evaluate l = f_y(t, q, v, z).
  // TODO(antequ): Right now this invalidates the entire cache that depends on
  // any of the continuous state. Investigate invalidating less of the cache
  // once we have a Context method for modifying just the generalized position.
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q);
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(*context);
  return xc_deriv.CopyToVector().tail(ny);
}


template <class T>
bool VelocityImplicitEulerIntegrator<T>::StepVelocityImplicitEuler(
    const T& t0, const T& h, const VectorX<T>& xn,
    const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jy, int trial) {
  using std::abs;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);
  DRAKE_LOGGER_DEBUG(
      "VelocityImplicitEulerIntegrator::StepVelocityImplicitEuler(h={}) t={}",
      h, t0);

  const System<T>& system = this->get_system();
  // Verify xtplus
  DRAKE_ASSERT(xtplus != nullptr && xtplus->size() == xn.size() &&
               xtplus_guess.size() == xn.size());

  // Initialize xtplus to the guess
  *xtplus = xtplus_guess;

  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  int nq = cstate.num_q();
  int nv = cstate.num_v();
  int nz = cstate.num_z();
  const Eigen::VectorBlock<const VectorX<T>> qn = xn.head(nq);
  const Eigen::VectorBlock<const VectorX<T>> yn = xn.tail(nv + nz);

  // Define references to q, y, v, and z portions of xtplus for readability.
  Eigen::VectorBlock<VectorX<T>> qtplus = xtplus->head(nq);
  Eigen::VectorBlock<VectorX<T>> ytplus = xtplus->tail(nv + nz);
  const Eigen::VectorBlock<VectorX<T>> vtplus = xtplus->segment(nq, nv);
  const Eigen::VectorBlock<VectorX<T>> ztplus = xtplus->tail(nz);
  unused(ztplus);

  // Set last_qtplus to qk. This will be used in computing dx to determine
  // convergence.
  VectorX<T> last_qtplus = qtplus;

  // Initialize the vector for qdot.
  BasicVector<T> qdot(nq);

  // We compute our residuals at tf = t0 + h.
  const T tf = t0 + h;

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  VectorX<T> dx(xn.size());
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Calculate Jacobian and iteration matrices (and factorizations), as needed,
  // around (t0, xn). We do not do this calculation if full Newton is in use;
  // the calculation will be performed at the beginning of the loop instead.
  if (!this->get_use_full_newton() &&
      !this->MaybeFreshenVelocityMatrices(t0, yn, qn, qn, h, trial,
      ComputeAndFactorImplicitEulerIterationMatrix, iteration_matrix, Jy)) {
    return false;
  }

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    DRAKE_LOGGER_DEBUG(
        "VelocityImplicitEulerIntegrator::StepVelocityImplicitEuler() entered "
        "for t={}, h={}, trial={}", t0, h, trial);

    this->FreshenVelocityMatricesIfFullNewton(
        tf, ytplus, qtplus, qn, h,
        ComputeAndFactorImplicitEulerIterationMatrix, iteration_matrix, Jy);

    // Update the number of Newton-Raphson iterations.
    ++num_nr_iterations_;

    // Evaluate the residual error, which is defined above as R(yₖ):
    //     R(yₖ) = yₖ - yⁿ - h l(yₖ).
    VectorX<T> residual = ComputeResidualR(tf, ytplus, qtplus, qn,
                                           yn, h, &qdot);

    // Compute the state update using the equation A*y = -R(), where A is the
    // iteration matrix.
    const VectorX<T> dy = iteration_matrix->Solve(-residual);

    // Update the y portion of xtplus to yₖ₊₁.
    ytplus += dy;

    // Update the q portion of xtplus to qₖ₊₁ = qⁿ + h N(qₖ) vₖ₊₁. Note that
    // currently, qtplus is set to qₖ, while vtplus is set to vₖ₊₁.
    // TODO(antequ): Optimize this so that the context doesn't invalidate the
    // position state cache an unnecessary number of times, because evaluating
    // N(q) does not set any cache.
    // TODO(antequ): Right now this invalidates the entire cache that depends
    // on any of the continuous state. Investigate invalidating less of the
    // cache once we have a Context method for modifying just the generalized
    // position.
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetFromVector(qtplus);
    system.MapVelocityToQDot(*context, vtplus, &qdot);
    qtplus = qn + h * qdot.get_value();
    dx << qtplus - last_qtplus, dy;

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);

    // TODO(antequ): Replace this with CalcStateChangeNorm() when error
    // control has been implemented.
    // Get the norm of the update vector.
    T dx_norm = dx_state_->CopyToVector().norm();

    // Check for Newton-Raphson convergence.
    typename ImplicitIntegrator<T>::ConvergenceStatus status =
        this->CheckNewtonConvergence(i, *xtplus, dx, dx_norm, last_dx_norm);

    // If it converged, we're done.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kConverged)
      return true;
    // If it diverged, we have to abort and try again.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kDiverged)
      break;
    // Otherwise, continue to the next Newton-Raphson iteration.
    DRAKE_DEMAND(status ==
                 ImplicitIntegrator<T>::ConvergenceStatus::kNotConverged);

    last_dx_norm = dx_norm;
    last_qtplus = qtplus;
  }

  DRAKE_LOGGER_DEBUG("Velocity-Implicit Euler integrator convergence failed"
                     "for t={}, h={}, trial={}", t0, h, trial);

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try; otherwise, the following code will recurse
  // into this function again, and freshen computations as helpful. Note that
  // get_reuse() returns false if "full Newton-Raphson" mode is activated (see
  // ImplicitIntegrator::get_use_full_newton()).
  if (!this->get_reuse()) return false;

  // Try StepVelocityImplicitEuler again. This method will
  // freshen Jacobians and iteration matrix factorizations as necessary.
  return StepVelocityImplicitEuler(t0, h, xn, xtplus_guess, xtplus,
                                   iteration_matrix, Jy, trial + 1);
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::DoImplicitIntegratorStep(const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  DRAKE_LOGGER_DEBUG("VelocityImplicitEulerIntegrator::"
      "DoImplicitIntegratorStep(h={}) t={}", h, t0);

  xn_ = context->get_continuous_state().CopyToVector();
  xtplus_ie_.resize(xn_.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Euler step.
  if (h < this->get_working_minimum_step_size()) {
    DRAKE_LOGGER_DEBUG(
        "-- requested step too small, taking explicit step instead, at t={}, "
        "h={}, minimum_h={}", t0, h, this->get_working_minimum_step_size());

    // Compute the explicit Euler step.
    xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_ie_ = xn_ + h * xdot_;
  } else {
    // Use the current state as the candidate value for the next state.
    // [Hairer 1996] validates this choice (p. 120).
    const VectorX<T>& xtplus_guess = xn_;
    const bool success = StepVelocityImplicitEuler(t0, h, xn_, xtplus_guess,
        &xtplus_ie_, &iteration_matrix_ie_, &Jy_ie_);

    // If the step was not successful, reset the time and state.
    if (!success) {
      DRAKE_LOGGER_DEBUG(
          "Velocity-Implicit Euler approach did not converge for "
          "time t={}, step size h={}", t0, h);
      context->SetTimeAndContinuousState(t0, xn_);
      return false;
    }
  }

  // Set the state to the computed state.
  context->SetTimeAndContinuousState(t0 + h, xtplus_ie_);

  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::VelocityImplicitEulerIntegrator)
