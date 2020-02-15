#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

namespace internal {
#ifndef DRAKE_DOXYGEN_CXX
__attribute__((noreturn)) inline void EmitNoErrorEstimatorStatAndMessage() {
  throw std::logic_error(
      "No error estimator is currently implemented, so "
      "query error estimator statistics is not yet supported.");
}
#endif
}  // namespace internal

/**
 * A first-order, fully implicit integrator optimized for second-order systems.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * The velocity-implicit Euler integrator is a variant of the first-order
 * implicit Euler that takes advantage of the simple mapping q̇ = N(q) v
 * of second order systems to formulate a smaller problem in velocities (and
 * miscellaneous states if any) only. For second-order systems,
 * %VelocityImplicitEulerIntegrator formulates a problem that is half as large
 * as that formulated by Drake's ImplicitEulerIntegrator, resulting in improved
 * run-time performance. Upon convergence of the resulting system of equations,
 * this method provides the same discretization as ImplicitEulerIntegrator, but
 * at a fraction of the computational cost.
 *
 * This integrator requires a system of ordinary differential equations in
 * state `x = (q,v,z)` to be expressible in the following form:
 *
 *     q̇ = N(q) v;                            (1)
 *     ẏ = f_y(t,q,y),                        (2)
 * where `q̇` and `v` are linearly related via the kinematic mapping `N(q)`, 
 * `y = (v,z)`, and `f_y` is a function that can depend on the time and state.
 *
 * Implicit Euler uses the following update rule at time step n:
 *
 *     qⁿ⁺¹ = qⁿ + h N(qⁿ⁺¹) vⁿ⁺¹;            (3)
 *     yⁿ⁺¹ = yⁿ + h f_y(tⁿ⁺¹,qⁿ⁺¹,yⁿ⁺¹).     (4)
 *
 * To solve the nonlinear system for `(qⁿ⁺¹,yⁿ⁺¹)`, the velocity-implicit Euler
 * integrator iterates with a modified Newton's method: At iteration `k`, it 
 * finds a `(qₖ₊₁,yₖ₊₁)` that attempts to satisfy
 *
 *     qₖ₊₁ = qⁿ + h N(qₖ) vₖ₊₁.              (5)
 *     yₖ₊₁ = yⁿ + h f_y(tⁿ⁺¹,qₖ₊₁,yₖ₊₁);     (6)
 *
 * In this notation, the `n`'s index timesteps, while the `k`'s index the
 * specific Newton-Raphson iterations within each time step.
 * 
 * Notice that we've intentionally lagged N(qₖ) one iteration behind in Eq (5).
 * This allows it to substitute (5) into (6) to obtain a non-linear system in
 * `y` only. Contrast this strategy with the one implemented by
 * ImplicitEulerIntegrator, which solves a larger non-linear system in the full
 * state x.
 *
 * To find a `(qₖ₊₁,yₖ₊₁)` that approximately satisfies (5-6), we linearize
 * the system (5-6) to compute a Newton step. Define
 *
 *     l(y) = f_y(tⁿ⁺¹,qⁿ + h N(qₖ) v,y),     (7)
 *     Jₗ(y) = ∂l(y) / ∂y.                    (8)
 *
 * To advance the Newton step, the velocity-implicit Euler integrator solves
 * the following linear equation for `Δy`:
 *
 *     (I - h Jₗ) Δy = - R(yₖ),               (9)
 * where `R(y) = y - yⁿ - h l(y)` and `Δy = yₖ₊₁ - yₖ`. The `Δy` solution
 * directly gives us `yₖ₊₁`. It then substitutes the `vₖ₊₁` component of `yₖ₊₁`
 * in (5) to get `qₖ₊₁`.
 *
 * This implementation uses a Newton method and relies upon the obvious
 * convergence to a solution for `y` in `R(y) = 0` where
 * `R(y) = y - yⁿ - h l(y)` as `h` becomes sufficiently small.
 * General implementational details were gleaned from [Hairer, 1996].
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996, Section IV.8, p. 118–130.
 *
 * @note This integrator uses the integrator accuracy setting, even when run
 *       in fixed-step mode, to limit the error in the underlying Newton-Raphson
 *       process. See IntegratorBase::set_target_accuracy() for more info.
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
 * @see ImplicitEulerIntegrator class documentation for information about
 *      the "implicit Euler" integration method.
 */
template <class T>
class VelocityImplicitEulerIntegrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityImplicitEulerIntegrator)

  ~VelocityImplicitEulerIntegrator() override = default;

  explicit VelocityImplicitEulerIntegrator(const System<T>& system,
                                           Context<T>* context = nullptr)
      : ImplicitIntegrator<T>(system, context) {}

  /// The integrator does not support error estimation.
  bool supports_error_estimation() const final { return false; }

  /// Returns 0 for the error estimation order because this integrator does not
  /// support error estimation.
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

  int64_t do_get_num_error_estimator_newton_raphson_iterations() const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  void DoResetImplicitIntegratorStatistics() final;

  static void ComputeAndFactorImplicitEulerIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  void DoInitialize() final;

  bool DoImplicitIntegratorStep(const T& h) final;

  // Steps the system forward by a single step of h using the Velocity-Implicit
  // Euler method.
  // @param t0 the time at the left end of the integration interval.
  // @param h the time increment to step forward.
  // @param xn the continuous state at t0, which is xⁿ.
  // @param xtplus_guess the starting guess for xⁿ⁺¹.
  // @param [out] xtplus the computed value for xⁿ⁺¹ on successful return.
  // @param [in, out] iteration_matrix the cached iteration matrix, which is
  //        updated if get_use_full_newton() is true, if get_reuse() is false,
  //        or if the Newton-Raphson fails to converge on the first try.
  // @param [in, out] Jy the cached Jacobian Jₗ(y), which is updated if
  //        get_use_full_newton() is true, if get_reuse() is false, or if the
  //        Newton-Raphson fails to converge on the second try.
  // @param trial the attempt for this approach (1-4).
  //        StepVelocityImplicitEuler() uses increasingly computationally
  //        expensive methods as the trial numbers increase.
  // @returns `true` if the step of size `h` was successful, `false` otherwise.
  // @note The time and continuous state in the context are indeterminate upon
  //       exit.
  bool StepVelocityImplicitEuler(
      const T& t0, const T& h, const VectorX<T>& xn,
      const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jy, int trial = 1);

  // Compute the partial derivatives of the ordinary differential equations with
  // respect to the y variables of a given x(t). In particular, we compute the
  // Jacobian, Jₗ(y), of the function l(y), used in this integrator's
  // residual computation, with respect to y, where y = (v,z) and x = (q,v,z).
  // This Jacobian is then defined as:
  //     l(y)  = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)   (7)
  //     Jₗ(y) = ∂l(y)/∂y                       (8)
  // @param t refers to tⁿ⁺¹, the time used in the definition of l(y)
  // @param h is the timestep size parameter, h, used in the definition of
  //        l(y)
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn refers to qⁿ, the initial position used in l(y)
  // @param [out] Jy is the Jacobian matrix, Jₗ(y).
  // @post The context's time will be set to t, and its continuous state will
  //       be indeterminate on return.
  void CalcVelocityJacobian(const T& t, const T& h, const VectorX<T>& y,
                            const VectorX<T>& qk, const VectorX<T>& qn,
                            MatrixX<T>* Jy);

  // Uses first-order forward differencing to compute the Jacobian, Jₗ(y), of
  // the function l(y), used in this integrator's residual computation, with
  // respect to y, where y = (v,z). This Jacobian is then defined as:
  //     l(y)  = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)   (7)
  //     Jₗ(y) = ∂l(y)/∂y                       (8)
  //
  // In this method, we compute the Jacobian Jₗ(y) using a first-order forward
  // difference (i.e. numerical differentiation),
  //     Jₗ(y)ᵢⱼ = (l(y')ᵢ - l(y)ᵢ )/ δy(j),
  // where y' = y + δy(j) eⱼ, δy(j) = (√ε) max(1,|yⱼ|), and eⱼ is the j-th
  // standard Cartesian basis vector.
  // In the code we hereby refer to y as "the baseline" and y' as "prime".
  // @param t refers to tⁿ⁺¹, the time used in the definition of l(y).
  // @param h is the timestep size parameter, h, used in the definition of
  //        l(y).
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn refers to qⁿ, the initial position used in l(y).
  // @param [out] Jy is the Jacobian matrix, Jₗ(y).
  // @note The context's time will be set to t, and its continuous state will
  //       be indeterminate on return.
  void ComputeForwardDiffVelocityJacobian(const T& t, const T& h,
                                          const VectorX<T>& y,
                                          const VectorX<T>& qk,
                                          const VectorX<T>& qn,
                                          MatrixX<T>* Jy);

  // Computes necessary matrices (Jacobian and iteration matrix) for
  // Newton-Raphson (NR) iterations, as necessary. This method is based off of
  // ImplicitIntegrator<T>::MaybeFreshenMatrices. We implement our own version
  // here to use a specialized Jacobian Jₗ(y). The aformentioned method was
  // designed for use in DoImplicitIntegratorStep() processes that follow this
  // model:
  // 1. DoImplicitIntegratorStep(h) is called;
  // 2. One or more NR iterations is performed until either (a) convergence is
  //    identified, (b) the iteration is found to diverge, or (c) too many
  //    iterations were taken. In the case of (a), DoImplicitIntegratorStep(h)
  //    will return success. Otherwise, the Newton-Raphson process is attempted
  //    again with (i) a recomputed and refactored iteration matrix and (ii) a
  //    recomputed Jacobian and a recomputed an refactored iteration matrix, in
  //    that order. The process stage of that NR algorithm is indicated by the
  //    `trial` parameter below. In this model, DoImplicitIntegratorStep()
  //    returns failure if the NR iterations reach a fourth trial.
  //
  // We provide our own method to execute the same logic, but with the
  // following differences:
  // 1. We use the specialized Jacobian Jₗ(y) instead of the full Jacobian.
  // 2. We no longer use the get_reuse() logic to reuse a Jacobian
  //    when the time-step size (h) shrinks, because the specialized Jacobian
  //    Jₗ(y) depends on h.
  // These changes allow the velocity-implicit Euler method to use the smaller
  // specialized Jacobian Jₗ(y) in its Newton solves.
  //
  // @param t is the time at which to compute the Jacobian.
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y), which is used in the definition of Jₗ(y).
  // @param qn is the generalized position at the beginning of the step.
  // @param h is the integration step size.
  // @param trial specifies which trial (1-4) the Newton-Raphson process is in
  //        when calling this method.
  // @param compute_and_factor_iteration_matrix is a function pointer for
  //        computing and factoring the iteration matrix.
  // @param [in, out] iteration_matrix is the updated and factored iteration
  //        matrix on return.
  // @param [in, out] Jy is the updated and factored Jacobian matrix Jₗ(y) on
  //        return.
  // @returns `false` if the calling stepping method should indicate failure;
  //          `true` otherwise.
  // @pre 1 <= `trial` <= 4.
  // @post The internal context may or may not be altered on return; if
  //       altered, the time will be set to t and the continuous state will be
  //       indeterminate.
  bool MaybeFreshenVelocityMatrices(
      const T& t, const VectorX<T>& y, const VectorX<T>& qk,
      const VectorX<T>& qn, const T& h, int trial,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jy);

  // Computes necessary matrices (Jacobian and iteration matrix) for full
  // Newton-Raphson (NR) iterations, if full Newton-Raphson method is activated
  // (if it's not activated, this method is a no-op).
  // @param t the time at which to compute the Jacobian.
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate Jₗ(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y), which is used in the definition of Jₗ(y).
  // @param qn is qⁿ, the generalized position at the beginning of the step.
  // @param h the integration step size (for computing iteration matrices).
  // @param compute_and_factor_iteration_matrix a function pointer for
  //        computing and factoring the iteration matrix.
  // @param[out] iteration_matrix the updated and factored iteration matrix on
  //             return.
  // @param[out] Jy the updated Jacobian matrix Jₗ(y).
  // @post The internal context may or may not be altered on return; if
  //       altered, the time will be set to t and the continuous state will be
  //       indeterminate.
  void FreshenVelocityMatricesIfFullNewton(
      const T& t, const VectorX<T>& y, const VectorX<T>& qk,
      const VectorX<T>& qn, const T& h,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jy);

  // This helper method evaluates the Newton-Raphson residual R(y), defined as
  // the following:
  //     R(y)  = y - yⁿ - h l(y),
  //     l(y) = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),    (7)
  // with tⁿ⁺¹, y = (v, z), qₖ, qⁿ, yⁿ, and h passed in.
  // @param t refers to tⁿ⁺¹, the time at which to compute the residual R(y).
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate R(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn is qⁿ, the generalized position at the beginning of the step.
  // @param yn is yⁿ, the generalized velocity and miscellaneous states at the
  //        beginning of the step
  // @param h is the step size.
  // @param [in, out] qdot is a temporary BasicVector<T> of the same size as qⁿ
  //        allocated by the caller so that this method avoids unnecessary heap
  //        allocations. Its value is indeterminate upon return.
  // @param [out] result is set to R(y).
  // @post The context is set to (tⁿ⁺¹, qⁿ + h N(qₖ) v, y).
  VectorX<T> ComputeResidualR(const T& t, const VectorX<T>& y,
                              const VectorX<T>& qk, const VectorX<T>& qn,
                              const VectorX<T>& yn, const T& h,
                              BasicVector<T>* qdot);

  // This helper method evaluates l(y), defined as the following:
  //     l(y) = f_y(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),    (7)
  // with tⁿ⁺¹, y = (v, z), qₖ, qⁿ, yⁿ, and h passed in.
  // @param t refers to tⁿ⁺¹, the time at which to compute the residual R(y).
  // @param y is the generalized velocity and miscellaneous states around which
  //        to evaluate l(y).
  // @param qk is qₖ, the current-iteration position used in the definition of
  //        l(y).
  // @param qn is qⁿ, the generalized position at the beginning of the step.
  // @param h is the step size.
  // @param [in, out] qdot is a temporary BasicVector<T> of the same size as qⁿ
  //        allocated by the caller so that this method avoids unnecessary heap
  //        allocations. Its value is indeterminate upon return.
  // @param [out] result is set to l(y).
  // @post The context is set to (tⁿ⁺¹, qⁿ + h N(qₖ) v, y).
  VectorX<T> ComputeLOfY(const T& t, const VectorX<T>& y, const VectorX<T>& qk,
                         const VectorX<T>& qn, const T& h,
                         BasicVector<T>* qdot);

  // The last computed iteration matrix and factorization.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_ie_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Variables to avoid heap allocations.
  VectorX<T> xn_, xdot_, xtplus_ie_;

  // Various statistics.
  int64_t num_nr_iterations_{0};

  // The last computed velocity+misc Jacobian matrix.
  MatrixX<T> Jy_ie_;
};

// TODO(antequ): The method implementations should be moved to the CC file.
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

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::VelocityImplicitEulerIntegrator)

