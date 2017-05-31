#pragma once

#include <algorithm>
#include <memory>
#include <limits>

/// @file
/// Template method implementations for half_explicit_DAE1_solver.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

namespace drake {
namespace systems {

// Updates the continuous state in the Context using the saved generalized
// coordinates and velocities and the given constraint forces.
// @param context The pointer to the context to update.
// @param lambda The vector of constraint forces.
// @param dt The integration step size.
template <class T>
void HalfExplicitDAE1Solver<T>::UpdateContinuousState(Context<T>* context,
                                                      const Eigen::VectorXd&
                                                        lambda,
                                                      const Eigen::MatrixXd& J,
                                                      double dt) const {
  const System<T>& system = this->get_system();

  // Compute the effect of applying the constraint forces (as impulsive forces).
  const auto& dv = system.CalcVelocityChangeFromConstraintImpulses(*context, J,
                                                                   lambda);

  // Compute v(t+Δt) and q(t+Δt).
  const auto& vplus = vsave_ + dv;
  system.MapVelocityToQDot(*context, vplus, qdot_.get());
  const auto& qplus = qsave_ + vplus*dt;
  auto x = context->get_mutable_continuous_state();
  x->get_mutable_generalized_position()->SetFromVector(qplus);
  x->get_mutable_generalized_velocity()->SetFromVector(vplus);
}

// Calculates the Jacobian of the algebraic equations in the DAE with respect
// to the given constraint forces.
// @param context The context containing the generalized state variables, with
//                respect to which the Jacobian matrix will be computed.
// @param lambda The vector of constraint forces, which should be of the same
//               dimension (m) as the number of algebraic equations.
// @param dt     The integration step size.
// @returns      A m × m Jacobian matrix of the m algebraic equations
//               differentiated with respect to the m-dimensional constraint
//               force vector.
template <class T>
Eigen::MatrixXd HalfExplicitDAE1Solver<T>::CalcAlgebraicJacobian(
    const Context<T>& context, const Eigen::VectorXd& lambda,
    const Eigen::MatrixXd& Jc, double dt) const {
  const System<T>& system = this->get_system();
  const double sqrt_eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the context as modifiable (we will restore everything we touch).
  Context<T>* context_mut = (Context<T>*) &context;
  const auto& qsave = context.get_continuous_state()->
      get_generalized_position().CopyToVector();
  const auto& vsave = context.get_continuous_state()->
      get_generalized_velocity().CopyToVector();

  // Calculate the error in the algebraic equations at the current lambda.
  UpdateContinuousState(context_mut, lambda, Jc, dt);
  const auto& g0 = system.EvalConstraintEquations(context);

  // Initialize the Jacobian matrix.
  const int neq = g0.size();
  const int nf = lambda.size();
  Eigen::MatrixXd J(neq, nf);

  // Copy lambda.
  Eigen::VectorXd lambda_prime = lambda;

  // Iterate over each dimension of lambda
  for (int i=0; i< nf; ++i) {
    // Compute a good increment to the lambda dimension.
    const double abs_lambda = std::abs(lambda(i));
    double dlambda = (abs_lambda > 0.0) ? sqrt_eps * abs_lambda : sqrt_eps;

    // Minimize effect of roundoff error.
    lambda_prime(i) = lambda(i) + dlambda;
    dlambda = lambda_prime(i) - lambda(i);

    // Compute the column of the Jacobian.
    UpdateContinuousState(context_mut, lambda_prime, Jc, dt);
    const auto& gprime = system.EvalConstraintEquations(context);

    lambda_prime(i) = lambda(i);
    J.col(i) = (gprime - g0)/dlambda;
  }

  // Restore the generalized position and velocity.
  context_mut->get_mutable_continuous_state()->
      get_mutable_generalized_position()->SetFromVector(qsave);
  context_mut->get_mutable_continuous_state()->
      get_mutable_generalized_velocity()->SetFromVector(vsave);

  return J;
}

// Computes the constraint Jacobian (the m × n matrix of partial derivatives of
// the m constraint equations taken with respect to the n generalized velocity
// variables).
// @param context The context containing the generalized state variables, upon
//                which the constraint equations are dependent.
// @returns the constraint Jacobian matrix.
template <class T>
Eigen::MatrixXd HalfExplicitDAE1Solver<T>::CalcConstraintJacobian(
    const Context<T>& context) const {
  const System<T> &system = this->get_system();

  // Get the number of constraint equations.
  const int neq = system.get_num_constraint_equations(context);

  // Store the current generalized velocity.
  const auto& gv_save = context.get_continuous_state()->
      get_generalized_velocity().CopyToVector();

  // Get the context in mutable form.
  Context<T>* context_mut = (Context<T>*) &context;

  // Evaluating the time derivative of the constraint equations is equivalent
  // to multiplying J * v, where J is the constraint Jacobian. By setting the
  // generalized velocity to a sequence of unit vectors and evaluating the
  // time derivative of the constraint equations repeatedly, we compute
  // the constraint Jacobian.
  const int ngv = gv_save.size();
  Eigen::MatrixXd J(neq, ngv);
  auto gv = context_mut->get_mutable_continuous_state()->
      get_mutable_generalized_velocity();
  for (int i=0; i< ngv; ++i) {
    // Set the generalized velocity to a unit vector.
    gv->SetFromVector(Eigen::Matrix<T, Eigen::Dynamic, 1>::Unit(ngv, i));

    // Evaluate the constraint equation and set the corresponding column of
    // the Jacobian matrix.
    J.col(i) = system.EvalConstraintEquationsDot(*context_mut);
  }

  // Restore the generalized velocity.
  context_mut->get_mutable_continuous_state()->
      get_mutable_generalized_velocity()->SetFromVector(gv_save);

  return J;
}

// Calculates the matrix J⋅M⁻¹⋅Jᵀ, where J is the Jacobian matrix of the
// constraint equations (differentiated with respect to the generalized velocity
// variables) and M⁻¹ is the inverse of the generalized inertia matrix.
// @param context The context containing the generalized state variables, for
//                which the constraint space inertia matrix should be computed.
// @param J The m × n matrix of the partial derivatives of the m constraint
//          equations taken with respect to the n generalized velocity
//          variables.
// @returns A m × m matrix, where m is the number of constraint equations.
template <class T>
Eigen::MatrixXd HalfExplicitDAE1Solver<T>::CalcConstraintSpaceInertiaMatrix(
    const systems::Context<T>& context, const Eigen::MatrixXd& J) const {
  const System<T>& system = this->get_system();

  // Create the constraint force vector.
  const int neq = system.get_num_constraint_equations(context);
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(neq);

  // Initialize the matrix.
  Eigen::MatrixXd M(neq, neq);

  // Propagate an impulsive force through the constraints and measure the
  // constraint velocities.
  for (int i=0; i< neq; ++i) {
    // Apply a unit impulse, then zero that component.
    lambda(i) = 1.0;
    Eigen::VectorXd dv = system.CalcVelocityChangeFromConstraintImpulses(
        context, J, lambda);
    lambda(i) = 0.0;

    // Set the appropriate row of the matrix.
    M.row(i) = dv;
  }

  return M;
}

/// Integrates the DAE system forward in time by dt. This value is determined
/// by IntegratorBase::Step().
/// @pre The algebraic equation (g(q(t)) = 0 is true on entry.
template <class T>
void HalfExplicitDAE1Solver<T>::DoStepOnceFixedSize(const T& dt) {
  // Find the continuous state xc within the Context, just once.
  auto context = this->get_mutable_context();
  const auto& xc = context->get_mutable_continuous_state();
  const auto& system = this->get_system();

  // TODO(edrumwri): Change this to a user-settable parameter.
  const double grad_tol = 1e-6;

  // Retrieve the generalized coordinates and velocities and auxiliary
  // variables.
  VectorBase<T>* q = xc->get_mutable_generalized_position();
  VectorBase<T>* v = xc->get_mutable_generalized_velocity();
  VectorBase<T>* z = xc->get_mutable_misc_continuous_state();

  // Resize qdot, if necessary.
  if (qdot_ == nullptr)
    qdot_ = std::make_unique<systems::BasicVector<T>>(q->size());

  // Save the current generalized coordinates (may need to restore these,
  // repeatedly).
  qsave_ = q->CopyToVector();

  // Evaluate the derivatives without accounting for constraint forces.
  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  system.CalcTimeDerivatives(this->get_context(), derivs_.get());

  // Retrieve the accelerations and auxiliary variable derivatives.
  const auto& vdot = derivs_->get_generalized_velocity();
  const auto& zdot = derivs_->get_misc_continuous_state();

  // Update the generalized velocity and auxiliary variables using these
  // accelerations (i.e., accelerations without constraint forces).
  // v'(t+Δt) = v(t) + Δt⋅(f(q(t), v(t))
  // z(t+Δt) = z(t) + Δt⋅dz/dt
  v->PlusEqScaled({ {dt, vdot }});
  z->PlusEqScaled({ {dt, zdot} });

  // Save the updated generalized velocities.
  vsave_ = v->CopyToVector();

  // Compute constraint forces such that J*v(t+dt) = 0, where:
  // v(t+Δt) = v'(t+Δt) + ΔtM⁻¹Jᵀλ
  // where J is the constraint Jacobian and M is the generalized inertia
  // matrix. This first step should give a good starting point for the
  // constraint forces. Given the relationship q(t+Δt) = q(t) + Δt⋅N⋅v(t+Δt),
  // where N(q(t)) is a matrix that converts generalized velocities to the time
  // derivatives of generalized coordinates, the starting point will be perfect
  // if g(q(t)) = 0 (the method precondition) and if g() is a linear function
  // (implying that g(q(t) + Δt⋅v(t+Δt)) = g(q(t)) + Δt⋅N⋅g(v(t+Δt)) = 0 + 0).
  // Such an example occurs when a revolute joint is constrained to be
  // stationary (as if at a stop).

  // Compute JM⁻¹Jᵀ and Jv'(t+Δt).
  const auto& J = CalcConstraintJacobian(*context);
  const auto& JiMJT = CalcConstraintSpaceInertiaMatrix(*context, J);
  Eigen::VectorXd Jvprime = system.EvalConstraintEquationsDot(*context);

  // Solve ΔtJM⁻¹Jᵀλ + Jv'(t+Δt) = 0 for λ.
  // @TODO(edrumwri): Investigate a more clever form of least squares than this
  //                  (Tikhonov regularization), based on the number of
  //                  constraint equations compared to the number of generalized
  //                  velocity variables. We can increase the Jacobian one row
  //                  (constraint) at a time until the number of constraint
  //                  equations is identical to the generalized velocity
  //                  dimension. A constraint will only be added if it allows
  //                  the Cholesky factorization (used already below) to
  //                  succeed. The time complexity for the algorithm just
  //                  described will be limited to O(n³), rather than O(m³),
  //                  where n is the generalized velocity dimension, and m
  //                  is the number of algebraic equations. Just doing a series
  //                  of Cholesky factorizations might prove to be generally
  //                  faster (albeit, expected to be less accurate).
  chol_.compute(JiMJT);
  DRAKE_DEMAND(chol_.info() != Eigen::ComputationInfo::InvalidInput);
  if (chol_.info() != Eigen::ComputationInfo::Success) {
    double alpha = 1e-15;         // Initial regularization factor.
    const double beta = 10.0;     // Regularization factor increase.
    const int n = JiMJT.rows();
    do {
      chol_.compute(JiMJT + drake::MatrixX<T>::Identity(n, n) * alpha);
      alpha *= beta;
    } while (chol_.info() != Eigen::ComputationInfo::Success);
  }
  Eigen::VectorXd lambda = chol_.solve(-Jvprime);

  // Compute the constraint error based on this lambda.
  UpdateContinuousState(context, lambda, J, dt);
  auto err = system.EvalConstraintEquations(*context);

  // Compute the current value of the objective function.
  double fcurrent = err.squaredNorm() / 2;

  // Check for immediate success.
  bool solution_found =
      (system.CalcConstraintErrorNorm(*context, err) < constraint_tol_);

  // If the error is acceptable, use the updated velocity and position.
  if (!solution_found) {
    // Given the starting value for λ, iteratively compute the correct
    // constraint forces such that g(q(t+Δt)) ≈ 0. We do this by iteratively:
    // (1) Computing the error g(.) for a given λ; (2) Determining the Jacobian
    // matrix (J) of the algebraic constraint equation errors computed with
    // respect to the change in the current values of λ; (3) Solving JΔλ =
    // -g(.) for Δλ. (4) Using line search to determine a "good" value of α such
    // that ||g((q(t+Δt))|| = ||g(q(t), v(t), λ+Δαλ)|| is reduced.
    for (int j=0; j< max_nr_iterations_; ++j) {
      // Revert the coordinates and velocity in the context.
      q->SetFromVector(qsave_);
      v->SetFromVector(vsave_);

      // Determine the Jacobian matrix for the current lambda.
      const auto& Jl = CalcAlgebraicJacobian(*context, lambda, J, dt);

      // Solve for Δλ.
      LU_.compute(Jl);
      Eigen::VectorXd dlambda = LU_.solve(-err);

      // Compute the gradient of the objective function with respect to lambda.
      // 1/2*g(lambda)'*g(lambda) = J*g(lambda)
      Eigen::VectorXd grad = Jl * err;

      // Use line search to determine a good value of α.
      LineSearchOutput lout = search_line(lambda, dlambda, fcurrent,
                                          grad, J, dt);

      // Set updated lambda, constraint function outputs, and objective function
      // evaluation.
      lambda = lout.lambda_new;
      err = lout.goutput;
      fcurrent = lout.fnew;

      // Check for solution.
      solution_found =
          (system.CalcConstraintErrorNorm(*context, err) < constraint_tol_);
      if (solution_found)
        break;

      // Check for gradient approximately zero, indicating convergence to a
      // non-solution.
      double test = 0;
      double den = std::max(0.5 * lambda.size(), lout.fnew);
      for (int i=0; i< lambda.size(); ++i)
        test = std::max(test, std::abs(grad(i))*std::max(lambda(i), 1.0)/den);
      if (test < grad_tol)
        break;
    }

    // If the residual error is too large, throw an exception
    // indicating that a smaller step size should be attempted.
    if (!solution_found) {
      // TODO(edrumwri): Throw the exception.
      DRAKE_ABORT_MSG("Non-convergence of Newton-Raphson not yet handled.");
    }
  }

  // Update the time.
  context->set_time(context->get_time() + dt);
  IntegratorBase<T>::UpdateStatistics(dt);
}

/// Searches the line λ+αΔλ for a α that results in a "good" value of the
/// objective function.
/// @param lambda The current value of λ.
/// @param dlambda The computed descent direction for λ (i.e., Δλ).
/// @param fold The old objective function value (determined by computing the
///             Euclidean norm of the vector output of the algebraic constraint
///             equations and then scaling by 1/2).
/// @param gradient The gradient of the objective function (i.e., the gradient
///                 of the Euclidean norm of the vector output of the algebraic
///                 constraint equations and then scaling by 1/2).
/// @returns a structure containing the new value of λ, the new value of the
///          objective function, and the new outputs of the algebraic constraint
///          equations.
/// @note This routine is an implementation of the line search algorithm
///       described in "Numerical Recipes in C" (Section 9.7). The routine
///       does not attempt to optimize α, as common wisdom indicates that
///       approach to be computationally inefficient. Instead, this algorithm
///       seeks the α for which (1) the average decrease of the objective (f)
///       is at least some fraction of the initial rate of decrease Δλ⋅∇, where
///       ∇ is @p gradient- this ensures that f does not decrease
///       too slowly relative to the step lengths- and (2) the rate of decrease
///       in the objective at λ+αΔλ is greater than a fraction of the rate of
///       decrease in f at λ (addressing the problem where the
///       step lengths are too small relative to the initial rate of decrease
///       of f). This algorithm attempts to fit a quadratic (for the first
///       backtracking attempt) or cubic (for subsequent backtracking attempts)
///       to find a good value for α.
template <class T>
typename HalfExplicitDAE1Solver<T>::LineSearchOutput
      HalfExplicitDAE1Solver<T>::search_line(
          const Eigen::VectorXd& lambda, const Eigen::VectorXd& dlambda,
          double fold, const Eigen::VectorXd& gradient,
          const Eigen::MatrixXd& J, double dt) const {
  const auto& system = this->get_system();
  const Context<T>& context = this->get_context();
  Context<T>* context_mut = (Context<T>*) &context;

  const double meps = std::numeric_limits<double>::epsilon();
  const double gamma = 1e-4;               // Rate of decrease constant.
  const double deltal_tol =  meps*100;     // Convergence criterion for Δλ.

  // The initial step length scalar- we always try a full Newton step first.
  double alpha = 1.0;

  // Get the slope of the step.
  double slope = gradient.dot(dlambda);

  // Compute the new lambda.
  LineSearchOutput out;

  // Set last alpha to NaN both to indicate that this is the first time the
  // loop is being run and to help with debugging in case one or more values
  // is unexpectedly not assigned.
  double last_alpha = std::nan("");
  double last_f = std::nan("");

  // Determine minimum possible value of alpha.
  double test = 0.0;
  for (int i=0; i< dlambda.size(); ++i) {
    test = std::max(test,
                    std::abs(dlambda(i)) / std::max(std::abs(lambda(i)), 1.0));
  }
  const double alpha_min = deltal_tol / test;

  while (true) {
    // Update lambda.
    out.lambda_new = lambda + dlambda*alpha;

    // Compute the new value of the objective function.
    UpdateContinuousState(context_mut, out.lambda_new, J, dt);
    out.goutput = system.EvalConstraintEquations(context);
    out.fnew = out.goutput.squaredNorm() / 2;

    // Look for minimum alpha.
    if (alpha < alpha_min) {
      break;
    } else {
      // Look for sufficient decrease.
      if (out.fnew <= fold+gamma*alpha*slope) {
        break;
      } else {
        double tmp_beta;
        if (std::isnan(last_alpha)) {
          tmp_beta = -slope / (2 * (out.fnew - fold - slope));
        } else {
          const double rhs1 = out.fnew - fold - alpha*slope;
          const double rhs2 = last_f - fold - last_alpha * slope;
          const double alpha2 = alpha*alpha;
          const double last_alpha2 = last_alpha*last_alpha;
          const double a = (rhs1/alpha2 - rhs2/last_alpha2)/(alpha-last_alpha);
          const double b = (-last_alpha*rhs1/alpha2 + alpha*rhs2/last_alpha2)/
              (alpha-last_alpha);
          if (a == 0.0) {
            tmp_beta = -slope / (2 * b);
          } else {
            const double disc = b*b - 3*a*slope;
            if (disc < 0.0) {
              tmp_beta = 0.5*alpha;
            } else {
              if (b <= 0.0)
                tmp_beta = (-b+std::sqrt(disc))/(3*a);
              else
                tmp_beta = -slope/(b+std::sqrt(disc));
            }

            if (tmp_beta > alpha/2)
              tmp_beta  = alpha/2;
          }
        }

        last_alpha = alpha;
        last_f = out.fnew;
        alpha = std::max(tmp_beta, 0.1*alpha);
      }
    }
  }

  return out;
}

}  // namespace systems
}  // namespace drake

