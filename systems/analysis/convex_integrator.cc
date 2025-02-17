#include "drake/systems/analysis/convex_integrator.h"

#include "drake/multibody/contact_solvers/newton_with_bisection.h"
#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/geometry_contact_data.h"

namespace drake {
namespace systems {

using Eigen::VectorXd;
using geometry::PenetrationAsPointPair;
using multibody::DiscreteContactApproximation;
using multibody::Frame;
using multibody::RigidBody;
using multibody::contact_solvers::internal::Bracket;
using multibody::contact_solvers::internal::DoNewtonWithBisectionFallback;
using multibody::contact_solvers::internal::MakeContactConfiguration;
using multibody::contact_solvers::internal::MatrixBlock;
using multibody::contact_solvers::internal::SapConstraintJacobian;
using multibody::contact_solvers::internal::SapHessianFactorizationType;
using multibody::contact_solvers::internal::SapHuntCrossleyApproximation;
using multibody::contact_solvers::internal::SapHuntCrossleyConstraint;
using multibody::contact_solvers::internal::SapSolverStatus;
using multibody::internal::GetCombinedDissipationTimeConstant;
using multibody::internal::GetCombinedDynamicCoulombFriction;
using multibody::internal::GetCombinedHuntCrossleyDissipation;
using multibody::internal::GetCombinedPointContactStiffness;
using multibody::internal::GetHydroelasticModulus;
using multibody::internal::GetPointContactStiffness;
using multibody::internal::TreeIndex;

// Copied from sap_solver.cc
namespace {
constexpr char kNanValuesMessage[] =
    "The typical root cause for this failure is usually outside the solver, "
    "when there are not enough checks to catch it earlier. In this case, a "
    "previous (valid) simulation result led to the generation of NaN values in "
    "a controller, that are then fed as actuation through MultibodyPlant's "
    "ports. If you don't believe this is the root cause of your problem, "
    "please contact the Drake developers and/or open a Drake issue with a "
    "minimal reproduction example to help debug your problem.";
}

template <class T>
void ConvexIntegrator<T>::DoInitialize() {
  using std::isnan;
  const int nq = plant().num_positions();
  const int nv = plant().num_velocities();

  // TODO(vincekurtz): figure out some reasonable accuracy bounds and defaults
  const double kDefaultAccuracy = 1e-1;

  // TODO(vincekurtz): in the future we might want some fancy caching instead of
  // the workspace, but for now we'll just try to allocate most things here.
  workspace_.q.resize(nq);
  workspace_.err.resize(nq + nv);
  workspace_.k.resize(nv);
  workspace_.f_ext = std::make_unique<MultibodyForces<T>>(plant());
  workspace_.v_star.resize(nv);
  workspace_.A_dense.resize(nv, nv);
  workspace_.A.assign(tree_topology().num_trees(), MatrixX<T>(nv, nv));

  workspace_.timestep_independent_data.M.resize(nv, nv);
  workspace_.timestep_independent_data.a.resize(nv);
  workspace_.timestep_independent_data.v0.resize(nv);

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");

    this->request_initial_step_size_target(this->get_maximum_step_size());
  }

  // Set the target accuracy
  double working_accuracy = this->get_target_accuracy();
  if (isnan(working_accuracy)) working_accuracy = kDefaultAccuracy;
  this->set_accuracy_in_use(working_accuracy);

  // Set SAP solver parameters (default is sparse algebra)
  sap_parameters_.linear_solver_type = SapHessianFactorizationType::kDense;
  sap_parameters_.max_iterations = 100;
}

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  // Get the plant context. Note that we assume the only continuous state is the
  // plant's, and there are no controllers connected to it.
  Context<T>& diagram_context = *this->get_mutable_context();
  Context<T>& context = plant().GetMyMutableContextFromRoot(&diagram_context);
  const int nq = plant().num_positions();
  const int nv = plant().num_velocities();

  // Set time step for debugging
  time_ = diagram_context.get_time();
  time_step_ = h;

  // Workspace preallocations
  VectorX<T>& q = workspace_.q;
  SapSolverResults<T>& sap_results = workspace_.sap_results;
  VectorX<T>& err = workspace_.err;
  TimestepIndependentProblemData<T>& data =
      workspace_.timestep_independent_data;

  // Compute problem data at time (t)
  CalcTimestepIndependentProblemData(context, &data);

  // Solve the for the full step (t+h)
  solve_phase_ = 0;
  SapContactProblem<T> problem = MakeSapContactProblem(context, data, h);
  SapSolverStatus status = SolveWithGuess(problem, data.v0, &sap_results);
  DRAKE_DEMAND(status == SapSolverStatus::kSuccess);

  // q_{t+h} = q_t + h N(q_t) v_{t+h}
  plant().MapVelocityToQDot(context, h * sap_results.v, &q);
  q += plant().GetPositions(context);

  if (this->get_fixed_step_mode()) {
    // No error control, so we'll just set x_{t+h} in the context.
    plant().SetPositions(&context, q);
    plant().SetVelocities(&context, sap_results.v);

  } else {
    // We're using error control, and will compare with two half-steps. So we'll
    // start by putting the full-step result x_{t+h} in the error buffer.
    err.head(nq) = q;
    err.tail(nv) = sap_results.v;

    // Solve the first half-step (t + h/2). Here we can reuse the problem data
    // from the full step.
    solve_phase_ = 1;
    problem = MakeSapContactProblem(context, data, 0.5 * h);
    status = SolveWithGuess(problem, data.v0, &sap_results);
    DRAKE_DEMAND(status == SapSolverStatus::kSuccess);

    // q_{t+h/2} = q_t + h/2 N(q_t) v_{t+h/2}
    plant().MapVelocityToQDot(context, 0.5 * h * sap_results.v, &q);
    q += plant().GetPositions(context);

    // Put x_{t+h/2} in the context to prepare the next step
    plant().SetPositions(&context, q);
    plant().SetVelocities(&context, sap_results.v);

    // Set up the second half-step from (t+h/2) to (t+h). We reset the initial
    // velocity, but freeze the mass matrix and coriolis/centripedal terms.
    data.v0 = plant().GetVelocities(context);
    problem = MakeSapContactProblem(context, data, 0.5 * h);

    // Solve the second half-step problem. We'll use the full step from before
    // as the initial guess.
    solve_phase_ = 2;
    const Eigen::Ref<const VectorX<T>> v_full = err.tail(nv);
    status = SolveWithGuess(problem, v_full, &sap_results);
    DRAKE_DEMAND(status == SapSolverStatus::kSuccess);

    // q_{t+h} = q_{t+h/2} + h/2 N(q_{t+h/2}) v_{t+h}
    plant().MapVelocityToQDot(context, 0.5 * h * sap_results.v, &q);
    q += plant().GetPositions(context);

    // Finish out the error computation (compare with the full step)
    err.head(nq) -= q;
    err.tail(nv) -= sap_results.v;
    this->get_mutable_error_estimate()->get_mutable_vector().SetFromVector(err);

    // Set x_{t+h} in the context, using the (more accurate) half steps
    plant().SetPositions(&context, q);
    plant().SetVelocities(&context, sap_results.v);
  }

  // Advance time to t+h
  diagram_context.SetTime(diagram_context.get_time() + h);

  return true;  // step was successful
}

template <>
SapSolverStatus ConvexIntegrator<double>::SolveWithGuess(
    const SapContactProblem<double>& problem, const VectorXd& v_guess,
    SapSolverResults<double>* results) {
  if (problem.num_constraints() == 0) {
    // In the absence of constraints the solution is trivially v = v*.
    results->Resize(problem.num_velocities(),
                    problem.num_constraint_equations());
    results->v = problem.v_star();
    results->j.setZero();
    return SapSolverStatus::kSuccess;
  }

  // Create the sap model and allocate context, which handles caching.
  auto model = std::make_unique<SapModel<double>>(
      &problem, sap_parameters_.linear_solver_type);
  auto context = model->MakeContext();

  // Put the velocities into the model context (copies
  // SapSolver::SetProblemVelocitiesIntoModelContext).
  Eigen::VectorBlock<VectorXd> v_model =
      model->GetMutableVelocities(context.get());
  model->velocities_permutation().Apply(v_guess, &v_model);

  // Solve the convex optimization problem
  // TODO(vincekurtz): consider flag for Hessian re-use here
  const SapSolverStatus status = SolveWithGuessImpl(*model, context.get());
  num_solver_iterations_ += sap_stats_.num_iters;
  if (status != SapSolverStatus::kSuccess) return status;

  // Gather up the results
  PackSapSolverResults(*model, *context, results);
  return status;
}

template <>
SapSolverStatus ConvexIntegrator<AutoDiffXd>::SolveWithGuess(
    const SapContactProblem<AutoDiffXd>&, const VectorX<AutoDiffXd>&,
    SapSolverResults<AutoDiffXd>*) {
  // Eventually we could propagate gradients with the implicit function theorem.
  // For now we'll throw if autodiff is used.
  throw std::logic_error("ConvexIntegrator does not support AutoDiffXd.");
}

template <typename T>
SapSolverStatus ConvexIntegrator<T>::SolveWithGuessImpl(
    const SapModel<T>& model, Context<T>* context)
  requires std::is_same_v<T, double>
{  // NOLINT(whitespace/braces)
  using std::abs;
  using std::max;

  const int nv = model.num_velocities();
  const int nk = model.num_constraint_equations();

  // Allocate the necessary memory to work with.
  auto scratch = model.MakeContext();
  SearchDirectionData search_direction_data(nv, nk);
  sap_stats_ = SapStatistics();

  // Set up alternative convergence criteria per [Hairer, 1996] Sec. IV.8.
  double dv_norm = 0.0;
  double last_dv_norm = 0.0;

  // Start Newton iterations.
  int k = 0;
  double ell = model.EvalCost(*context);
  double ell_previous = ell;
  bool converged = false;
  double alpha = 1.0;
  int num_line_search_iters = 0;
  bool theta_criterion_reached = false;
  double theta = std::numeric_limits<double>::quiet_NaN();
  for (;; ++k) {
    // We first verify the stopping criteria. If satisfied, we skip expensive
    // factorizations.
    double momentum_residual, momentum_scale;

    CalcStoppingCriteriaResidual(model, *context, &momentum_residual,
                                 &momentum_scale);
    sap_stats_.optimality_criterion_reached =
        momentum_residual <= sap_parameters_.abs_tolerance +
                                 sap_parameters_.rel_tolerance * momentum_scale;
    sap_stats_.cost.push_back(ell);
    sap_stats_.alpha.push_back(alpha);
    sap_stats_.momentum_residual.push_back(momentum_residual);
    sap_stats_.momentum_scale.push_back(momentum_scale);

    if (k > 1) {
      // Alternative optimality criterion based on the method of [Hairer, 1996],
      // Equation IV.8.10. This indicates whether or not the *next* step will
      // converge, using a looser tolerance (than the SAP criteria) that scales
      // with the desired accuracy.
      const double kappa = 0.05;  // copied from implicit_integrator.cc
      theta = dv_norm / last_dv_norm;
      const double eta = theta / (1.0 - theta);
      const double k_dot_tol = kappa * this->get_accuracy_in_use();
      theta_criterion_reached = (theta < 1.0) && (eta * dv_norm < k_dot_tol);

      // Choose whether to re-compute the Hessian factorization using Equation
      // IV.8.11 of [Hairer, 1996]. This essentially predicts whether we'll
      // converge within (k_max - k) iterations, assuming linear convergence.
      // We'll set k_max to something a smaller that the hard iteration limit,
      // to give ourselves some room.
      const int k_max = sap_parameters_.max_iterations / 2;
      const double anticipated_residual =
          std::pow(theta, k_max - k) / (1 - theta) * dv_norm;

      // Only refresh the Hessian if it looks like we won't converge in time
      if (anticipated_residual > k_dot_tol || theta >= 1.0) {
        refresh_hessian_ = true;
      }
    }

    // Force a Hessian refresh if the problem structure has changed, or if 
    // Hessian re-use is disabled by the user. 
    bool problem_structure_changed = !hessian_factorization_.matches(model);
    if (get_use_full_newton() || problem_structure_changed) {
      refresh_hessian_ = true;
    }
   
    ////////////////////////////////////////////////////////////////////////
    // Debugging plots

    if (is_first_iteration_) {
      // CVS header
      fmt::print("t,h,k,residual,refresh_hessian,problem_changed,theta_converged,theta,solve_phase\n");
      is_first_iteration_ = false;
    }

    // CSV data
    fmt::print("{},{},{},{},{},{},{},{},{}\n", time_, time_step_, k, momentum_residual,
               static_cast<int>(refresh_hessian_),
               static_cast<int>(problem_structure_changed),
               static_cast<int>(theta_criterion_reached), theta, solve_phase_);

    ////////////////////////////////////////////////////////////////////////

    // TODO(amcastro-tri): consider monitoring the duality gap.
    if (sap_stats_.optimality_criterion_reached ||
        sap_stats_.cost_criterion_reached) {
      converged = true;
      break;
    } else {
      // SAP's convergence is monotonic. We sanity check this here. We use a
      // slop to account for round-off errors.
      // N.B. Notice the check for monotonic convergence is placed AFTER the
      // check for convergence. This is done puposedly to avoid round-off errors
      // in the cost near convergence when the gradient is almost zero.
      const double ell_scale = 0.5 * (abs(ell) + abs(ell_previous));
      const double ell_slop =
          sap_parameters_.relative_slop * std::max(1.0, ell_scale);
      if (ell > ell_previous + ell_slop) {
        DRAKE_LOGGER_DEBUG(
            "At iter {} cost increased by: {}. alpha = {}. Relative momentum "
            "residual = {}\n",
            k, std::abs(ell - ell_previous), alpha,
            momentum_residual / momentum_scale);
        if (sap_parameters_.nonmonotonic_convergence_is_error) {
          throw std::logic_error(
              "SapSolver: Non-monotonic convergence detected.");
        }
      }
    }

    // Exit if the maximum number of iterations is reached, but only after
    // checking the convergence criteria, so that also the last iteration is
    // considered.
    if (k == sap_parameters_.max_iterations) break;

    // This is the most expensive update: it performs the factorization of H to
    // solve for the search direction dv.
    CalcSearchDirectionData(model, *context, &search_direction_data);
    const VectorX<double>& dv = search_direction_data.dv;

    // Perform line search. We'll do exact linesearch only, regardless of the
    // sap parameters.
    std::tie(alpha, num_line_search_iters) = PerformExactLineSearch(
        model, *context, search_direction_data, scratch.get());

    sap_stats_.num_line_search_iters += num_line_search_iters;

    // Update state.
    last_dv_norm = dv_norm;
    dv_norm = dv.norm();
    model.GetMutableVelocities(context) += alpha * dv;

    ell_previous = ell;
    ell = model.EvalCost(*context);

    const double ell_scale = 0.5 * (abs(ell) + abs(ell_previous));
    // N.B. Even though theoretically we expect ell < ell_previous, round-off
    // errors might make the difference ell_previous - ell negative, within
    // machine epsilon. Therefore we take the absolute value here.
    const double ell_decrement = std::abs(ell_previous - ell);

    // N.B. Here we want alpha≈1 and therefore we impose alpha > 0.5, an
    // arbitrarily "large" value. This is to avoid a false positive on the
    // convergence of the cost due to a small value of the line search
    // parameter.
    sap_stats_.cost_criterion_reached =
        ell_decrement < sap_parameters_.cost_abs_tolerance +
                            sap_parameters_.cost_rel_tolerance * ell_scale &&
        alpha > 0.5;

    // The theta convergence criterion is only relevant *after* a step has been
    // taken. This is different from the (tighter) SAP convergence thresholds,
    // which are checked before the step is taken.
    if (theta_criterion_reached) {
      converged = true;
      break;
    }
  }

  if (!converged) return SapSolverStatus::kFailure;

  // N.B. If the stopping criteria is satisfied for k = 0, the solver is not
  // even instantiated and no factorizations are performed (the expensive part
  // of the computation). We report zero number of iterations.
  sap_stats_.num_iters = k;

  return SapSolverStatus::kSuccess;
}

template <typename T>
void ConvexIntegrator<T>::CalcStoppingCriteriaResidual(
    const SapModel<T>& model, const Context<T>& context, T* momentum_residual,
    T* momentum_scale) const {
  using std::max;
  const VectorX<T>& inv_sqrt_A = model.inv_sqrt_dynamics_matrix();
  const VectorX<T>& p = model.EvalMomentum(context);
  const VectorX<T>& jc = model.EvalGeneralizedImpulses(context);
  const VectorX<T>& ell_grad = model.EvalCostGradient(context);

  // Scale generalized momentum quantities using inv_sqrt_A so that all entries
  // have the same units and we can weigh them equally.
  const VectorX<T> ell_grad_tilde = inv_sqrt_A.asDiagonal() * ell_grad;
  const VectorX<T> p_tilde = inv_sqrt_A.asDiagonal() * p;
  const VectorX<T> jc_tilde = inv_sqrt_A.asDiagonal() * jc;

  *momentum_residual = ell_grad_tilde.norm();
  *momentum_scale = max(p_tilde.norm(), jc_tilde.norm());
}

template <typename T>
void ConvexIntegrator<T>::PackSapSolverResults(
    const SapModel<T>& model, const Context<T>& context,
    SapSolverResults<T>* results) const {
  DRAKE_DEMAND(results != nullptr);
  results->Resize(model.problem().num_velocities(),
                  model.num_constraint_equations());

  // For non-participating velocities the solutions is v = v*. Therefore we
  // first initialize to v = v* and overwrite with the non-trivial participating
  // values in the following line.
  results->v = model.problem().v_star();
  const VectorX<T>& v_participating = model.GetVelocities(context);
  model.velocities_permutation().ApplyInverse(v_participating, &results->v);

  // Constraints equations are clustered (essentially their order is permuted
  // for a better sparsity structure). Therefore constraint velocities and
  // impulses are evaluated in this clustered order and permuted into the
  // original order described by the model right after.
  const VectorX<T>& vc_clustered = model.EvalConstraintVelocities(context);
  model.impulses_permutation().ApplyInverse(vc_clustered, &results->vc);
  const VectorX<T>& gamma_clustered = model.EvalImpulses(context);
  model.impulses_permutation().ApplyInverse(gamma_clustered, &results->gamma);

  // For non-participating velocities we have v=v* and the generalized impulses
  // are zero. Therefore we first zero-out all generalized impulses and
  // overwrite with the non-trivial non-zero values for the participating DOFs
  // right after.
  const VectorX<T>& tau_participating = model.EvalGeneralizedImpulses(context);
  results->j.setZero();
  model.velocities_permutation().ApplyInverse(tau_participating, &results->j);
}

template <typename T>
void ConvexIntegrator<T>::CalcHessianFactorization(
    const SapModel<T>& model, const Context<T>& context,
    HessianFactorization* hessian)
  requires std::is_same_v<T, double>
{  // NOLINT(whitespace/braces)
  // Hackily keep J and A from going out of scope even when model is destoryed.
  // TODO(vincekurtz): consider storing these in hessian_factorization_ or
  // figuring out a better solution.
  A_ = model.dynamics_matrix();
  J_ = model.constraints_bundle().J();

  *hessian = HessianFactorization(model.hessian_type(), &A_, &J_, model);
  const std::vector<MatrixX<double>>& G = model.EvalConstraintsHessian(context);
  hessian->UpdateWeightMatrixAndFactor(G);

  num_hessian_factorizations_++;
}

template <typename T>
void ConvexIntegrator<T>::CalcSearchDirectionData(const SapModel<T>& model,
                                                  const Context<T>& context,
                                                  SearchDirectionData* data)
  requires std::is_same_v<T, double>
{  // NOLINT(whitespace/braces)
  // We compute the rhs on data->dv to allow in-place solution.
  data->dv = -model.EvalCostGradient(context);

  // Factorizing the Hessian is expensive, so we only do it when we have to
  if (refresh_hessian_) {
    CalcHessianFactorization(model, context, &hessian_factorization_);

    // Now that we've factorized the hessian, let's try not to do it again at
    // the next iteration.
    refresh_hessian_ = false;
  }

  hessian_factorization_.SolveInPlace(&data->dv);

  // Update Δp, Δvc and d²ellA/dα².
  model.constraints_bundle().J().Multiply(data->dv, &data->dvc);
  model.MultiplyByDynamicsMatrix(data->dv, &data->dp);
  data->d2ellA_dalpha2 = data->dv.dot(data->dp);

  using std::isnan;
  if (isnan(data->d2ellA_dalpha2)) {
    throw std::logic_error(fmt::format(
        "The Hessian of the momentum cost along the search direction is NaN. "
        "{}",
        kNanValuesMessage));
  }
  if (data->d2ellA_dalpha2 <= 0) {
    throw std::logic_error(fmt::format(
        "The Hessian of the momentum cost along the search direction is not "
        "positive, d²ℓ/dα² = {}. This can only be caused by a mass matrix that "
        "is not SPD. This would indicate bad problem data (e.g. a zero mass "
        "floating body, though this would be caught earlier). If you don't "
        "believe this is the root cause of your problem, please contact the "
        "Drake developers and/or open a Drake issue with a minimal "
        "reproduction example to help debug your problem.",
        data->d2ellA_dalpha2));
  }
}

template <typename T>
std::pair<T, int> ConvexIntegrator<T>::PerformExactLineSearch(
    const SapModel<T>& model, const Context<T>& context,
    const SearchDirectionData& search_direction_data, Context<T>* scratch) const
  requires std::is_same_v<T, double>
{  // NOLINT(whitespace/braces)
  DRAKE_DEMAND(sap_parameters_.line_search_type ==
               SapSolverParameters::LineSearchType::kExact);
  DRAKE_DEMAND(scratch != nullptr);
  DRAKE_DEMAND(scratch != &context);
  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const VectorX<double>& ell_grad_v0 = model.EvalCostGradient(context);
  const VectorX<double>& dv = search_direction_data.dv;
  const double dell_dalpha0 = ell_grad_v0.dot(dv);

  // dℓ/dα(α = 0) is guaranteed to be strictly negative given the the Hessian of
  // the cost is positive definite. Only round-off errors in the factorization
  // of the Hessian for ill-conditioned systems (small regularization) can
  // destroy this property. If so, we abort given that'd mean the model must be
  // revisited.
  if (dell_dalpha0 >= 0) {
    throw std::logic_error(
        "The cost does not decrease along the search direction. This is "
        "usually caused by an excessive accumulation round-off errors for "
        "ill-conditioned systems. Consider revisiting your model.");
  }

  const double alpha_max = sap_parameters_.exact_line_search.alpha_max;
  double dell{NAN};
  double d2ell{NAN};
  VectorX<double> vec_scratch;
  const double ell0 =
      CalcCostAlongLine(model, context, search_direction_data, alpha_max,
                        scratch, &dell, &d2ell, &vec_scratch);

  // If the cost is still decreasing at alpha_max, we accept this value.
  if (dell <= 0) return std::make_pair(alpha_max, 0);

  // If the user requests very tight tolerances (say, smaller than 10⁻¹⁰) then
  // we might enter the line search with a very small gradient. If close to
  // machine epsilon, the Newton method below might return inaccurate results.
  // Therefore return early if we detect this situation.
  // For these tight tolerances, we'd reach this condition close to the global
  // minimizer of the cost. Even if the norm of the search direction Δv is
  // small, we allow the Newton solver to take a full step and therefore we
  // return with a step size of one.
  if (-dell_dalpha0 < sap_parameters_.cost_abs_tolerance +
                          sap_parameters_.cost_rel_tolerance * ell0)
    return std::make_pair(1.0, 0);

  // N.B. We place the data needed to evaluate cost and gradients into a single
  // struct so that cost_and_gradient only needs to capture a single pointer.
  // This avoids heap allocations when passing the lambda to
  // DoNewtonWithBisectionFallback().
  struct EvalData {
    const ConvexIntegrator<double>& solver;
    const SapModel<double>& model;
    const Context<double>& context0;  // Context at alpha = 0.
    const SearchDirectionData& search_direction_data;
    Context<double>& scratch;  // Context at alpha != 0.
    // N.B. We normalize the gradient to minimize round-off errors as f(alpha) =
    // −ℓ'(α)/dell_scale.
    const double dell_scale;
    VectorX<double> vec_scratch;
  };

  // N.B. At this point we know that dell_dalpha0 < 0. Also, if the line search
  // was called it is because the residual (the gradient of the cost) is
  // non-zero. Therefore we can safely divide by dell_dalpha0.
  // N.B. We then define f(alpha) = −ℓ'(α)/ℓ'₀ so that f(alpha=0) = -1.
  const double dell_scale = -dell_dalpha0;
  EvalData data{*this,    model,     context, search_direction_data,
                *scratch, dell_scale};

  // Cost and gradient of f(α) = −ℓ'(α)/ℓ'₀.
  auto cost_and_gradient = [&data](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    data.solver.CalcCostAlongLine(
        data.model, data.context0, data.search_direction_data, x, &data.scratch,
        &dell_dalpha, &d2ell_dalpha2, &data.vec_scratch);
    return std::make_pair(dell_dalpha / data.dell_scale,
                          d2ell_dalpha2 / data.dell_scale);
  };

  // To estimate a guess, we approximate the cost as being quadratic around
  // alpha = 0.
  const double alpha_guess = std::min(-dell_dalpha0 / d2ell, alpha_max);
  if (std::isnan(alpha_guess)) {
    throw std::logic_error(fmt::format(
        "The initial guess for line search is NaN. {}", kNanValuesMessage));
  }

  // N.B. If we are here, then we already know that dell_dalpha0 < 0 and dell >
  // 0, and therefore [0, alpha_max] is a valid bracket.
  const Bracket bracket(0., dell_dalpha0 / dell_scale, alpha_max,
                        dell / dell_scale);

  // This relative tolerance was obtained by experimentation on a large set of
  // tests cases. We found out that with f_tolerance ∈ [10⁻¹⁴, 10⁻³] the solver
  // is robust with small changes in performances (about 30%). We then choose a
  // safe tolerance far enough from the lower limit (close to machine epsilon)
  // and the upper limit (close to an inexact method).
  const double f_tolerance = 1.0e-8;  // f = −ℓ'(α)/ℓ'₀ is dimensionless.
  const double alpha_tolerance = f_tolerance * alpha_guess;
  const auto [alpha, iters] = DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, alpha_tolerance, f_tolerance,
      sap_parameters_.exact_line_search.max_iterations);

  return std::make_pair(alpha, iters);
}

template <typename T>
T ConvexIntegrator<T>::CalcCostAlongLine(
    const SapModel<T>& model, const Context<T>& context,
    const SearchDirectionData& search_direction_data, const T& alpha,
    Context<T>* scratch, T* dell_dalpha, T* d2ell_dalpha2,
    VectorX<T>* d2ell_dalpha2_scratch) const {
  DRAKE_DEMAND(scratch != nullptr);
  DRAKE_DEMAND(scratch != &context);
  if (d2ell_dalpha2 != nullptr) DRAKE_DEMAND(d2ell_dalpha2_scratch != nullptr);

  // Data.
  const VectorX<T>& v_star = model.v_star();

  // Search direction quantities at state v.
  const VectorX<T>& dv = search_direction_data.dv;
  const VectorX<T>& dp = search_direction_data.dp;
  const VectorX<T>& dvc = search_direction_data.dvc;
  const T& d2ellA_dalpha2 = search_direction_data.d2ellA_dalpha2;

  // N.B. d2ellA_dalpha2 coming as input in SearchDirectionData was already
  // validated to be strictly positive by CalcSearchDirectionData(). Therefore
  // an assert here is sufficient.
  DRAKE_ASSERT(d2ellA_dalpha2 > 0.0);

  // State at v(alpha).
  Context<T>& context_alpha = *scratch;
  const VectorX<T>& v = model.GetVelocities(context);
  model.GetMutableVelocities(&context_alpha) = v + alpha * dv;

  if (d2ell_dalpha2 != nullptr) {
    // Since it is more efficient to calculate impulses (gamma) and their
    // derivatives (G) together, this evaluation avoids calculating the impulses
    // twice.
    model.EvalConstraintsHessian(context_alpha);
  }

  // Update velocities and impulses at v(alpha).
  // N.B. This evaluation should be cheap given we called
  // EvalConstraintsHessian() at the very start of the scope of this function.
  const VectorX<T>& gamma = model.EvalImpulses(context_alpha);

  // Regularizer cost.
  const T ellR = model.EvalConstraintsCost(context_alpha);

  // Momentum cost. We use the O(n) strategy described in [Castro et al., 2021].
  // The momentum cost is: ellA(α) = 0.5‖v(α)−v*‖², where ‖⋅‖ is the norm
  // defined by A. v(α) corresponds to the value of v along the search
  // direction: v(α) = v + αΔv. Using v(α) in the expression of the cost and
  // expanding the squared norm leads to: ellA(α) = 0.5‖v−v*‖² + αΔvᵀ⋅A⋅(v−v*) +
  // 0.5‖Δv‖²α². We now notice some of those terms are already cached:
  //  - dpᵀ = Δvᵀ⋅A
  //  - ellA(v) = 0.5‖v−v*‖²
  //  - d2ellA_dalpha2 = 0.5‖Δv‖²α², see [Castro et al., 2021; §VIII.C].
  T ellA = model.EvalMomentumCost(context);
  ellA += alpha * dp.dot(v - v_star);
  ellA += 0.5 * alpha * alpha * d2ellA_dalpha2;
  const T ell = ellA + ellR;

  // Compute first derivative.
  if (dell_dalpha != nullptr) {
    const VectorX<T>& v_alpha = model.GetVelocities(context_alpha);

    // First derivative.
    const T dellA_dalpha = dp.dot(v_alpha - v_star);  // Momentum term.
    const T dellR_dalpha = -dvc.dot(gamma);           // Regularizer term.
    *dell_dalpha = dellA_dalpha + dellR_dalpha;
  }

  // Compute second derivative.
  if (d2ell_dalpha2 != nullptr) {
    // N.B. This evaluation should be cheap given we called
    // EvalConstraintsHessian() at the very start of the scope of this function.
    const std::vector<MatrixX<T>>& G =
        model.EvalConstraintsHessian(context_alpha);

    // First compute d2ell_dalpha2_scratch = G⋅Δvc.
    d2ell_dalpha2_scratch->resize(model.num_constraint_equations());
    const int nc = model.num_constraints();
    int constraint_start = 0;
    for (int i = 0; i < nc; ++i) {
      const MatrixX<T>& G_i = G[i];
      // Number of equations for the i-th constraint.
      const int ni = G_i.rows();
      const auto dvc_i = dvc.segment(constraint_start, ni);
      d2ell_dalpha2_scratch->segment(constraint_start, ni) = G_i * dvc_i;
      constraint_start += ni;
    }

    // For the constraints' contribution to the cost, d²ℓ/dα² = Δvcᵀ⋅G⋅Δvc.
    const T d2ellR_dalpha2 = dvc.dot(*d2ell_dalpha2_scratch);

    // Sanity check these terms are all positive.
    using std::isnan;
    if (isnan(d2ellR_dalpha2)) {
      throw std::logic_error(fmt::format(
          "The Hessian of the constraints cost along the search direction is "
          "NaN. {}",
          kNanValuesMessage));
    }
    if (d2ellR_dalpha2 < 0) {
      throw std::logic_error(fmt::format(
          "The Hessian of the constraints cost along the search direction is "
          "negative, d²ℓ/dα² = {}. This can only be caused by a degenerate "
          "constraint Hessian (a bug). This would be indicated by a value that "
          "might be negative but close to machine epsilon. Please contact the "
          "Drake developers and/or open a Drake issue with a minimal "
          "reproduction example to help debug your problem.",
          d2ellR_dalpha2));
    }

    *d2ell_dalpha2 = d2ellA_dalpha2 + d2ellR_dalpha2;

    // N.B. With d2ellA_dalpha2 > 0 and d2ellR_dalpha2 validated to be positive
    // just a few lines above, then d2ell_dalpha2 must be strictly positive.
    DRAKE_DEMAND(*d2ell_dalpha2 > 0);
  }

  return ell;
}

template <class T>
void ConvexIntegrator<T>::CalcTimestepIndependentProblemData(
    const Context<T>& context, TimestepIndependentProblemData<T>* data) {
  // workspace pre-allocations
  VectorX<T>& k = workspace_.k;
  MultibodyForces<T>& f_ext = *workspace_.f_ext;

  data->v0 = plant().GetVelocities(context);  // initial velocity
  plant().CalcMassMatrix(context, &data->M);  // mass matrix

  // accelerations
  plant().CalcForceElementsContribution(context, &f_ext);
  k = plant().CalcInverseDynamics(
      context, VectorX<T>::Zero(plant().num_velocities()), f_ext);
  data->a = data->M.ldlt().solve(-k);
}

template <class T>
SapContactProblem<T> ConvexIntegrator<T>::MakeSapContactProblem(
    const Context<T>& context, const TimestepIndependentProblemData<T>& data,
    const T& h) {
  // workspace pre-allocations
  VectorX<T>& v_star = workspace_.v_star;
  MatrixX<T>& A_dense = workspace_.A_dense;
  std::vector<MatrixX<T>>& A = workspace_.A;

  // free-motion velocities v*
  v_star = data.v0 + h * data.a;

  // linearized dynamics matrix A = M + hD
  A_dense = data.M;
  A_dense.diagonal() += h * plant().EvalJointDampingCache(context);

  for (TreeIndex t(0); t < tree_topology().num_trees(); ++t) {
    const int tree_start_in_v = tree_topology().tree_velocities_start_in_v(t);
    const int tree_nv = tree_topology().num_tree_velocities(t);
    A[t] = A_dense.block(tree_start_in_v, tree_start_in_v, tree_nv, tree_nv);
  }

  // problem creation
  // TODO(vincekurtz): consider updating rather than recreating
  SapContactProblem<T> problem(h, A, v_star);
  problem.set_num_objects(plant().num_bodies());

  // contact constraints (point contact + hydro)
  AddContactConstraints(context, &problem);

  return problem;
}

template <class T>
void ConvexIntegrator<T>::AddContactConstraints(const Context<T>& context,
                                                SapContactProblem<T>* problem) {
  // N.B. this is essentially copy-pasted from SapDriver, with some
  // simplification to focus only on kLagged
  DRAKE_DEMAND(problem != nullptr);
  DRAKE_DEMAND(plant().get_discrete_contact_approximation() ==
               DiscreteContactApproximation::kLagged);
  constexpr double sigma = 1.0e-3;

  DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      workspace_.contact_data;
  CalcContactPairs(context, &contact_pairs);
  const int num_contacts = contact_pairs.size();

  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& pair = contact_pairs[icontact];

    const T stiffness = pair.stiffness;
    const T damping = pair.damping;
    const T friction = pair.friction_coefficient;
    const auto& jacobian_blocks = pair.jacobian;

    auto make_hunt_crossley_parameters = [&]() {
      const double vs = plant().stiction_tolerance();
      SapHuntCrossleyApproximation model =
          SapHuntCrossleyApproximation::kLagged;
      return typename SapHuntCrossleyConstraint<T>::Parameters{
          model, friction, stiffness, damping, vs, sigma};
    };

    if (jacobian_blocks.size() == 1) {
      SapConstraintJacobian<T> J(jacobian_blocks[0].tree,
                                 std::move(jacobian_blocks[0].J));
      problem->AddConstraint(std::make_unique<SapHuntCrossleyConstraint<T>>(
          MakeContactConfiguration(pair), std::move(J),
          make_hunt_crossley_parameters()));
    } else {
      SapConstraintJacobian<T> J(
          jacobian_blocks[0].tree, std::move(jacobian_blocks[0].J),
          jacobian_blocks[1].tree, std::move(jacobian_blocks[1].J));
      problem->AddConstraint(std::make_unique<SapHuntCrossleyConstraint<T>>(
          MakeContactConfiguration(pair), std::move(J),
          make_hunt_crossley_parameters()));
    }
  }
}

template <class T>
void ConvexIntegrator<T>::CalcContactPairs(
    const Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* result) const {
  // N.B. this is essentially copy-pasted from
  // DiscreteUpdateManater::CalcDiscreteContactPairs.
  plant().ValidateContext(context);
  DRAKE_DEMAND(result != nullptr);
  result->Clear();
  AppendDiscreteContactPairsForPointContact(context, result);
  if constexpr (std::is_same_v<T, symbolic::Expression>) {
    throw std::logic_error("This method doesn't support T = Expression.");
  } else {
    AppendDiscreteContactPairsForHydroelasticContact(context, result);
  }
}

template <class T>
void ConvexIntegrator<T>::AppendDiscreteContactPairsForPointContact(
    const Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* contact_pairs) const {
  // N.B. this is essentially copy-pasted from DiscreteUpdateManager

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      plant().EvalGeometryContactData(context).get().point_pairs;

  const int num_point_contacts = point_pairs.size();
  if (num_point_contacts == 0) {
    return;
  }

  contact_pairs->Reserve(num_point_contacts, 0, 0);
  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);
  const MultibodyTreeTopology& topology = internal_tree().get_topology();
  const Eigen::VectorBlock<const VectorX<T>> v = plant().GetVelocities(context);
  const Frame<T>& frame_W = plant().world_frame();

  // Scratch workspace variables.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);

  // Fill in the point contact pairs.
  for (int point_pair_index = 0; point_pair_index < num_point_contacts;
       ++point_pair_index) {
    const PenetrationAsPointPair<T>& pair = point_pairs[point_pair_index];
    const BodyIndex body_A_index = FindBodyByGeometryId(pair.id_A);
    const RigidBody<T>& body_A = plant().get_body(body_A_index);
    const BodyIndex body_B_index = FindBodyByGeometryId(pair.id_B);
    const RigidBody<T>& body_B = plant().get_body(body_B_index);

    const TreeIndex treeA_index = topology.body_to_tree_index(body_A_index);
    const TreeIndex treeB_index = topology.body_to_tree_index(body_B_index);
    const bool treeA_has_dofs = topology.tree_has_dofs(treeA_index);
    const bool treeB_has_dofs = topology.tree_has_dofs(treeB_index);

    const T kA = GetPointContactStiffness(
        pair.id_A, default_contact_stiffness(), inspector);
    const T kB = GetPointContactStiffness(
        pair.id_B, default_contact_stiffness(), inspector);

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kA + kB;
    const T wA = (denom == 0 ? 0.5 : kA / denom);
    const T wB = (denom == 0 ? 0.5 : kB / denom);
    const Vector3<T> p_WC = wA * pair.p_WCa + wB * pair.p_WCb;

    // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian will
    // be:
    //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
    // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
    internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WAc_W);
    internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WBc_W);
    Jv_AcBc_W = Jv_WBc_W - Jv_WAc_W;

    // Define a contact frame C at the contact point such that the z-axis Cz
    // equals nhat_W. The tangent vectors are arbitrary, with the only
    // requirement being that they form a valid right handed basis with
    // nhat_W.
    const Vector3<T> nhat_AB_W = -pair.nhat_BA_W;
    math::RotationMatrix<T> R_WC =
        math::RotationMatrix<T>::MakeFromOneVector(nhat_AB_W, 2);

    // Contact velocity stored in the current context (previous time step).
    const Vector3<T> v_AcBc_W = Jv_AcBc_W * v;
    const Vector3<T> v_AcBc_C = R_WC.transpose() * v_AcBc_W;
    const T vn0 = v_AcBc_C(2);

    // We have at most two blocks per contact.
    std::vector<typename DiscreteContactPair<T>::JacobianTreeBlock>
        jacobian_blocks;
    jacobian_blocks.reserve(2);

    // Tree A contribution to contact Jacobian Jv_W_AcBc_C.
    if (treeA_has_dofs) {
      Matrix3X<T> J =
          R_WC.matrix().transpose() *
          Jv_AcBc_W.middleCols(
              tree_topology().tree_velocities_start_in_v(treeA_index),
              tree_topology().num_tree_velocities(treeA_index));
      jacobian_blocks.emplace_back(treeA_index, MatrixBlock<T>(std::move(J)));
    }

    // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
    // This contribution must be added only if B is different from A.
    if ((treeB_has_dofs && !treeA_has_dofs) ||
        (treeB_has_dofs && treeB_index != treeA_index)) {
      Matrix3X<T> J =
          R_WC.matrix().transpose() *
          Jv_AcBc_W.middleCols(
              tree_topology().tree_velocities_start_in_v(treeB_index),
              tree_topology().num_tree_velocities(treeB_index));
      jacobian_blocks.emplace_back(treeB_index, MatrixBlock<T>(std::move(J)));
    }

    // Contact stiffness and damping
    const T k = GetCombinedPointContactStiffness(
        pair.id_A, pair.id_B, default_contact_stiffness(), inspector);
    // Hunt & Crossley dissipation. Ignored, for instance, by Sap. See
    // multibody::DiscreteContactApproximation for details about these contact
    // models.
    const T d = GetCombinedHuntCrossleyDissipation(
        pair.id_A, pair.id_B, kA, kB, default_contact_dissipation(), inspector);
    // Dissipation time scale. Ignored, for instance, by Similar and Lagged
    // models. See multibody::DiscreteContactApproximation for details about
    // these contact models.
    const double default_dissipation_time_constant = 0.1;
    const T tau = GetCombinedDissipationTimeConstant(
        pair.id_A, pair.id_B, default_dissipation_time_constant, body_A.name(),
        body_B.name(), inspector);
    const T mu =
        GetCombinedDynamicCoulombFriction(pair.id_A, pair.id_B, inspector);

    const T phi0 = -pair.depth;
    const T fn0 = k * pair.depth;

    // Contact point position relative to each body.
    const RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, body_A);
    const Vector3<T>& p_WA = X_WA.translation();
    const Vector3<T> p_AC_W = p_WC - p_WA;
    const RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, body_B);
    const Vector3<T>& p_WB = X_WB.translation();
    const Vector3<T> p_BC_W = p_WC - p_WB;

    DiscreteContactPair<T> contact_pair{.jacobian = std::move(jacobian_blocks),
                                        .id_A = pair.id_A,
                                        .object_A = body_A_index,
                                        .id_B = pair.id_B,
                                        .object_B = body_B_index,
                                        .R_WC = R_WC,
                                        .p_WC = p_WC,
                                        .p_ApC_W = p_AC_W,
                                        .p_BqC_W = p_BC_W,
                                        .nhat_BA_W = pair.nhat_BA_W,
                                        .phi0 = phi0,
                                        .vn0 = vn0,
                                        .fn0 = fn0,
                                        .stiffness = k,
                                        .damping = d,
                                        .dissipation_time_scale = tau,
                                        .friction_coefficient = mu,
                                        .surface_index{} /* no surface index */,
                                        .face_index = {} /* no face index */,
                                        .point_pair_index = point_pair_index};
    contact_pairs->AppendPointData(std::move(contact_pair));
  }
}

template <typename T>
void ConvexIntegrator<T>::AppendDiscreteContactPairsForHydroelasticContact(
    const Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* contact_pairs) const
  requires scalar_predicate<T>::is_bool
{  // NOLINT(whitespace/braces)
  const std::vector<geometry::ContactSurface<T>>& surfaces =
      plant().EvalGeometryContactData(context).get().surfaces;
  // N.B. For discrete hydro we use a first order quadrature rule. As such,
  // the per-face quadrature point is the face's centroid and the weight is 1.
  // This is compatible with a mesh that is triangle or polygon. If we attempted
  // higher order quadrature, polygons would have to be decomposed into smaller
  // n-gons which can receive an appropriate set of quadrature points.
  int num_hydro_contacts = 0;
  for (const auto& s : surfaces) {
    // One quadrature point per face.
    num_hydro_contacts += s.num_faces();
  }
  if (num_hydro_contacts == 0) {
    return;
  }

  contact_pairs->Reserve(0, num_hydro_contacts, 0);
  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);
  const MultibodyTreeTopology& topology = internal_tree().get_topology();
  const Eigen::VectorBlock<const VectorX<T>> v = plant().GetVelocities(context);
  const Frame<T>& frame_W = plant().world_frame();

  // Scratch workspace variables.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);

  const int num_surfaces = surfaces.size();
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    const auto& s = surfaces[surface_index];

    const bool M_is_compliant = s.HasGradE_M();
    const bool N_is_compliant = s.HasGradE_N();
    DRAKE_DEMAND(M_is_compliant || N_is_compliant);

    // We always call the body associated with geometry M, A, and the body
    // associated with geometry N, B.
    const BodyIndex body_A_index = FindBodyByGeometryId(s.id_M());
    const RigidBody<T>& body_A = plant().get_body(body_A_index);
    const BodyIndex body_B_index = FindBodyByGeometryId(s.id_N());
    const RigidBody<T>& body_B = plant().get_body(body_B_index);

    const TreeIndex& tree_A_index = topology.body_to_tree_index(body_A_index);
    const TreeIndex& tree_B_index = topology.body_to_tree_index(body_B_index);
    const bool treeA_has_dofs = topology.tree_has_dofs(tree_A_index);
    const bool treeB_has_dofs = topology.tree_has_dofs(tree_B_index);

    // TODO(amcastro-tri): Consider making the modulus required, instead of
    // a default infinite value.
    const T hydro_modulus_M = GetHydroelasticModulus(
        s.id_M(), std::numeric_limits<double>::infinity(), inspector);
    const T hydro_modulus_N = GetHydroelasticModulus(
        s.id_N(), std::numeric_limits<double>::infinity(), inspector);
    // Hunt & Crossley dissipation. Used by the Tamsi, Lagged, and Similar
    // contact models. Ignored by Sap. See
    // multibody::DiscreteContactApproximation for details about these contact
    // models.
    const T d = GetCombinedHuntCrossleyDissipation(
        s.id_M(), s.id_N(), hydro_modulus_M, hydro_modulus_N,
        default_contact_dissipation(), inspector);
    // Dissipation time scale. Used by Sap contact model. Ignored by Tamsi,
    // Lagged, and Similar contact model. See
    // multibody::DiscreteContactApproximation for details about these contact
    // models.
    const double default_dissipation_time_constant = 0.1;
    const T tau = GetCombinedDissipationTimeConstant(
        s.id_M(), s.id_N(), default_dissipation_time_constant, body_A.name(),
        body_B.name(), inspector);
    // Combine friction coefficients.
    const T mu =
        GetCombinedDynamicCoulombFriction(s.id_M(), s.id_N(), inspector);

    for (int face = 0; face < s.num_faces(); ++face) {
      const T& Ae = s.area(face);  // Face element area.

      // We found out that the hydroelastic query might report
      // infinitesimally small triangles (consider for instance an initial
      // condition that perfectly places an object at zero distance from the
      // ground.) While the area of zero sized triangles is not a problem by
      // itself, the badly computed normal on these triangles leads to
      // problems when computing the contact Jacobians (since we need to
      // obtain an orthonormal basis based on that normal.)
      // We therefore ignore infinitesimally small triangles. The tolerance
      // below is somehow arbitrary and could possibly be tightened.
      if (Ae > 1.0e-14) {
        // From ContactSurface's documentation: The normal of each face is
        // guaranteed to point "out of" N and "into" M. Recall that A is
        // associated with M, and B is associated with N.
        const Vector3<T>& nhat_BA_W = s.face_normal(face);

        // One dimensional pressure gradient (in Pa/m). Unlike [Masterjohn
        // 2022], for convenience we define both pressure gradients
        // to be positive in the direction "into" the bodies. Therefore,
        // we use the minus sign for gN.
        // [Masterjohn 2022] Velocity Level Approximation of Pressure
        // Field Contact Patches.
        const T gM = M_is_compliant
                         ? s.EvaluateGradE_M_W(face).dot(nhat_BA_W)
                         : T(std::numeric_limits<double>::infinity());
        const T gN = N_is_compliant
                         ? -s.EvaluateGradE_N_W(face).dot(nhat_BA_W)
                         : T(std::numeric_limits<double>::infinity());

        constexpr double kGradientEpsilon = 1.0e-14;
        if (gM < kGradientEpsilon || gN < kGradientEpsilon) {
          // Mathematically g = gN*gM/(gN+gM) and therefore g = 0 when
          // either gradient on one of the bodies is zero. A zero gradient
          // means there is no contact constraint, and therefore we
          // ignore it to avoid numerical problems in the discrete solver.
          continue;
        }

        // Effective hydroelastic pressure gradient g result of
        // compliant-compliant interaction, see [Masterjohn 2022].
        // The expression below is mathematically equivalent to g =
        // gN*gM/(gN+gM) but it has the advantage of also being valid if
        // one of the gradients is infinity.
        const T g = 1.0 / (1.0 / gM + 1.0 / gN);

        // Position of quadrature point C in the world frame (since mesh_W
        // is measured and expressed in W).
        const Vector3<T>& p_WC = s.centroid(face);

        // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian
        // will be:
        //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
        // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
        internal_tree().CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W,
            p_WC, frame_W, frame_W, &Jv_WAc_W);
        internal_tree().CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W,
            p_WC, frame_W, frame_W, &Jv_WBc_W);
        Jv_AcBc_W = Jv_WBc_W - Jv_WAc_W;

        // Define a contact frame C at the contact point such that the
        // z-axis Cz equals nhat_AB_W. The tangent vectors are arbitrary,
        // with the only requirement being that they form a valid right
        // handed basis with nhat_AB_W.
        const Vector3<T> nhat_AB_W = -nhat_BA_W;
        math::RotationMatrix<T> R_WC =
            math::RotationMatrix<T>::MakeFromOneVector(nhat_AB_W, 2);

        // Contact velocity stored in the current context (previous time
        // step).
        const Vector3<T> v_AcBc_W = Jv_AcBc_W * v;
        const Vector3<T> v_AcBc_C = R_WC.transpose() * v_AcBc_W;
        const T vn0 = v_AcBc_C(2);

        // We have at most two blocks per contact.
        std::vector<typename DiscreteContactPair<T>::JacobianTreeBlock>
            jacobian_blocks;
        jacobian_blocks.reserve(2);

        // Tree A contribution to contact Jacobian Jv_W_AcBc_C.
        if (treeA_has_dofs) {
          Matrix3X<T> J =
              R_WC.matrix().transpose() *
              Jv_AcBc_W.middleCols(
                  tree_topology().tree_velocities_start_in_v(tree_A_index),
                  tree_topology().num_tree_velocities(tree_A_index));
          jacobian_blocks.emplace_back(tree_A_index,
                                       MatrixBlock<T>(std::move(J)));
        }

        // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
        // This contribution must be added only if B is different from A.
        if ((treeB_has_dofs && !treeA_has_dofs) ||
            (treeB_has_dofs && tree_B_index != tree_A_index)) {
          Matrix3X<T> J =
              R_WC.matrix().transpose() *
              Jv_AcBc_W.middleCols(
                  tree_topology().tree_velocities_start_in_v(tree_B_index),
                  tree_topology().num_tree_velocities(tree_B_index));
          jacobian_blocks.emplace_back(tree_B_index,
                                       MatrixBlock<T>(std::move(J)));
        }

        // For a triangle, its centroid has the fixed barycentric
        // coordinates independent of the shape of the triangle. Using
        // barycentric coordinates to evaluate field value could be
        // faster than using Cartesian coordinates, especially if the
        // TriangleSurfaceMeshFieldLinear<> does not store gradients and
        // has to solve linear equations to convert Cartesian to
        // barycentric coordinates.
        const Vector3<T> tri_centroid_barycentric(1 / 3., 1 / 3., 1 / 3.);
        // Pressure at the quadrature point.
        const T p0 = s.is_triangle()
                         ? s.tri_e_MN().Evaluate(face, tri_centroid_barycentric)
                         : s.poly_e_MN().EvaluateCartesian(face, p_WC);

        // Force contribution by this quadrature point.
        const T fn0 = Ae * p0;

        // Effective compliance in the normal direction for the given
        // discrete patch, refer to [Masterjohn 2022] for details.
        // [Masterjohn 2022] Masterjohn J., Guoy D., Shepherd J. and
        // Castro A., 2022. Velocity Level Approximation of Pressure Field
        // Contact Patches. Available at https://arxiv.org/abs/2110.04157.
        const T k = Ae * g;

        // phi < 0 when in penetration.
        const T phi0 = -p0 / g;

        // Contact point position relative to each body.
        const RigidTransform<T>& X_WA =
            plant().EvalBodyPoseInWorld(context, body_A);
        const Vector3<T>& p_WA = X_WA.translation();
        const Vector3<T> p_AC_W = p_WC - p_WA;
        const RigidTransform<T>& X_WB =
            plant().EvalBodyPoseInWorld(context, body_B);
        const Vector3<T>& p_WB = X_WB.translation();
        const Vector3<T> p_BC_W = p_WC - p_WB;

        DiscreteContactPair<T> contact_pair{
            .jacobian = std::move(jacobian_blocks),
            .id_A = s.id_M(),
            .object_A = body_A_index,
            .id_B = s.id_N(),
            .object_B = body_B_index,
            .R_WC = R_WC,
            .p_WC = p_WC,
            .p_ApC_W = p_AC_W,
            .p_BqC_W = p_BC_W,
            .nhat_BA_W = nhat_BA_W,
            .phi0 = phi0,
            .vn0 = vn0,
            .fn0 = fn0,
            .stiffness = k,
            .damping = d,
            .dissipation_time_scale = tau,
            .friction_coefficient = mu,
            .surface_index = surface_index,
            .face_index = face,
            .point_pair_index = {} /* no point pair index */};
        contact_pairs->AppendHydroData(std::move(contact_pair));
      }
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
