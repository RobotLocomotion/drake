#include "drake/multibody/plant/tamsi_driver.h"

#include <algorithm>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/slicing_and_indexing.h"
#include "drake/multibody/plant/tamsi_solver.h"

using drake::multibody::contact_solvers::internal::ContactSolverResults;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
TamsiDriver<T>::TamsiDriver(const CompliantContactManager<T>* manager)
    : manager_(manager) {
  DRAKE_DEMAND(manager != nullptr);
}

template <typename T>
void TamsiDriver<T>::CalcContactSolverResults(
    const systems::Context<T>& context,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  // Assert this method was called on a context storing discrete state.
  plant().ValidateContext(context);
  DRAKE_ASSERT(context.num_continuous_states() == 0);
  // Only discrete state updates for rigid bodies is supported.
  DRAKE_ASSERT(context.num_discrete_state_groups() == 1);

  const int nq = plant().num_positions();
  const int nv = plant().num_velocities();

  // Quick exit if there are no moving objects.
  if (nv == 0) return;

  // Get the system state as raw Eigen vectors
  // (solution at the previous time step).
  auto x0 = context.get_discrete_state(0).get_value();
  VectorX<T> q0 = x0.topRows(nq);
  VectorX<T> v0 = x0.bottomRows(nv);

  // Mass matrix.
  MatrixX<T> M0(nv, nv);
  plant().CalcMassMatrix(context, &M0);

  // Forces at the previous time step.
  MultibodyForces<T> forces0(plant());

  manager().CalcNonContactForces(context, &forces0);

  // Workspace for inverse dynamics:
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(plant().num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Body forces (alias to forces0).
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces0.mutable_body_forces();

  // With vdot = 0, this computes:
  //   -tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  VectorX<T>& minus_tau = forces0.mutable_generalized_forces();
  manager().internal_tree().CalcInverseDynamics(
      context, vdot, F_BBo_W_array, minus_tau, &A_WB_array,
      &F_BBo_W_array, /* Note: these arrays get overwritten on output. */
      &minus_tau);

  // Compute all contact pairs, including both penetration pairs and quadrature
  // pairs for discrete hydroelastic.
  const std::vector<internal::DiscreteContactPair<T>>& contact_pairs =
      manager().EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  // Compute normal and tangential velocity Jacobians at t0.
  const internal::ContactJacobians<T>& contact_jacobians =
      manager().EvalContactJacobians(context);

  // Get friction coefficient into a single vector.
  VectorX<T> mu(num_contacts);
  std::transform(contact_pairs.begin(), contact_pairs.end(), mu.data(),
                 [](const internal::DiscreteContactPair<T>& pair) {
                   return pair.friction_coefficient;
                 });

  // Fill in data as required by our discrete solver.
  VectorX<T> fn0(num_contacts);
  VectorX<T> stiffness(num_contacts);
  VectorX<T> damping(num_contacts);
  VectorX<T> phi0(num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    fn0[i] = contact_pairs[i].fn0;
    stiffness[i] = contact_pairs[i].stiffness;
    damping[i] = contact_pairs[i].damping;
    phi0[i] = contact_pairs[i].phi0;
  }

  // Joint locking: quick exit if everything is locked.
  const auto& indices = manager().EvalJointLockingIndices(context);
  if (indices.empty()) {
    // Everything is locked! Return a result that indicates no velocity, but
    // reports normal forces.
    results->Resize(nv, num_contacts);
    results->v_next.setZero();
    results->fn = fn0;
    results->ft.setZero();
    results->vn.setZero();
    results->vt.setZero();
    results->tau_contact = contact_jacobians.Jn.transpose() * results->fn;
    return;
  }

  // Joint locking: reduce solver inputs.
  MatrixX<T> M0_unlocked = SelectRowsCols(M0, indices);
  VectorX<T> minus_tau_unlocked = SelectRows(minus_tau, indices);
  MatrixX<T> Jn_unlocked = SelectCols(contact_jacobians.Jn, indices);
  MatrixX<T> Jt_unlocked = SelectCols(contact_jacobians.Jt, indices);
  MatrixX<T> Jc_unlocked = SelectCols(contact_jacobians.Jc, indices);

  VectorX<T> v0_unlocked = SelectRows(v0, indices);

  contact_solvers::internal::ContactSolverResults<T> results_unlocked;
  results_unlocked.Resize(indices.size(), num_contacts);

  TamsiSolver<T> tamsi_solver(indices.size());
  if (tamsi_solver.get_solver_parameters().stiction_tolerance !=
      plant().stiction_tolerance()) {
    // Set the stiction tolerance according to the values set by users with
    // set_stiction_tolerance().
    TamsiSolverParameters solver_parameters;
    solver_parameters.stiction_tolerance = plant().stiction_tolerance();
    tamsi_solver.set_solver_parameters(solver_parameters);
  }

  CallTamsiSolver(&tamsi_solver, context.get_time(), v0_unlocked, M0_unlocked,
                  minus_tau_unlocked, fn0, Jn_unlocked, Jt_unlocked, stiffness,
                  damping, mu, &results_unlocked);

  // Joint locking: expand reduced outputs.
  results->v_next =
      ExpandRows(results_unlocked.v_next, plant().num_velocities(), indices);
  results->tau_contact =
      contact_jacobians.Jn.transpose() * results_unlocked.fn +
      contact_jacobians.Jt.transpose() * results_unlocked.ft;

  results->fn = results_unlocked.fn;
  results->ft = results_unlocked.ft;
  results->vn = results_unlocked.vn;
  results->vt = results_unlocked.vt;
}

template <typename T>
TamsiSolverResult TamsiDriver<T>::SolveUsingSubStepping(
    TamsiSolver<T>* tamsi_solver, int num_substeps, const MatrixX<T>& M0,
    const MatrixX<T>& Jn, const MatrixX<T>& Jt, const VectorX<T>& minus_tau,
    const VectorX<T>& stiffness, const VectorX<T>& damping,
    const VectorX<T>& mu, const VectorX<T>& v0, const VectorX<T>& fn0) const {
  const double dt = plant().time_step();  // just a shorter alias.
  const double dt_substep = dt / num_substeps;
  VectorX<T> v0_substep = v0;
  VectorX<T> fn0_substep = fn0;

  // Initialize info to an unsuccessful result.
  TamsiSolverResult info{TamsiSolverResult::kMaxIterationsReached};

  for (int substep = 0; substep < num_substeps; ++substep) {
    // Discrete update before applying friction forces.
    // We denote this state x* = [q*, v*], the "star" state.
    // Generalized momentum "star", before contact forces are applied.
    VectorX<T> p_star_substep = M0 * v0_substep - dt_substep * minus_tau;

    // Update the data.
    tamsi_solver->SetTwoWayCoupledProblemData(&M0, &Jn, &Jt, &p_star_substep,
                                              &fn0_substep, &stiffness,
                                              &damping, &mu);

    info = tamsi_solver->SolveWithGuess(dt_substep, v0_substep);

    // Break the sub-stepping loop on failure and return the info result.
    if (info != TamsiSolverResult::kSuccess) break;

    // Update previous time step to new solution.
    v0_substep = tamsi_solver->get_generalized_velocities();

    // TAMSI updates each normal force according to:
    //   fₙ = (1 − d vₙ)₊ (fₙ₀ − h k vₙ)₊
    // using the last computed normal velocity vₙ and we use the shorthand
    // notation  h = dt_substep in this scope.
    // The input fₙ₀ to the solver is the undamped (no dissipation) term only.
    // We must update fₙ₀ for each substep accordingly, i.e:
    //   fₙ₀(next) = (fₙ₀(previous) − h k vₙ(next))₊
    const auto vn_substep = tamsi_solver->get_normal_velocities();
    fn0_substep = fn0_substep.array() -
                  dt_substep * stiffness.array() * vn_substep.array();
    fn0_substep = fn0_substep.cwiseMax(T(0.0));
  }

  return info;
}

template <typename T>
void TamsiDriver<T>::CallTamsiSolver(
    TamsiSolver<T>* tamsi_solver, const T& time0, const VectorX<T>& v0,
    const MatrixX<T>& M0, const VectorX<T>& minus_tau, const VectorX<T>& fn0,
    const MatrixX<T>& Jn, const MatrixX<T>& Jt, const VectorX<T>& stiffness,
    const VectorX<T>& damping, const VectorX<T>& mu,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  // Solve for v and the contact forces.
  TamsiSolverResult info{TamsiSolverResult::kMaxIterationsReached};

  TamsiSolverParameters params = tamsi_solver->get_solver_parameters();
  // A nicely converged NR iteration should not take more than 20 iterations.
  // Otherwise we attempt a smaller time step.
  params.max_iterations = 20;
  tamsi_solver->set_solver_parameters(params);

  // We attempt to compute the update during the time interval dt using a
  // progressively larger number of sub-steps (i.e each using a smaller time
  // step than in the previous attempt). This loop breaks on the first
  // successful attempt.
  // We only allow a maximum number of trials. If the solver is unsuccessful
  // in this number of trials, the user should probably decrease the discrete
  // update time step dt or evaluate the validity of the model.
  const int kNumMaxSubTimeSteps = 20;
  int num_substeps = 0;
  do {
    ++num_substeps;
    info = SolveUsingSubStepping(tamsi_solver, num_substeps, M0, Jn, Jt,
                                 minus_tau, stiffness, damping, mu, v0, fn0);
  } while (info != TamsiSolverResult::kSuccess &&
           num_substeps < kNumMaxSubTimeSteps);

  if (info != TamsiSolverResult::kSuccess) {
    const std::string msg = fmt::format(
        "MultibodyPlant's discrete update solver failed to converge at "
        "simulation time = {} with discrete update period = {}. "
        "This usually means that the plant's discrete update period is too "
        "large to resolve the system's dynamics for the given simulation "
        "conditions. This is often the case during abrupt collisions or during "
        "complex and fast changing contact configurations. Another common "
        "cause is the use of high gains in the simulation of closed loop "
        "systems. These might cause numerical instabilities given our discrete "
        "solver uses an explicit treatment of actuation inputs. Possible "
        "solutions include:\n"
        "  1. reduce the discrete update period set at construction,\n"
        "  2. decrease the high gains in your controller whenever possible,\n"
        "  3. switch to a continuous model (discrete update period is zero), "
        "     though this might affect the simulation run time.",
        time0, plant().time_step());
    throw std::runtime_error(msg);
  }

  // TODO(amcastro-tri): implement capability to dump solver statistics to a
  // file for analysis.

  // Update the results.
  results->v_next = tamsi_solver->get_generalized_velocities();
  results->fn = tamsi_solver->get_normal_forces();
  results->ft = tamsi_solver->get_friction_forces();
  results->vn = tamsi_solver->get_normal_velocities();
  results->vt = tamsi_solver->get_tangential_velocities();
  results->tau_contact = tamsi_solver->get_generalized_contact_forces();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::TamsiDriver);
