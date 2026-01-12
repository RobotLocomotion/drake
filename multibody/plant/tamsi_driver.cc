#include "drake/multibody/plant/tamsi_driver.h"

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/slicing_and_indexing.h"
#include "drake/multibody/plant/tamsi_solver.h"
#include "drake/multibody/topology/forest.h"

using drake::geometry::GeometryId;
using drake::math::RigidTransform;
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
TamsiDriver<T>::~TamsiDriver() = default;

template <typename T>
internal::ContactJacobians<T> TamsiDriver<T>::CalcContactJacobians(
    const systems::Context<T>& context) const {
  const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      manager().EvalDiscreteContactPairs(context);

  const int nc = contact_pairs.size();
  const int nv = manager().plant().num_velocities();
  internal::ContactJacobians<T> contact_jacobians;
  contact_jacobians.Jc = MatrixX<T>::Zero(3 * nc, nv);
  contact_jacobians.Jn = MatrixX<T>::Zero(nc, nv);
  contact_jacobians.Jt = MatrixX<T>::Zero(2 * nc, nv);

  const SpanningForest& forest = get_forest();
  for (int i = 0; i < nc; ++i) {
    const int row_offset = 3 * i;
    const DiscreteContactPair<T>& contact_pair = contact_pairs[i];
    for (const typename DiscreteContactPair<T>::JacobianTreeBlock&
             tree_jacobian : contact_pair.jacobian) {
      const SpanningForest::Tree& tree = forest.trees(tree_jacobian.tree);
      contact_jacobians.Jc.block(row_offset, tree.v_start(), 3, tree.nv()) =
          tree_jacobian.J.MakeDenseMatrix();
    }
    contact_jacobians.Jt.middleRows(2 * i, 2) =
        contact_jacobians.Jc.middleRows(3 * i, 2);
    contact_jacobians.Jn.row(i) = contact_jacobians.Jc.row(3 * i + 2);
  }
  return contact_jacobians;
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

  // Compute non-contact forces at the previous time step. This also checks
  // whether an algebra loop exists but isn't detected when building the diagram
  // due to #12786. Therefore, we choose to do this before the quick exit when
  // there's no moving objects.
  MultibodyForces<T> forces0(plant());
  manager().CalcNonContactForces(
      context, /* include_joint_limit_penalty_forces */ true,
      /* include_pd_controlled_input */ true, &forces0);

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

  // Workspace for inverse dynamics:
  // Bodies' accelerations, ordered by MobodIndex.
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
  const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      manager().EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  // Compute normal and tangential velocity Jacobians at t0.
  const internal::ContactJacobians<T> contact_jacobians =
      CalcContactJacobians(context);

  // Get friction coefficient into a single vector.
  VectorX<T> mu(num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    mu[i] = contact_pairs[i].friction_coefficient;
  }

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
  const auto& indices =
      manager().EvalJointLocking(context).unlocked_velocity_indices;
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

template <typename T>
void TamsiDriver<T>::CalcAndAddSpatialContactForcesFromContactResults(
    const systems::Context<T>& context,
    const ContactResults<T>& contact_results,
    std::vector<SpatialForce<T>>* spatial_contact_forces) const {
  // Add contribution from point contact.
  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    const PointPairContactInfo<T>& pair =
        contact_results.point_pair_contact_info(i);
    const RigidBody<T>& bodyA = plant().get_body(pair.bodyA_index());
    const RigidBody<T>& bodyB = plant().get_body(pair.bodyB_index());
    const Vector3<T>& f_Bc_W = pair.contact_force();
    const Vector3<T>& p_WC = pair.contact_point();
    const SpatialForce<T> F_Bc_W(Vector3<T>::Zero(), f_Bc_W);

    // Contact spatial force on body A.
    const RigidTransform<T>& X_WA = plant().EvalBodyPoseInWorld(context, bodyA);
    const Vector3<T>& p_WA = X_WA.translation();
    const Vector3<T> p_CA_W = p_WA - p_WC;
    const SpatialForce<T> F_Ao_W = -F_Bc_W.Shift(p_CA_W);

    // Contact spatial force on body B.
    const RigidTransform<T>& X_WB = plant().EvalBodyPoseInWorld(context, bodyB);
    const Vector3<T>& p_WB = X_WB.translation();
    const Vector3<T> p_CB_W = p_WB - p_WC;
    const SpatialForce<T> F_Bo_W = F_Bc_W.Shift(p_CB_W);

    spatial_contact_forces->at(bodyA.mobod_index()) += F_Ao_W;
    spatial_contact_forces->at(bodyB.mobod_index()) += F_Bo_W;
  }

  // Add contribution from hydroelastic contact. When T = Expression, we skip
  // this because HydroelasticContactInfo is empty.
  if constexpr (scalar_predicate<T>::is_bool) {
    for (int i = 0; i < contact_results.num_hydroelastic_contacts(); ++i) {
      const HydroelasticContactInfo<T>& info =
          contact_results.hydroelastic_contact_info(i);

      const GeometryId geometryM_id = info.contact_surface().id_M();
      const GeometryId geometryN_id = info.contact_surface().id_N();
      const BodyIndex bodyA_index =
          manager().FindBodyByGeometryId(geometryM_id);
      const BodyIndex bodyB_index =
          manager().FindBodyByGeometryId(geometryN_id);
      const RigidBody<T>& bodyA = plant().get_body(bodyA_index);
      const RigidBody<T>& bodyB = plant().get_body(bodyB_index);

      // Spatial contact force at the centroid of the contact surface.
      const SpatialForce<T>& F_Ac_W = info.F_Ac_W();
      const Vector3<T>& p_WC = info.contact_surface().centroid();

      // Contact spatial force on body A.
      const RigidTransform<T>& X_WA =
          plant().EvalBodyPoseInWorld(context, bodyA);
      const Vector3<T>& p_WA = X_WA.translation();
      const Vector3<T> p_CA_W = p_WA - p_WC;
      const SpatialForce<T> F_Ao_W = F_Ac_W.Shift(p_CA_W);

      // Contact spatial force on body B.
      const RigidTransform<T>& X_WB =
          plant().EvalBodyPoseInWorld(context, bodyB);
      const Vector3<T>& p_WB = X_WB.translation();
      const Vector3<T> p_CB_W = p_WB - p_WC;
      const SpatialForce<T> F_Bo_W = -F_Ac_W.Shift(p_CB_W);

      spatial_contact_forces->at(bodyA.mobod_index()) += F_Ao_W;
      spatial_contact_forces->at(bodyB.mobod_index()) += F_Bo_W;
    }
  } else {
    DRAKE_DEMAND(contact_results.num_hydroelastic_contacts() == 0);
  }
}

template <typename T>
void TamsiDriver<T>::CalcDiscreteUpdateMultibodyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  manager().CalcNonContactForces(context,
                                 /* include_joint_limit_penalty_forces */ true,
                                 /* include_pd_controlled_input */ true,
                                 forces);
  // This Calc is somewhat inefficient. On the other hand, TAMSI is dispreferred
  // and is only kept for backwards compatibility, so we don't plan on trying to
  // optimize this.
  ContactResults<T> contact_results;
  manager().CalcContactResults(context, &contact_results);

  auto& Fapplied_Bo_W_array = forces->mutable_body_forces();
  CalcAndAddSpatialContactForcesFromContactResults(context, contact_results,
                                                   &Fapplied_Bo_W_array);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::TamsiDriver);
