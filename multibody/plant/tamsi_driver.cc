#include "drake/multibody/plant/tamsi_driver.h"

#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
TamsiDriver<T>::TamsiDriver(CompliantContactManager<T>* manager)
    : manager_(manager) {
  (void)manager;
  // Declare cache entries that TAMSI might need.
  // Maybe even allocate data/info TAMSI might need (joint locking stuff?)
}

template <typename T>
void TamsiDriver<T>::CalcContactSolverResults(
    const systems::Context<T>& context,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  const MultibodyPlant<T>& plant = manager().plant();

  const int nq = plant.num_positions();
  const int nv = plant.num_velocities();

  (void) nq;
  (void) nv;
  (void) context;
  (void) results;

#if 0
  // Quick exit if there are no moving objects.
  if (nv == 0) return;

  // Get the system state as raw Eigen vectors
  // (solution at the previous time step).
  auto x0 = context.get_discrete_state(0).get_value();
  VectorX<T> q0 = x0.topRows(nq);
  VectorX<T> v0 = x0.bottomRows(nv);

  // Mass matrix.
  MatrixX<T> M0(nv, nv);
  this->internal_tree().CalcMassMatrix(context, &M0);

  // Forces at the previous time step.
  MultibodyForces<T> forces0(this->internal_tree());

  CalcNonContactForces(context, true /* discrete */, &forces0);

  // Workspace for inverse dynamics:
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Body forces (alias to forces0).
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces0.mutable_body_forces();

  // With vdot = 0, this computes:
  //   -tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  VectorX<T>& minus_tau = forces0.mutable_generalized_forces();
  this->internal_tree().CalcInverseDynamics(
      context, vdot, F_BBo_W_array, minus_tau, &A_WB_array,
      &F_BBo_W_array, /* Note: these arrays get overwritten on output. */
      &minus_tau);

  // Compute all contact pairs, including both penetration pairs and quadrature
  // pairs for discrete hydroelastic.
  const std::vector<internal::DiscreteContactPair<T>>& contact_pairs =
      EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  // Compute normal and tangential velocity Jacobians at t0.
  const internal::ContactJacobians<T>& contact_jacobians =
      EvalContactJacobians(context);

  // Get friction coefficient into a single vector. Static friction is ignored
  // by the time stepping scheme.
  std::vector<CoulombFriction<double>> combined_friction_pairs =
      CalcCombinedFrictionCoefficients(context, contact_pairs);
  VectorX<T> mu(num_contacts);
  std::transform(combined_friction_pairs.begin(), combined_friction_pairs.end(),
                 mu.data(),
                 [](const CoulombFriction<double>& coulomb_friction) {
                   return coulomb_friction.dynamic_friction();
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
  const auto& indices = EvalJointLockingIndices(context);
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

  systems::CacheEntryValue& value =
      this->get_cache_entry(cache_indexes_.contact_solver_scratch)
          .get_mutable_cache_entry_value(context);
  auto& tamsi_solver = value.GetMutableValueOrThrow<TamsiSolver<T>>();
  if (tamsi_solver.get_solver_parameters().stiction_tolerance !=
      friction_model_.stiction_tolerance()) {
    // Set the stiction tolerance according to the values set by users with
    // set_stiction_tolerance().
    TamsiSolverParameters solver_parameters;
    solver_parameters.stiction_tolerance = friction_model_.stiction_tolerance();
    tamsi_solver.set_solver_parameters(solver_parameters);
  }

  // TAMSI is initialized with num_velocities(). Resize the internal solver
  // workspace if needed.
  tamsi_solver.ResizeIfNeeded(indices.size());

  CallTamsiSolver(&tamsi_solver, context.get_time(), v0_unlocked, M0_unlocked,
                  minus_tau_unlocked, fn0, Jn_unlocked, Jt_unlocked, stiffness,
                  damping, mu, &results_unlocked);

  // Joint locking: expand reduced outputs.
  results->v_next =
      ExpandRows(results_unlocked.v_next, num_velocities(), indices);
  results->tau_contact =
      contact_jacobians.Jn.transpose() * results_unlocked.fn +
      contact_jacobians.Jt.transpose() * results_unlocked.ft;

  results->fn = results_unlocked.fn;
  results->ft = results_unlocked.ft;
  results->vn = results_unlocked.vn;
  results->vt = results_unlocked.vt;
#endif  
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::TamsiDriver);