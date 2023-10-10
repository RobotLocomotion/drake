#include "drake/multibody/plant/compliant_contact_manager.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/scope_exit.h"
#include "drake/common/unused.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/tamsi_driver.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/systems/framework/context.h"

using drake::geometry::ContactSurface;
using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::contact_solvers::internal::ContactConfiguration;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::MatrixBlock;
using drake::multibody::internal::DiscreteContactPair;
using drake::multibody::internal::MultibodyTreeTopology;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
AccelerationsDueNonConstraintForcesCache<
    T>::AccelerationsDueNonConstraintForcesCache(const MultibodyTreeTopology&
                                                     topology)
    : forces(topology.num_bodies(), topology.num_velocities()),
      abic(topology),
      Zb_Bo_W(topology.num_bodies()),
      aba_forces(topology),
      ac(topology) {}

template <typename T>
CompliantContactManager<T>::CompliantContactManager() = default;

template <typename T>
CompliantContactManager<T>::~CompliantContactManager() = default;

template <typename T>
bool CompliantContactManager<T>::is_cloneable_to_double() const {
  return true;
}

template <typename T>
bool CompliantContactManager<T>::is_cloneable_to_autodiff() const {
  return true;
}

template <typename T>
bool CompliantContactManager<T>::is_cloneable_to_symbolic() const {
  return true;
}

template <typename T>
void CompliantContactManager<T>::set_sap_solver_parameters(
    const contact_solvers::internal::SapSolverParameters& parameters) {
  if constexpr (!std::is_same_v<T, symbolic::Expression>) {
    DRAKE_DEMAND(sap_driver_ != nullptr);
    sap_driver_->set_sap_solver_parameters(parameters);
  } else {
    unused(parameters);
    throw std::logic_error(
        "We do not provide SAP support T = symbolic::Expression. Therefore "
        "this method cannot be called.");
  }
}

template <typename T>
void CompliantContactManager<T>::DoDeclareCacheEntries() {
  // N.B. We use xd_ticket() instead of q_ticket() since discrete
  // multibody plant does not have q's, but rather discrete state.
  // Therefore if we make it dependent on q_ticket() the Jacobian only
  // gets evaluated once at the start of the simulation.

  // Cache hydroelastic contact info.
  const auto& hydroelastic_contact_info_cache_entry = this->DeclareCacheEntry(
      "Hydroelastic contact info.",
      systems::ValueProducer(
          this, &CompliantContactManager<T>::CalcHydroelasticContactInfo),
      // Compliant contact forces due to hydroelastics with Hunt &
      // Crosseley are function of the kinematic variables q & v only.
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.hydroelastic_contact_info =
      hydroelastic_contact_info_cache_entry.cache_index();

  // Accelerations due to non-constraint forces.
  // We cache non-contact forces, ABA forces and accelerations into an
  // AccelerationsDueNonConstraintForcesCache.
  AccelerationsDueNonConstraintForcesCache<T>
      non_constraint_forces_accelerations(this->internal_tree().get_topology());
  const auto base_cache_indices = DiscreteUpdateManager<T>::cache_indexes();
  const auto& discrete_input_port_forces_cache_entry =
      plant().get_cache_entry(base_cache_indices.discrete_input_port_forces);
  const auto& non_constraint_forces_accelerations_cache_entry =
      this->DeclareCacheEntry(
          "Non-constraint forces and induced accelerations.",
          systems::ValueProducer(
              this, non_constraint_forces_accelerations,
              &CompliantContactManager<
                  T>::CalcAccelerationsDueToNonConstraintForcesCache),
          {systems::System<T>::xd_ticket(),
           systems::System<T>::all_parameters_ticket(),
           discrete_input_port_forces_cache_entry.ticket()});
  cache_indexes_.non_constraint_forces_accelerations =
      non_constraint_forces_accelerations_cache_entry.cache_index();

  // Discrete updates with SAP are not supported when T = symbolic::Expression.
  if constexpr (!std::is_same_v<T, symbolic::Expression>) {
    if (sap_driver_ != nullptr) sap_driver_->DeclareCacheEntries(this);
  }
}

template <typename T>
VectorX<T> CompliantContactManager<T>::CalcEffectiveDamping(
    const systems::Context<T>& context) const {
  const VectorX<T> diagonal_inertia =
      plant().EvalReflectedInertiaCache(context) +
      joint_damping_ * plant().time_step();
  return diagonal_inertia;
}

template <typename T>
void CompliantContactManager<T>::CalcAccelerationsDueToNonConstraintForcesCache(
    const systems::Context<T>& context,
    AccelerationsDueNonConstraintForcesCache<T>* forward_dynamics_cache) const {
  DRAKE_DEMAND(forward_dynamics_cache != nullptr);

  // SAP models joint limits and actuation inputs (with effort limits) using
  // constraints. Therefore these terms are not included here since they are
  // included later as SAP constraints.
  this->CalcNonContactForces(
      context, /* include_joint_limit_penalty_forces */ false,
      /* include_pd_controlled_input */ false, &forward_dynamics_cache->forces);

  // Our goal is to compute accelerations from the Newton-Euler equations:
  //   M⋅v̇ = k(x)
  // where k(x) includes continuous forces of the state x not from constraints
  // such as force elements, Coriolis terms, actuation through input ports and
  // joint damping. We use a discrete time stepping scheme with time step dt
  // and accelerations
  //   v̇ = (v-v₀)/dt
  // where v₀ are the previous time step generalized velocities. We split
  // generalized forces k(x) as:
  //   k(x) = k₁(x) - D⋅v
  // where k₁(x) includes all other force contributions except damping and D
  // is the non-negative diagonal matrix for damping. Using this split, we
  // evaluate dissipation "implicitly" using the next time step velocities and
  // every other force in k₁(x) "explicitly" at the previous time step state
  // x₀. In total, our discrete update for the free motion velocities reads:
  //   M⋅(v-v₀)/dt = k₁(x₀) - D⋅v
  // We can rewrite this by adding and subtracting -D⋅v₀ on the right hand
  // side:
  //   M⋅(v-v₀)/dt = k₁(x₀) - D⋅(v-v₀) - D⋅v₀
  // which can be rearranged as:
  //   (M + dt⋅D)⋅(v-v₀)/dt = k₁(x₀) - D⋅v₀ = k(x₀)
  // Therefore the generalized accelerations a = (v-v₀)/dt can be computed
  // using ABA forward dynamics with non-constraint continuous forces
  // evaluated at x₀ and the addition of the diagonal term dt⋅D. We do this
  // below in terms of MultibodyTree APIs.

  // We must include reflected rotor inertias along with the new term dt⋅D.
  const VectorX<T> diagonal_inertia = CalcEffectiveDamping(context);

  // We compute the articulated body inertia including the contribution of the
  // additional diagonal elements arising from the implicit treatment of joint
  // damping.
  this->internal_tree().CalcArticulatedBodyInertiaCache(
      context, diagonal_inertia, &forward_dynamics_cache->abic);
  this->internal_tree().CalcArticulatedBodyForceBias(
      context, forward_dynamics_cache->abic, &forward_dynamics_cache->Zb_Bo_W);
  this->internal_tree().CalcArticulatedBodyForceCache(
      context, forward_dynamics_cache->abic, forward_dynamics_cache->Zb_Bo_W,
      forward_dynamics_cache->forces, &forward_dynamics_cache->aba_forces);
  this->internal_tree().CalcArticulatedBodyAccelerations(
      context, forward_dynamics_cache->abic, forward_dynamics_cache->aba_forces,
      &forward_dynamics_cache->ac);
}

template <typename T>
const multibody::internal::AccelerationKinematicsCache<T>&
CompliantContactManager<T>::EvalAccelerationsDueToNonConstraintForcesCache(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.non_constraint_forces_accelerations)
      .template Eval<AccelerationsDueNonConstraintForcesCache<T>>(context)
      .ac;
}

template <typename T>
void CompliantContactManager<T>::DoCalcContactSolverResults(
    const systems::Context<T>& context,
    ContactSolverResults<T>* contact_results) const {
  if (plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap) {
    if constexpr (std::is_same_v<T, symbolic::Expression>) {
      throw std::logic_error(
          "Discrete updates with the SAP solver are not supported for T = "
          "symbolic::Expression");
    } else {
      DRAKE_DEMAND(sap_driver_ != nullptr);
      sap_driver_->CalcContactSolverResults(context, contact_results);
    }
  }

  if (plant().get_discrete_contact_solver() == DiscreteContactSolver::kTamsi) {
    DRAKE_DEMAND(tamsi_driver_ != nullptr);
    tamsi_driver_->CalcContactSolverResults(context, contact_results);
  }
}

template <typename T>
void CompliantContactManager<T>::AppendContactResultsForPointContact(
    const drake::systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      plant().EvalPointPairPenetrations(context);
  const DiscreteContactData<DiscreteContactPair<T>>& discrete_pairs =
      this->EvalDiscreteContactPairs(context);
  const DiscreteContactData<ContactPairKinematics<T>>& contact_kinematics =
      this->EvalContactKinematics(context);
  const contact_solvers::internal::ContactSolverResults<T>& solver_results =
      this->EvalContactSolverResults(context);

  const VectorX<T>& fn = solver_results.fn;
  const VectorX<T>& ft = solver_results.ft;
  const VectorX<T>& vt = solver_results.vt;
  const VectorX<T>& vn = solver_results.vn;

  const int num_point_contacts = discrete_pairs.num_point_contacts();

  DRAKE_DEMAND(fn.size() >= num_point_contacts);
  DRAKE_DEMAND(ft.size() >= 2 * num_point_contacts);
  DRAKE_DEMAND(vn.size() >= num_point_contacts);
  DRAKE_DEMAND(vt.size() >= 2 * num_point_contacts);

  for (int icontact = 0; icontact < num_point_contacts; ++icontact) {
    const auto& discrete_pair = discrete_pairs[icontact];
    const auto& point_pair = point_pairs[icontact];

    const GeometryId geometryA_id = discrete_pair.id_A;
    const GeometryId geometryB_id = discrete_pair.id_B;

    const BodyIndex bodyA_index = this->FindBodyByGeometryId(geometryA_id);
    const BodyIndex bodyB_index = this->FindBodyByGeometryId(geometryB_id);

    const RotationMatrix<T>& R_WC =
        contact_kinematics[icontact].configuration.R_WC;

    // Contact forces applied on B at contact point C.
    const Vector3<T> f_Bc_C(ft(2 * icontact), ft(2 * icontact + 1),
                            fn(icontact));
    const Vector3<T> f_Bc_W = R_WC * f_Bc_C;

    // Slip velocity.
    const T slip = vt.template segment<2>(2 * icontact).norm();

    // Separation velocity in the normal direction.
    const T separation_velocity = vn(icontact);

    // Add pair info to the contact results.
    contact_results->AddContactInfo({bodyA_index, bodyB_index, f_Bc_W,
                                     discrete_pair.p_WC, separation_velocity,
                                     slip, point_pair});
  }
}

template <typename T>
void CompliantContactManager<T>::AppendContactResultsForHydroelasticContact(
    const drake::systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  const std::vector<HydroelasticContactInfo<T>>& contact_info =
      this->EvalHydroelasticContactInfo(context);

  for (const HydroelasticContactInfo<T>& info : contact_info) {
    // Note: caching dependencies guarantee that the lifetime of `info` is
    // valid for the lifetime of the contact results.
    contact_results->AddContactInfo(&info);
  }
}

template <typename T>
void CompliantContactManager<T>::CalcHydroelasticContactInfo(
    const systems::Context<T>& context,
    std::vector<HydroelasticContactInfo<T>>* contact_info) const {
  DRAKE_DEMAND(contact_info != nullptr);

  const std::vector<ContactSurface<T>>& all_surfaces =
      this->EvalContactSurfaces(context);

  // Reserve memory here to keep from repeatedly allocating heap storage in
  // the loop below.
  // TODO(joemasterjohn): Consider caching this vector and the quadrature
  // point data vectors to avoid this dynamic allocation.
  contact_info->clear();
  contact_info->reserve(all_surfaces.size());

  const DiscreteContactData<DiscreteContactPair<T>>& discrete_pairs =
      this->EvalDiscreteContactPairs(context);
  const DiscreteContactData<ContactPairKinematics<T>>& contact_kinematics =
      this->EvalContactKinematics(context);

  const contact_solvers::internal::ContactSolverResults<T>& solver_results =
      this->EvalContactSolverResults(context);

  const VectorX<T>& fn = solver_results.fn;
  const VectorX<T>& ft = solver_results.ft;
  const VectorX<T>& vt = solver_results.vt;
  const VectorX<T>& vn = solver_results.vn;

  // Discrete pairs contain both point and hydro contact force results.
  const int num_contacts = discrete_pairs.size();
  DRAKE_DEMAND(fn.size() == num_contacts);
  DRAKE_DEMAND(ft.size() == 2 * num_contacts);
  DRAKE_DEMAND(vn.size() == num_contacts);
  DRAKE_DEMAND(vt.size() == 2 * num_contacts);

  const int num_point_contacts = discrete_pairs.num_point_contacts();
  const int num_hydro_contacts = discrete_pairs.num_hydro_contacts();
  const int num_surfaces = all_surfaces.size();

  std::vector<SpatialForce<T>> F_Ao_W_per_surface(num_surfaces,
                                                  SpatialForce<T>::Zero());

  std::vector<std::vector<HydroelasticQuadraturePointData<T>>> quadrature_data(
      num_surfaces);
  for (int isurface = 0; isurface < num_surfaces; ++isurface) {
    quadrature_data[isurface].reserve(all_surfaces[isurface].num_faces());
  }

  // We only scan discrete pairs corresponding to hydroelastic quadrature
  // points. These are appended by CalcDiscreteContactPairs() at the end of the
  // point contact forces.
  for (int icontact = num_point_contacts;
       icontact < num_point_contacts + num_hydro_contacts; ++icontact) {
    const auto& pair = discrete_pairs[icontact];
    // Quadrature point Q.
    const Vector3<T>& p_WQ = pair.p_WC;
    const RotationMatrix<T>& R_WC =
        contact_kinematics[icontact].configuration.R_WC;

    // Contact forces applied on B at quadrature point Q expressed in the
    // contact frame.
    const Vector3<T> f_Bq_C(ft(2 * icontact), ft(2 * icontact + 1),
                            fn(icontact));
    // Contact force applied on A at quadrature point Q expressed in the world
    // frame.
    const Vector3<T> f_Aq_W = -(R_WC * f_Bq_C);

    const int surface_index = pair.surface_index.value();
    const auto& s = all_surfaces[surface_index];
    // Surface's centroid point O.
    const Vector3<T>& p_WO = s.is_triangle() ? s.tri_mesh_W().centroid()
                                             : s.poly_mesh_W().centroid();

    // Spatial force
    const Vector3<T> p_QO_W = p_WO - p_WQ;
    const SpatialForce<T> Fq_Ao_W =
        SpatialForce<T>(Vector3<T>::Zero(), f_Aq_W).Shift(p_QO_W);
    // Accumulate force for the corresponding contact surface.
    F_Ao_W_per_surface[surface_index] += Fq_Ao_W;

    // Velocity of Aq relative to Bq in the tangent direction.
    // N.B. DiscreteUpdateManager<T>::CalcContactKinematics() uses the
    // convention of computing J_AcBc_C and thus J_AcBc_C * v = v_AcBc_W (i.e.
    // relative velocity of Bc with respect to Ac). Thus we flip the sign here
    // for the convention used by HydroelasticQuadratureData.
    const Vector3<T> vt_BqAq_C(-vt(2 * icontact), -vt(2 * icontact + 1), 0);
    const Vector3<T> vt_BqAq_W = R_WC * vt_BqAq_C;

    // Traction vector applied to body A at point Aq (Aq and Bq are coincident)
    // expressed in the world frame.
    const int face_index = pair.face_index.value();
    const Vector3<T> traction_Aq_W = f_Aq_W / s.area(face_index);

    quadrature_data[surface_index].emplace_back(p_WQ, face_index, vt_BqAq_W,
                                                traction_Aq_W);
  }

  const MultibodyTreeTopology& topology = this->internal_tree().get_topology();
  const std::vector<std::vector<int>>& per_tree_unlocked_indices =
      this->EvalJointLockingCache(context).unlocked_velocity_indices_per_tree;

  // Update contact info to include the correct contact forces.
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    BodyIndex bodyA_index = this->geometry_id_to_body_index().at(
        all_surfaces[surface_index].id_M());
    BodyIndex bodyB_index = this->geometry_id_to_body_index().at(
        all_surfaces[surface_index].id_N());

    const TreeIndex& treeA_index = topology.body_to_tree_index(bodyA_index);
    const TreeIndex& treeB_index = topology.body_to_tree_index(bodyB_index);

    //  For joint locking, filter out contacts between bodies who belong to
    //  trees with 0 degrees of freedom. For a contact to remain in
    //  consideration, at least one of the trees involved has to be valid and
    //  have a non-zero number of DOFs.
    if ((treeA_index.is_valid() &&
         per_tree_unlocked_indices[treeA_index].size() != 0) ||
        (treeB_index.is_valid() &&
         per_tree_unlocked_indices[treeB_index].size() != 0)) {
      contact_info->emplace_back(&all_surfaces[surface_index],
                                 F_Ao_W_per_surface[surface_index],
                                 std::move(quadrature_data[surface_index]));
    }
  }
}

template <typename T>
const std::vector<HydroelasticContactInfo<T>>&
CompliantContactManager<T>::EvalHydroelasticContactInfo(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.hydroelastic_contact_info)
      .template Eval<std::vector<HydroelasticContactInfo<T>>>(context);
}

template <typename T>
void CompliantContactManager<T>::DoCalcContactResults(
    const drake::systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);
  contact_results->Clear();
  contact_results->set_plant(&plant());

  if (plant().num_collision_geometries() == 0) return;

  switch (plant().get_contact_model()) {
    case ContactModel::kPoint:
      AppendContactResultsForPointContact(context, contact_results);
      break;
    case ContactModel::kHydroelastic:
      AppendContactResultsForHydroelasticContact(context, contact_results);
      break;
    case ContactModel::kHydroelasticWithFallback:
      AppendContactResultsForPointContact(context, contact_results);
      AppendContactResultsForHydroelasticContact(context, contact_results);
      break;
  }
}

// TODO(xuchenhan-tri): Consider a scalar converting constructor to cut down
// repeated code in CloneToDouble() and CloneToAutoDiffXd().
template <typename T>
std::unique_ptr<DiscreteUpdateManager<double>>
CompliantContactManager<T>::CloneToDouble() const {
  // Create a manager with default SAP parameters.
  auto clone = std::make_unique<CompliantContactManager<double>>();
  // N.B. we should copy/clone all members except for those overwritten in
  // ExtractModelInfo and DeclareCacheEntries.
  // E.g. SapParameters for SapDriver won't be the same after the clone.
  return clone;
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>>
CompliantContactManager<T>::CloneToAutoDiffXd() const {
  // Create a manager with default SAP parameters.
  auto clone = std::make_unique<CompliantContactManager<AutoDiffXd>>();
  // N.B. we should copy/clone all members except for those overwritten in
  // ExtractModelInfo and DeclareCacheEntries.
  // E.g. SapParameters for SapDriver won't be the same after the clone.
  return clone;
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
CompliantContactManager<T>::CloneToSymbolic() const {
  // Create a manager with default SAP parameters.
  auto clone =
      std::make_unique<CompliantContactManager<symbolic::Expression>>();
  // N.B. we should copy/clone all members except for those overwritten in
  // ExtractModelInfo and DeclareCacheEntries.
  // E.g. SapParameters for SapDriver won't be the same after the clone.
  return clone;
}

template <typename T>
void CompliantContactManager<T>::DoExtractModelInfo() {
  // Collect joint damping coefficients into a vector.
  joint_damping_ = VectorX<T>::Zero(plant().num_velocities());
  for (JointIndex j(0); j < plant().num_joints(); ++j) {
    const Joint<T>& joint = plant().get_joint(j);
    const int velocity_start = joint.velocity_start();
    const int nv = joint.num_velocities();
    joint_damping_.segment(velocity_start, nv) = joint.damping_vector();
  }

  // Solver drivers are only created when ExtractModelInfo() is called and
  // therefore we expect these pointers to equal nullptr. The only reason for
  // one of them to be non-nullptr would be a bug leading to this method being
  // called more than once on the same manager.
  DRAKE_DEMAND(sap_driver_ == nullptr && tamsi_driver_ == nullptr);

  switch (plant().get_discrete_contact_solver()) {
    case DiscreteContactSolver::kSap:
      // N.B. SAP is not supported for T = symbolic::Expression.
      // However, exception will only be thrown if we attempt to use a SapDriver
      // to compute discrete updates. This allows a user to scalar convert a
      // plant to symbolic and perform other supported queries such as
      // introspection and kinematics.
      if constexpr (!std::is_same_v<T, symbolic::Expression>) {
        const double near_rigid_threshold =
            plant().get_sap_near_rigid_threshold();
        sap_driver_ =
            std::make_unique<SapDriver<T>>(this, near_rigid_threshold);
      }
      break;
    case DiscreteContactSolver::kTamsi:
      // N.B. We do allow discrete updates with TAMSI when T =
      // symbolic::Expression, but only when there is no contact.
      tamsi_driver_ = std::make_unique<TamsiDriver<T>>(this);
      break;
  }
}

template <typename T>
void CompliantContactManager<T>::DoCalcAccelerationKinematicsCache(
    const systems::Context<T>& context0,
    multibody::internal::AccelerationKinematicsCache<T>* ac) const {
  // Current state.
  const VectorX<T>& x0 =
      context0.get_discrete_state(this->multibody_state_index()).value();
  const auto v0 = x0.bottomRows(plant().num_velocities());

  // Next state.
  const ContactSolverResults<T>& results =
      this->EvalContactSolverResults(context0);
  const VectorX<T>& v_next = results.v_next;

  ac->get_mutable_vdot() = (v_next - v0) / plant().time_step();

  this->internal_tree().CalcSpatialAccelerationsFromVdot(
      context0, plant().EvalPositionKinematics(context0),
      plant().EvalVelocityKinematics(context0), ac->get_vdot(),
      &ac->get_mutable_A_WB_pool());
}

template <typename T>
void CompliantContactManager<T>::DoCalcDiscreteUpdateMultibodyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  // Thus far only TAMSI and SAP are supported. Verify this is true.
  DRAKE_DEMAND(
      plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap ||
      plant().get_discrete_contact_solver() == DiscreteContactSolver::kTamsi);

  // Delegate to specific solver driver.
  if (plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap) {
    if constexpr (std::is_same_v<T, symbolic::Expression>) {
      throw std::logic_error(
          "Discrete updates with the SAP solver are not supported for T = "
          "symbolic::Expression");
    } else {
      DRAKE_DEMAND(sap_driver_ != nullptr);
      sap_driver_->CalcDiscreteUpdateMultibodyForces(context, forces);
    }
  }

  if (plant().get_discrete_contact_solver() == DiscreteContactSolver::kTamsi) {
    DRAKE_DEMAND(tamsi_driver_ != nullptr);
    tamsi_driver_->CalcDiscreteUpdateMultibodyForces(context, forces);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
