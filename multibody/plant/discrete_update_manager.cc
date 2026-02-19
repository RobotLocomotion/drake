#include "drake/multibody/plant/discrete_update_manager.h"

#include <limits>
#include <utility>

#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::geometry::ContactSurface;
using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::MatrixBlock;
using drake::systems::Context;
using drake::systems::DependencyTicket;

template <typename T>
DiscreteUpdateManager<T>::~DiscreteUpdateManager() = default;

template <typename T>
void DiscreteUpdateManager<T>::CalcDiscreteValues(
    const systems::Context<T>& context, systems::DiscreteValues<T>* updates,
    DiscreteStepMemory::Data<T>* memory) const {
  DRAKE_DEMAND(updates != nullptr);
  // Perform discrete updates for deformable bodies if they exist.
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->CalcDiscreteStates(context, updates);
    }
  }
  // Perform discrete updates for rigid bodies.
  DoCalcDiscreteValues(context, updates);

  // TODO(jwnimmer-tri) Possibly there is a way to directly calculate these
  // entries into the Memory storage, to avoid this extra copying?
  if (memory != nullptr) {
    memory->net_actuation = EvalActuation(context);
    memory->geometry_contact_data = EvalGeometryContactData(context);
    memory->acceleration_kinematics_cache =
        plant().EvalForwardDynamics(context);
    memory->discrete_contact_pairs = EvalDiscreteContactPairs(context);
    memory->contact_solver_results = EvalContactSolverResults(context);
  }
}

template <typename T>
void DiscreteUpdateManager<T>::DeclareCacheEntries() {
  const auto& query_object_input_ticket =
      plant().get_geometry_query_input_port().ticket();
  const auto& contact_solver_results_cache_entry = DeclareCacheEntry(
      "Contact solver results",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcContactSolverResults),
      // ContactSolverResults include contribution from force elements, which
      // could involve user-injected dependencies. So we need to include all
      // possible tickets that users can choose to depend on.
      {systems::System<T>::all_input_ports_ticket(),
       systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket(),
       systems::System<T>::time_ticket(),
       systems::System<T>::accuracy_ticket()});
  cache_indexes_.contact_solver_results =
      contact_solver_results_cache_entry.cache_index();

  // See ThrowIfNonContactForceInProgress().
  const auto& non_contact_forces_evaluation_in_progress = DeclareCacheEntry(
      "Evaluation of non-contact forces and accelerations is in progress",
      // N.B. This flag is set to true only when the computation is in
      // progress. Therefore its default value is `false`.
      systems::ValueProducer(false, &systems::ValueProducer::NoopCalc),
      {systems::System<T>::nothing_ticket()});
  cache_indexes_.non_contact_forces_evaluation_in_progress =
      non_contact_forces_evaluation_in_progress.cache_index();

  MultibodyForces<T> model_forces(plant());
  const auto& multibody_forces_cache_entry = DeclareCacheEntry(
      "Discrete update multibody forces",
      systems::ValueProducer(
          this, model_forces,
          &DiscreteUpdateManager<T>::CalcDiscreteUpdateMultibodyForces),
      {systems::System<T>::all_input_ports_ticket(),
       systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.discrete_update_multibody_forces =
      multibody_forces_cache_entry.cache_index();

  const auto& actuation_cache_entry = DeclareCacheEntry(
      "Discrete update actuation",
      systems::ValueProducer(this, VectorX<T>(plant().num_actuated_dofs()),
                             &DiscreteUpdateManager<T>::CalcActuation),
      {systems::System<T>::all_input_ports_ticket(),
       systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.actuation = actuation_cache_entry.cache_index();

  // Cache discrete contact pairs.
  const auto& discrete_contact_pairs_cache_entry = DeclareCacheEntry(
      "Discrete contact pairs.",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcDiscreteContactPairs),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket(), query_object_input_ticket});
  cache_indexes_.discrete_contact_pairs =
      discrete_contact_pairs_cache_entry.cache_index();

  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->DeclareCacheEntries(this);
    }
  }

  // Allow derived classes to declare their own cache entries.
  DoDeclareCacheEntries();
}

template <typename T>
const GeometryContactData<T>& DiscreteUpdateManager<T>::EvalGeometryContactData(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::EvalGeometryContactData(plant(), context);
}

template <typename T>
systems::CacheEntry& DiscreteUpdateManager<T>::DeclareCacheEntry(
    std::string description, systems::ValueProducer value_producer,
    std::set<DependencyTicket> prerequisites_of_calc) {
  DRAKE_DEMAND(mutable_plant_ != nullptr);
  DRAKE_DEMAND(mutable_plant_ == plant_);
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::DeclareCacheEntry(
      mutable_plant_, std::move(description), std::move(value_producer),
      std::move(prerequisites_of_calc));
}

template <typename T>
double DiscreteUpdateManager<T>::default_contact_stiffness() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::default_contact_stiffness(plant());
}

template <typename T>
double DiscreteUpdateManager<T>::default_contact_dissipation() const {
  // TODO(xuchenhan-tri): The plant's default contact dissipation may be NaN,
  // We guard against that here as a band-aid fix. See #19981.
  const double plant_dissipation = MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::default_contact_dissipation(plant());
  if (!(plant_dissipation >= 0.0)) {
    return 0.0;
  }
  return plant_dissipation;
}

template <typename T>
BodyIndex DiscreteUpdateManager<T>::FindBodyByGeometryId(
    geometry::GeometryId geometry_id) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::FindBodyByGeometryId(
      plant(), geometry_id);
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<double>>
DiscreteUpdateManager<T>::CloneToDouble() const {
  throw std::logic_error(
      "Scalar conversion to double is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>>
DiscreteUpdateManager<T>::CloneToAutoDiffXd() const {
  throw std::logic_error(
      "Scalar conversion to AutodiffXd is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
DiscreteUpdateManager<T>::CloneToSymbolic() const {
  throw std::logic_error(
      "Scalar conversion to symbolic::Expression is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_double() const {
  return false;
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_autodiff() const {
  return false;
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_symbolic() const {
  return false;
}

template <typename T>
const MultibodyPlant<T>& DiscreteUpdateManager<T>::plant() const {
  DRAKE_DEMAND(plant_ != nullptr);
  return *plant_;
}

template <typename T>
void DiscreteUpdateManager<T>::CalcNonContactForces(
    const drake::systems::Context<T>& context,
    bool include_joint_limit_penalty_forces, bool include_pd_controlled_input,
    MultibodyForces<T>* forces) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(plant()));

  // Compute forces applied through force elements. Note that this resets
  // forces to empty so must come first.
  CalcForceElementsContribution(context, forces);

  // Incorporate all input forces.
  InputPortForces<T> inputs(plant());
  CalcInputPortForces(context, &inputs);

  // Copy into `forces` as requested.
  forces->AddInForces(inputs.externally_applied_forces);
  if (include_joint_limit_penalty_forces) {
    AddJointLimitsPenaltyForces(context, forces);
  }
  // PD controlled actuation is included only when requested.
  forces->mutable_generalized_forces() += inputs.actuation_wo_pd;
  if (include_pd_controlled_input) {
    forces->mutable_generalized_forces() += inputs.actuation_w_pd;
  }
}

template <typename T>
const ContactSolverResults<T>&
DiscreteUpdateManager<T>::EvalContactSolverResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_solver_results)
      .template Eval<ContactSolverResults<T>>(context);
}

template <typename T>
void DiscreteUpdateManager<T>::AddJointLimitsPenaltyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::AddJointLimitsPenaltyForces(
      plant(), context, forces);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcForceElementsContribution(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CalcForceElementsContribution(
      plant(), context, forces);
}

template <typename T>
const internal::JointLockingCacheData<T>&
DiscreteUpdateManager<T>::EvalJointLocking(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalJointLocking(
      plant(), context);
}

template <typename T>
const VectorX<T>& DiscreteUpdateManager<T>::EvalActuationInput(
    const systems::Context<T>& context, bool effort_limit) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalActuationInput(
      plant(), context, effort_limit);
}

template <typename T>
const DesiredStateInput<T>& DiscreteUpdateManager<T>::EvalDesiredStateInput(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalDesiredStateInput(
      plant(), context);
}

template <typename T>
const std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
DiscreteUpdateManager<T>::coupler_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::coupler_constraints_specs(*plant_);
}

template <typename T>
const std::map<MultibodyConstraintId, DistanceConstraintParams>&
DiscreteUpdateManager<T>::GetDistanceConstraintParams(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::GetDistanceConstraintParams(*plant_, context);
}

template <typename T>
const std::map<MultibodyConstraintId, internal::BallConstraintSpec>&
DiscreteUpdateManager<T>::ball_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::ball_constraints_specs(
      *plant_);
}

template <typename T>
const std::map<MultibodyConstraintId, internal::WeldConstraintSpec>&
DiscreteUpdateManager<T>::weld_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::weld_constraints_specs(
      *plant_);
}

template <typename T>
const std::map<MultibodyConstraintId, internal::TendonConstraintSpec>&
DiscreteUpdateManager<T>::tendon_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::tendon_constraints_specs(*plant_);
}

template <typename T>
const std::map<MultibodyConstraintId, bool>&
DiscreteUpdateManager<T>::GetConstraintActiveStatus(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::GetConstraintActiveStatus(context, *plant_);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcJointActuationForces(
    const systems::Context<T>& context, VectorX<T>* actuation_w_pd,
    VectorX<T>* actuation_wo_pd) const {
  DRAKE_DEMAND(actuation_w_pd != nullptr);
  DRAKE_DEMAND(actuation_w_pd->size() == plant().num_velocities());
  DRAKE_DEMAND(actuation_wo_pd != nullptr);
  DRAKE_DEMAND(actuation_wo_pd->size() == plant().num_velocities());
  actuation_w_pd->setZero();
  actuation_wo_pd->setZero();
  if (plant().num_actuators() > 0) {
    const VectorX<T>& u =
        EvalActuationInput(context, /* effort_limit = */ true);
    for (JointActuatorIndex actuator_index :
         plant().GetJointActuatorIndices()) {
      const JointActuator<T>& actuator =
          plant().get_joint_actuator(actuator_index);
      const Joint<T>& joint = actuator.joint();
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(joint.num_velocities() == 1);
      const int v_index = joint.velocity_start();
      VectorX<T>& actuation =
          actuator.has_controller() ? *actuation_w_pd : *actuation_wo_pd;
      actuation[v_index] = u[actuator.input_start()];
    }
  }
}

template <typename T>
void DiscreteUpdateManager<T>::CalcInputPortForces(
    const systems::Context<T>& context, InputPortForces<T>* forces) const {
  forces->SetZero();
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::
      AddAppliedExternalGeneralizedForces(plant(), context,
                                          &forces->externally_applied_forces);
  MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::AddAppliedExternalSpatialForces(plant(), context,
                                          &forces->externally_applied_forces);
  CalcJointActuationForces(context, &forces->actuation_w_pd,
                           &forces->actuation_wo_pd);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcContactResults(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  const GeometryContactData<T>& geometry_contact_data =
      EvalGeometryContactData(context);
  const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      EvalDiscreteContactPairs(context);
  const ContactSolverResults<T>& solver_results =
      EvalContactSolverResults(context);
  CalcContactResultsImpl(geometry_contact_data, contact_pairs, solver_results,
                         contact_results);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcContactResults(
    const DiscreteStepMemory::Data<T>& memory,
    ContactResults<T>* contact_results) const {
  CalcContactResultsImpl(memory.geometry_contact_data,
                         memory.discrete_contact_pairs,
                         memory.contact_solver_results, contact_results);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcContactResultsImpl(
    const GeometryContactData<T>& geometry_contact_data,
    const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
    const ContactSolverResults<T>& solver_results,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);

  std::vector<PointPairContactInfo<T>> contact_results_point_pair;
  CalcContactResultsForPointContact(geometry_contact_data, contact_pairs,
                                    solver_results,
                                    &contact_results_point_pair);

  std::vector<HydroelasticContactInfo<T>> contact_results_hydroelastic;
  CalcContactResultsForHydroelasticContact(geometry_contact_data, contact_pairs,
                                           solver_results,
                                           &contact_results_hydroelastic);

  std::vector<DeformableContactInfo<T>> contact_results_deformable;
  CalcContactResultsForDeformableContact(geometry_contact_data, contact_pairs,
                                         solver_results,
                                         &contact_results_deformable);

  std::shared_ptr<const void> backing_store = geometry_contact_data.Share();

  *contact_results = ContactResults<T>(std::move(contact_results_point_pair),
                                       std::move(contact_results_hydroelastic),
                                       std::move(contact_results_deformable),
                                       std::move(backing_store));
  contact_results->set_plant(&plant());
}

template <typename T>
void DiscreteUpdateManager<T>::CalcContactResultsForPointContact(
    const GeometryContactData<T>& geometry_contact_data,
    const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
    const ContactSolverResults<T>& solver_results,
    std::vector<PointPairContactInfo<T>>* contact_results_point_pair) const {
  DRAKE_DEMAND(contact_results_point_pair != nullptr);
  contact_results_point_pair->clear();

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      geometry_contact_data.get().point_pairs;
  if (point_pairs.empty()) {
    return;
  }

  const VectorX<T>& fn = solver_results.fn;
  const VectorX<T>& ft = solver_results.ft;
  const VectorX<T>& vt = solver_results.vt;
  const VectorX<T>& vn = solver_results.vn;

  const int num_point_contacts = contact_pairs.num_point_contacts();

  DRAKE_DEMAND(fn.size() >= num_point_contacts);
  DRAKE_DEMAND(ft.size() >= 2 * num_point_contacts);
  DRAKE_DEMAND(vn.size() >= num_point_contacts);
  DRAKE_DEMAND(vt.size() >= 2 * num_point_contacts);

  contact_results_point_pair->reserve(num_point_contacts);
  for (int icontact = 0; icontact < num_point_contacts; ++icontact) {
    const DiscreteContactPair<T>& pair = contact_pairs[icontact];

    DRAKE_DEMAND(pair.point_pair_index.has_value());
    const auto& point_pair = point_pairs[pair.point_pair_index.value()];

    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index = FindBodyByGeometryId(geometryA_id);
    const BodyIndex bodyB_index = FindBodyByGeometryId(geometryB_id);

    const RotationMatrix<T>& R_WC = pair.R_WC;

    // Contact forces applied on B at contact point C.
    const Vector3<T> f_Bc_C(ft(2 * icontact), ft(2 * icontact + 1),
                            fn(icontact));
    const Vector3<T> f_Bc_W = R_WC * f_Bc_C;

    // Slip velocity.
    const T slip = vt.template segment<2>(2 * icontact).norm();

    // Separation velocity in the normal direction.
    const T separation_velocity = vn(icontact);

    // Add pair info to the contact results.
    contact_results_point_pair->emplace_back(bodyA_index, bodyB_index, f_Bc_W,
                                             pair.p_WC, separation_velocity,
                                             slip, point_pair);
  }
}

template <typename T>
void DiscreteUpdateManager<T>::CalcContactResultsForHydroelasticContact(
    const GeometryContactData<T>& geometry_contact_data,
    const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
    const ContactSolverResults<T>& solver_results,
    std::vector<HydroelasticContactInfo<T>>* contact_results_hydroelastic)
    const {
  DRAKE_DEMAND(contact_results_hydroelastic != nullptr);
  contact_results_hydroelastic->clear();

  if constexpr (!std::is_same_v<T, symbolic::Expression>) {
    const std::vector<ContactSurface<T>>& all_surfaces =
        geometry_contact_data.get().surfaces;
    if (all_surfaces.empty()) {
      return;
    }

    const VectorX<T>& fn = solver_results.fn;
    const VectorX<T>& ft = solver_results.ft;
    const VectorX<T>& vt = solver_results.vt;
    const VectorX<T>& vn = solver_results.vn;

    // Discrete pairs contain point, hydro, and deformable contact force
    // results.
    const int num_contacts = contact_pairs.size();
    DRAKE_DEMAND(fn.size() == num_contacts);
    DRAKE_DEMAND(ft.size() == 2 * num_contacts);
    DRAKE_DEMAND(vn.size() == num_contacts);
    DRAKE_DEMAND(vt.size() == 2 * num_contacts);

    const int num_point_contacts = contact_pairs.num_point_contacts();
    const int num_hydro_contacts = contact_pairs.num_hydro_contacts();
    const int num_surfaces = all_surfaces.size();

    std::vector<SpatialForce<T>> F_Ao_W_per_surface(num_surfaces,
                                                    SpatialForce<T>::Zero());
    // We only scan discrete pairs corresponding to hydroelastic quadrature
    // points.
    for (int icontact = num_point_contacts;
         icontact < num_point_contacts + num_hydro_contacts; ++icontact) {
      const DiscreteContactPair<T>& pair = contact_pairs[icontact];
      // Quadrature point Q.
      const Vector3<T>& p_WQ = pair.p_WC;
      const RotationMatrix<T>& R_WC = pair.R_WC;

      // Contact forces applied on B at quadrature point Q expressed in the
      // contact frame.
      const Vector3<T> f_Bq_C(ft(2 * icontact), ft(2 * icontact + 1),
                              fn(icontact));
      // Contact force applied on A at quadrature point Q expressed in the world
      // frame.
      const Vector3<T> f_Aq_W = -(R_WC * f_Bq_C);

      DRAKE_DEMAND(pair.surface_index.has_value());
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
    }

    // Update contact info to include the correct contact forces.
    contact_results_hydroelastic->reserve(num_surfaces);
    for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
      // Here we use emplacement to add a HydroelasticContactInfo with a raw
      // pointer back into the GeometryContactData. This is safe only because
      // our caller adds the GeometryContactData to the "backing store" as a
      // keep-alive.
      contact_results_hydroelastic->emplace_back(
          &all_surfaces[surface_index], F_Ao_W_per_surface[surface_index]);
    }
  }
}

template <typename T>
void DiscreteUpdateManager<T>::CalcContactResultsForDeformableContact(
    const GeometryContactData<T>& geometry_contact_data,
    const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
    const ContactSolverResults<T>& solver_results,
    std::vector<DeformableContactInfo<T>>* contact_results_deformable) const {
  DRAKE_DEMAND(contact_results_deformable != nullptr);
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->CalcDeformableContactInfo(
          geometry_contact_data.get().deformable, contact_pairs, solver_results,
          contact_results_deformable);
    }
  } else {
    unused(geometry_contact_data, contact_pairs, solver_results,
           contact_results_deformable);
  }
}

template <typename T>
void DiscreteUpdateManager<T>::CalcDiscreteUpdateMultibodyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(plant()));
  DoCalcDiscreteUpdateMultibodyForces(context, forces);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcActuation(const systems::Context<T>& context,
                                             VectorX<T>* actuation) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(actuation != nullptr);
  DRAKE_DEMAND(actuation->size() == plant().num_actuated_dofs());
  DoCalcActuation(context, actuation);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcDiscreteContactPairs(
    const systems::Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* result) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(result != nullptr);
  result->Clear();
  AppendDiscreteContactPairsForPointContact(context, result);
  if constexpr (std::is_same_v<T, symbolic::Expression>) {
    throw std::logic_error("This method doesn't support T = Expression.");
  } else {
    AppendDiscreteContactPairsForHydroelasticContact(context, result);
  }
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->AppendDiscreteContactPairs(context, result);
    }
  }
}

template <typename T>
void DiscreteUpdateManager<T>::AppendDiscreteContactPairsForPointContact(
    const systems::Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* contact_pairs) const {
  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      EvalGeometryContactData(context).get().point_pairs;
  const int num_point_contacts = point_pairs.size();
  if (num_point_contacts == 0) {
    return;
  }

  contact_pairs->Reserve(num_point_contacts, 0, 0);
  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);
  const SpanningForest& forest = get_forest();
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

    const TreeIndex& tree_A_index = forest.link_to_tree_index(body_A_index);
    const bool tree_A_has_dofs =
        tree_A_index.is_valid() && forest.trees(tree_A_index).has_dofs();

    const TreeIndex& tree_B_index = forest.link_to_tree_index(body_B_index);
    const bool tree_B_has_dofs =
        tree_B_index.is_valid() && forest.trees(tree_B_index).has_dofs();

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
    if (tree_A_has_dofs) {
      const SpanningForest::Tree& tree_A = forest.trees(tree_A_index);
      Matrix3X<T> J = R_WC.matrix().transpose() *
                      Jv_AcBc_W.middleCols(tree_A.v_start(), tree_A.nv());
      jacobian_blocks.emplace_back(tree_A_index, MatrixBlock<T>(std::move(J)));
    }

    // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
    // This contribution must be added only if B is different from A.
    if ((tree_B_has_dofs && !tree_A_has_dofs) ||
        (tree_B_has_dofs && tree_B_index != tree_A_index)) {
      const SpanningForest::Tree& tree_B = forest.trees(tree_B_index);
      Matrix3X<T> J = R_WC.matrix().transpose() *
                      Jv_AcBc_W.middleCols(tree_B.v_start(), tree_B.nv());
      jacobian_blocks.emplace_back(tree_B_index, MatrixBlock<T>(std::move(J)));
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
void DiscreteUpdateManager<T>::AppendDiscreteContactPairsForHydroelasticContact(
    const systems::Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* contact_pairs) const
  requires scalar_predicate<T>::is_bool
{  // NOLINT(whitespace/braces)
  const std::vector<geometry::ContactSurface<T>>& surfaces =
      EvalGeometryContactData(context).get().surfaces;
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
  const SpanningForest& forest = get_forest();
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

    const TreeIndex& tree_A_index = forest.link_to_tree_index(body_A_index);
    // World isn't in any tree so we can't call forest.trees() on the index,
    // which will be invalid.
    const bool treeA_has_dofs =
        tree_A_index.is_valid() && forest.trees(tree_A_index).has_dofs();

    const TreeIndex& tree_B_index = forest.link_to_tree_index(body_B_index);
    const bool treeB_has_dofs =
        tree_B_index.is_valid() && forest.trees(tree_B_index).has_dofs();

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
          const SpanningForest::Tree& tree_A = forest.trees(tree_A_index);
          Matrix3X<T> J = R_WC.matrix().transpose() *
                          Jv_AcBc_W.middleCols(tree_A.v_start(), tree_A.nv());
          jacobian_blocks.emplace_back(tree_A_index,
                                       MatrixBlock<T>(std::move(J)));
        }

        // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
        // This contribution must be added only if B is different from A.
        if ((treeB_has_dofs && !treeA_has_dofs) ||
            (treeB_has_dofs && tree_B_index != tree_A_index)) {
          const SpanningForest::Tree& tree_B = forest.trees(tree_B_index);
          Matrix3X<T> J = R_WC.matrix().transpose() *
                          Jv_AcBc_W.middleCols(tree_B.v_start(), tree_B.nv());
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

template <typename T>
const MultibodyForces<T>&
DiscreteUpdateManager<T>::EvalDiscreteUpdateMultibodyForces(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.discrete_update_multibody_forces)
      .template Eval<MultibodyForces<T>>(context);
}

template <typename T>
const VectorX<T>& DiscreteUpdateManager<T>::EvalActuation(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.actuation)
      .template Eval<VectorX<T>>(context);
}

template <typename T>
const DiscreteContactData<DiscreteContactPair<T>>&
DiscreteUpdateManager<T>::EvalDiscreteContactPairs(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.discrete_contact_pairs)
      .template Eval<DiscreteContactData<DiscreteContactPair<T>>>(context);
}

template <typename T>
void DiscreteUpdateManager<T>::DoCalcDiscreteValues(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* updates) const {
  const ContactSolverResults<T>& results = EvalContactSolverResults(context);

  // Previous time step positions.
  const int nq = plant().num_positions();
  const VectorX<T>& x0 =
      context.get_discrete_state(multibody_state_index()).value();
  const auto q0 = x0.topRows(nq);

  // Retrieve the rigid velocity for the next time step.
  const VectorX<T>& v_next = results.v_next.head(plant().num_velocities());

  // Update generalized positions.
  VectorX<T> qdot_next(plant().num_positions());
  plant().MapVelocityToQDot(context, v_next, &qdot_next);
  const VectorX<T> q_next = q0 + plant().time_step() * qdot_next;

  VectorX<T> x_next(plant().num_multibody_states());
  x_next << q_next, v_next;
  updates->set_value(multibody_state_index(), x_next);
}

template <typename T>
void DiscreteUpdateManager<T>::ExtractModelInfo() {
  // Collect information from each PhysicalModel owned by the plant.
  const std::vector<const multibody::PhysicalModel<T>*> physical_models =
      plant().physical_models();
  for (const auto* model : physical_models) {
    std::visit(
        [this](auto&& concrete_model) {
          this->ExtractConcreteModel(concrete_model);
        },
        model->ToPhysicalModelPointerVariant());
  }

  // Allow derived class to extract relevant model information.
  DoExtractModelInfo();
}

template <typename T>
void DiscreteUpdateManager<T>::ExtractConcreteModel(
    const DeformableModel<T>* model) {
  if constexpr (std::is_same_v<T, double>) {
    DRAKE_DEMAND(model != nullptr);
    DRAKE_DEMAND(deformable_driver_ == nullptr);
    if (model->num_bodies() > 0) {
      deformable_driver_ =
          std::make_unique<DeformableDriver<double>>(model, this);
    }
  } else {
    if (!model->is_empty()) {
      throw std::logic_error(
          "Only T = double is supported for the simulation of deformable "
          "bodies.");
    }
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
