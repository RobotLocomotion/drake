#include "drake/multibody/plant/discrete_update_manager.h"

#include <limits>
#include <utility>

#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {

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

template <typename T>
void DiscreteUpdateManager<T>::CalcDiscreteValues(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* updates) const {
  // The discrete sampling of input ports needs to be the first step of a
  // discrete update.
  SampleDiscreteInputPortForces(context);
  DRAKE_DEMAND(updates != nullptr);
  // Perform discrete updates for deformable bodies if they exist.
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->CalcDiscreteStates(context, updates);
    }
  }
  // Perform discrete updates for rigid bodies.
  DoCalcDiscreteValues(context, updates);
}

template <typename T>
void DiscreteUpdateManager<T>::DeclareCacheEntries() {
  // The Correct Solution:
  // The Implicit Stribeck solver solution S is a function of state x,
  // actuation input u (and externally applied forces) and even time if
  // any of the force elements in the model is time dependent. We can
  // write this as S = S(t, x, u).
  // Even though this variables can change continuously with time, we
  // want the solver solution to be updated periodically (with period
  // time_step()) only. That is, ContactSolverResults should be handled
  // as an abstract state with periodic updates. In the systems::
  // framework terminology, we'd like to have an "unrestricted update"
  // with a periodic event trigger.
  // The Problem (#10149):
  // From issue #10149 we know unrestricted updates incur a very
  // noticeably performance hit that at this stage we are not willing to
  // pay.
  // The Work Around (#10888):
  // To emulate the correct behavior until #10149 is addressed we declare
  // the Implicit Stribeck solver solution dependent only on the discrete
  // state. This is not the correct solution given these results do
  // depend on time and (even continuous) inputs. However it does emulate
  // the discrete update of these values as if zero-order held, which is
  // what we want.
  const auto& discrete_input_port_forces_cache_entry = this->DeclareCacheEntry(
      "Discrete force input port values",
      systems::ValueProducer(this, InputPortForces<T>(plant()),
                             &DiscreteUpdateManager<T>::CalcInputPortForces),
      // The cache entry is manually managed to refresh at the beginning of a
      // discrete update.
      {systems::System<T>::nothing_ticket()});
  cache_indexes_.discrete_input_port_forces =
      discrete_input_port_forces_cache_entry.cache_index();

  const auto& contact_solver_results_cache_entry = this->DeclareCacheEntry(
      "Contact solver results",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcContactSolverResults),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket(),
       discrete_input_port_forces_cache_entry.ticket()});
  cache_indexes_.contact_solver_results =
      contact_solver_results_cache_entry.cache_index();

  // See ThrowIfNonContactForceInProgress().
  const auto& non_contact_forces_evaluation_in_progress =
      this->DeclareCacheEntry(
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
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.discrete_update_multibody_forces =
      multibody_forces_cache_entry.cache_index();

  const auto& contact_results_cache_entry = DeclareCacheEntry(
      "Contact results (discrete)",
      systems::ValueProducer(this,
                             &DiscreteUpdateManager<T>::CalcContactResults),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_results = contact_results_cache_entry.cache_index();

  // Cache discrete contact pairs.
  const auto& discrete_contact_pairs_cache_entry = this->DeclareCacheEntry(
      "Discrete contact pairs.",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcDiscreteContactPairs),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.discrete_contact_pairs =
      discrete_contact_pairs_cache_entry.cache_index();

  // Cache contact kinematics.
  const auto& contact_kinematics_cache_entry = this->DeclareCacheEntry(
      "Contact kinematics.",
      systems::ValueProducer(this,
                             &DiscreteUpdateManager::CalcContactKinematics),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_kinematics =
      contact_kinematics_cache_entry.cache_index();

  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->DeclareCacheEntries(this);
    }
  }

  // Allow derived classes to declare their own cache entries.
  DoDeclareCacheEntries();
}

template <typename T>
systems::CacheEntry& DiscreteUpdateManager<T>::DeclareCacheEntry(
    std::string description, systems::ValueProducer value_producer,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
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
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::default_contact_dissipation(plant());
}

template <typename T>
const std::unordered_map<geometry::GeometryId, BodyIndex>&
DiscreteUpdateManager<T>::geometry_id_to_body_index() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::geometry_id_to_body_index(*plant_);
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
const MultibodyTree<T>& DiscreteUpdateManager<T>::internal_tree() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::internal_tree(plant());
}

template <typename T>
void DiscreteUpdateManager<T>::CalcNonContactForces(
    const drake::systems::Context<T>& context,
    bool include_joint_limit_penalty_forces, bool include_pd_controlled_input,
    MultibodyForces<T>* forces) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(plant()));

  const ScopeExit guard = ThrowIfNonContactForceInProgress(context);

  // Compute forces applied through force elements. Note that this resets
  // forces to empty so must come first.
  CalcForceElementsContribution(context, forces);

  // Evaluate all input forces at once.
  const InputPortForces<T>& inputs = EvalInputPortForces(context);

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
const contact_solvers::internal::ContactSolverResults<T>&
DiscreteUpdateManager<T>::EvalContactSolverResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_solver_results)
      .template Eval<contact_solvers::internal::ContactSolverResults<T>>(
          context);
}

template <typename T>
const std::vector<geometry::ContactSurface<T>>&
DiscreteUpdateManager<T>::EvalContactSurfaces(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalContactSurfaces(
      plant(), context);
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
DiscreteUpdateManager<T>::EvalJointLockingCache(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalJointLockingCache(
      plant(), context);
}

template <typename T>
VectorX<T> DiscreteUpdateManager<T>::AssembleActuationInput(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::AssembleActuationInput(
      plant(), context);
}

template <typename T>
VectorX<T> DiscreteUpdateManager<T>::AssembleDesiredStateInput(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::AssembleDesiredStateInput(plant(), context);
}

template <typename T>
const std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
DiscreteUpdateManager<T>::coupler_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::coupler_constraints_specs(*plant_);
}

template <typename T>
const std::map<MultibodyConstraintId, internal::DistanceConstraintSpec>&
DiscreteUpdateManager<T>::distance_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::distance_constraints_specs(*plant_);
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
const std::map<MultibodyConstraintId, bool>&
DiscreteUpdateManager<T>::GetConstraintActiveStatus(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::GetConstraintActiveStatus(context, *plant_);
}

template <typename T>
BodyIndex DiscreteUpdateManager<T>::FindBodyByGeometryId(
    geometry::GeometryId geometry_id) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::FindBodyByGeometryId(
      plant(), geometry_id);
}

template <typename T>
ScopeExit DiscreteUpdateManager<T>::ThrowIfNonContactForceInProgress(
    const systems::Context<T>& context) const {
  systems::CacheEntryValue& value =
      plant()
          .get_cache_entry(
              cache_indexes_.non_contact_forces_evaluation_in_progress)
          .get_mutable_cache_entry_value(context);
  bool& evaluation_in_progress = value.GetMutableValueOrThrow<bool>();
  if (evaluation_in_progress) {
    const char* error_message =
        "Algebraic loop detected. This situation is caused when connecting "
        "the input of your MultibodyPlant to the output of a feedback system "
        "which is an algebraic function of a feedthrough output of the "
        "plant. Ways to remedy this: 1. Revisit the model for your feedback "
        "system. Consider if its output can be written in terms of other "
        "inputs. 2. Break the algebraic loop by adding state to the "
        "controller, typically to 'remember' a previous input. 3. Break the "
        "algebraic loop by adding a zero-order hold system between the "
        "output of the plant and your feedback system. This effectively "
        "delays the input signal to the controller.";
    throw std::runtime_error(error_message);
  }
  // Mark the start of the computation. If within an algebraic
  // loop, pulling from the plant's input ports during the
  // computation will trigger the recursive evaluation of this
  // method and the exception above will be thrown.
  evaluation_in_progress = true;
  // If the exception above is triggered, we will leave this method and the
  // computation will no longer be "in progress". We use a scoped guard so
  // that we have a chance to mark it as such when we leave this scope.
  return ScopeExit([&evaluation_in_progress]() {
    evaluation_in_progress = false;
  });
}

template <typename T>
void DiscreteUpdateManager<T>::SampleDiscreteInputPortForces(
    const drake::systems::Context<T>& context) const {
  const auto& discrete_input_forces_cache_entry =
      plant().get_cache_entry(cache_indexes_.discrete_input_port_forces);
  // The discrete sampling via cache entry trick only works when caching is
  // enable. See #12786 for details.
  if (discrete_input_forces_cache_entry.is_cache_entry_disabled(context)) {
    static const logging::Warn log_once(
        "The discrete sampling of external force input ports rely on caching "
        "turned on. Caching is disabled for the discrete MultibodyPlant's "
        "context. As a result, the external force input ports are sampled "
        "continuously instead. See issue #12643.");
  }
  // Actually sample the discrete forces.
  auto& cache_entry_value =
      discrete_input_forces_cache_entry.get_mutable_cache_entry_value(context);
  cache_entry_value.mark_out_of_date();
  InputPortForces<T>& forces =
      cache_entry_value.template GetMutableValueOrThrow<InputPortForces<T>>();
  CalcInputPortForces(context, &forces);
  cache_entry_value.mark_up_to_date();

  // Initiate a value modification event.
  const systems::DependencyTracker& tracker =
      context.get_tracker(discrete_input_forces_cache_entry.ticket());
  tracker.NoteValueChange(context.start_new_change_event());
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
    const VectorX<T> u = AssembleActuationInput(context);
    for (JointActuatorIndex actuator_index(0);
         actuator_index < plant().num_actuators(); ++actuator_index) {
      const JointActuator<T>& actuator =
          plant().get_joint_actuator(actuator_index);
      const Joint<T>& joint = actuator.joint();
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(joint.num_velocities() == 1);
      const int v_index = joint.velocity_start();
      VectorX<T>& actuation =
          actuator.has_controller() ? *actuation_w_pd : *actuation_wo_pd;
      actuation[v_index] = u[actuator_index];
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
  DRAKE_DEMAND(contact_results != nullptr);
  plant().ValidateContext(context);
  DoCalcContactResults(context, contact_results);
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
void DiscreteUpdateManager<T>::CalcContactKinematics(
    const systems::Context<T>& context,
    DiscreteContactData<ContactPairKinematics<T>>* result) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(result != nullptr);
  result->Clear();
  const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  // Quick no-op exit.
  if (num_contacts == 0) return;

  result->Reserve(contact_pairs.num_point_contacts(),
                  contact_pairs.num_hydro_contacts(),
                  contact_pairs.num_deformable_contacts());
  AppendContactKinematics(context, contact_pairs.point_contact_data(),
                          DiscreteContactType::kPoint, result);
  AppendContactKinematics(context, contact_pairs.hydro_contact_data(),
                          DiscreteContactType::kHydroelastic, result);
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->AppendContactKinematics(context, result);
    }
  }
}

template <typename T>
void DiscreteUpdateManager<T>::AppendContactKinematics(
    const systems::Context<T>& context,
    const std::vector<DiscreteContactPair<T>>& contact_pairs,
    DiscreteContactType type,
    DiscreteContactData<ContactPairKinematics<T>>* contact_kinematics) const {
  // Scratch workspace variables.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);

  const Frame<T>& frame_W = plant().world_frame();
  for (int icontact = 0; icontact < ssize(contact_pairs); ++icontact) {
    const auto& point_pair = contact_pairs[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = this->geometry_id_to_body_index().at(geometryA_id);
    const Body<T>& bodyA = plant().get_body(bodyA_index);
    BodyIndex bodyB_index = this->geometry_id_to_body_index().at(geometryB_id);
    const Body<T>& bodyB = plant().get_body(bodyB_index);

    // Contact normal from point A into B.
    const Vector3<T>& nhat_W = -point_pair.nhat_BA_W;
    const Vector3<T>& p_WC = point_pair.p_WC;

    // Contact point position relative to each body.
    const RigidTransform<T>& X_WA = plant().EvalBodyPoseInWorld(context, bodyA);
    const Vector3<T>& p_WA = X_WA.translation();
    const Vector3<T> p_AC_W = p_WC - p_WA;
    const RigidTransform<T>& X_WB = plant().EvalBodyPoseInWorld(context, bodyB);
    const Vector3<T>& p_WB = X_WB.translation();
    const Vector3<T> p_BC_W = p_WC - p_WB;

    // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian will be:
    //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
    // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
    this->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, bodyA.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WAc_W);
    this->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, bodyB.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WBc_W);
    Jv_AcBc_W = Jv_WBc_W - Jv_WAc_W;

    // Define a contact frame C at the contact point such that the z-axis Cz
    // equals nhat_W. The tangent vectors are arbitrary, with the only
    // requirement being that they form a valid right handed basis with nhat_W.
    math::RotationMatrix<T> R_WC =
        math::RotationMatrix<T>::MakeFromOneVector(nhat_W, 2);

    const TreeIndex& treeA_index =
        tree_topology().body_to_tree_index(bodyA_index);
    const TreeIndex& treeB_index =
        tree_topology().body_to_tree_index(bodyB_index);
    // Sanity check, at least one must be valid.
    DRAKE_DEMAND(treeA_index.is_valid() || treeB_index.is_valid());

    // We have at most two blocks per contact.
    std::vector<typename ContactPairKinematics<T>::JacobianTreeBlock>
        jacobian_blocks;
    jacobian_blocks.reserve(2);

    // Tree A contribution to contact Jacobian Jv_W_AcBc_C.
    if (treeA_index.is_valid()) {
      Matrix3X<T> J = R_WC.matrix().transpose() *
                      Jv_AcBc_W.middleCols(
                          tree_topology().tree_velocities_start(treeA_index),
                          tree_topology().num_tree_velocities(treeA_index));
      jacobian_blocks.emplace_back(treeA_index, MatrixBlock<T>(std::move(J)));
    }

    // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
    // This contribution must be added only if B is different from A.
    if ((treeB_index.is_valid() && !treeA_index.is_valid()) ||
        (treeB_index.is_valid() && treeB_index != treeA_index)) {
      Matrix3X<T> J = R_WC.matrix().transpose() *
                      Jv_AcBc_W.middleCols(
                          tree_topology().tree_velocities_start(treeB_index),
                          tree_topology().num_tree_velocities(treeB_index));
      jacobian_blocks.emplace_back(treeB_index, MatrixBlock<T>(std::move(J)));
    }

    ContactConfiguration<T> configuration{.objectA = bodyA_index,
                                          .p_ApC_W = p_AC_W,
                                          .objectB = bodyB_index,
                                          .p_BqC_W = p_BC_W,
                                          .phi = point_pair.phi0,
                                          .R_WC = R_WC};
    switch (type) {
      case DiscreteContactType::kPoint: {
        contact_kinematics->AppendPointData(ContactPairKinematics<T>{
            std::move(jacobian_blocks), std::move(configuration)});
        break;
      }
      case DiscreteContactType::kHydroelastic: {
        contact_kinematics->AppendHydroData(ContactPairKinematics<T>{
            std::move(jacobian_blocks), std::move(configuration)});
        break;
      }
      case DiscreteContactType::kDeformable: {
        throw std::logic_error(
            "Call DeformableDriver::AppendContactKinematics() to compute "
            "contact kinematics for deformable contact instead.");
      }
    }
  }
}

template <typename T>
void DiscreteUpdateManager<T>::CalcDiscreteContactPairs(
    const systems::Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* result) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(result != nullptr);
  result->Clear();
  if (plant().num_collision_geometries() == 0) return;

  const auto contact_model = plant().get_contact_model();

  // We first compute the number of contact pairs so that we can allocate all
  // memory at once.
  // N.B. num_point_pairs = 0 when:
  //   1. There are legitimately no point pairs or,
  //   2. the point pair model is not even in use.
  // We guard for case (2) since EvalPointPairPenetrations() cannot be called
  // when point contact is not used and would otherwise throw an exception.
  int num_point_pairs = 0;  // The number of point contact pairs.
  if (contact_model == ContactModel::kPoint ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    num_point_pairs = plant().EvalPointPairPenetrations(context).size();
  }

  int num_quadrature_pairs = 0;
  // N.B. For discrete hydro we use a first order quadrature rule. As such,
  // the per-face quadrature point is the face's centroid and the weight is 1.
  // This is compatible with a mesh that is triangle or polygon. If we attempted
  // higher order quadrature, polygons would have to be decomposed into smaller
  // n-gons which can receive an appropriate set of quadrature points.
  if (contact_model == ContactModel::kHydroelastic ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    const std::vector<geometry::ContactSurface<T>>& surfaces =
        this->EvalContactSurfaces(context);
    for (const auto& s : surfaces) {
      // One quadrature point per face.
      num_quadrature_pairs += s.num_faces();
    }
  }
  result->Reserve(num_point_pairs, num_quadrature_pairs, 0);

  if (contact_model == ContactModel::kPoint ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    AppendDiscreteContactPairsForPointContact(context, result);
  }
  if (contact_model == ContactModel::kHydroelastic ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    AppendDiscreteContactPairsForHydroelasticContact(context, result);
  }
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->AppendDiscreteContactPairs(context, result);
    }
  }
}

template <>
void DiscreteUpdateManager<symbolic::Expression>::CalcDiscreteContactPairs(
    const drake::systems::Context<symbolic::Expression>&,
    DiscreteContactData<DiscreteContactPair<symbolic::Expression>>*) const {
  // Currently, the computation of contact pairs is not supported when T =
  // symbolic::Expression.
  throw std::domain_error(
      fmt::format("This method doesn't support T = {}.",
                  NiceTypeName::Get<symbolic::Expression>()));
}

template <typename T>
void DiscreteUpdateManager<T>::AppendDiscreteContactPairsForPointContact(
    const systems::Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* result) const {
  DiscreteContactData<DiscreteContactPair<T>>& contact_pairs = *result;

  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

  const std::vector<std::vector<int>>& per_tree_unlocked_indices =
      this->EvalJointLockingCache(context).unlocked_velocity_indices_per_tree;
  const MultibodyTreeTopology& topology = this->internal_tree().get_topology();

  // Fill in the point contact pairs.
  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      plant().EvalPointPairPenetrations(context);
  for (const PenetrationAsPointPair<T>& pair : point_pairs) {
    const BodyIndex body_A_index =
        this->geometry_id_to_body_index().at(pair.id_A);
    const Body<T>& body_A = plant().get_body(body_A_index);
    const BodyIndex body_B_index =
        this->geometry_id_to_body_index().at(pair.id_B);
    const Body<T>& body_B = plant().get_body(body_B_index);

    const TreeIndex& treeA_index = topology.body_to_tree_index(body_A_index);
    const TreeIndex& treeB_index = topology.body_to_tree_index(body_B_index);

    //  For joint locking, filter out contacts between bodies who belong to
    //  trees with 0 degrees of freedom. For a contact to remain in
    //  consideration, at least one of the trees involved has to be valid and
    //  have a non-zero number of DOFs.
    if ((treeA_index.is_valid() &&
         per_tree_unlocked_indices[treeA_index].size() != 0) ||
        (treeB_index.is_valid() &&
         per_tree_unlocked_indices[treeB_index].size() != 0)) {
      const T kA = GetPointContactStiffness(
          pair.id_A, this->default_contact_stiffness(), inspector);
      const T kB = GetPointContactStiffness(
          pair.id_B, this->default_contact_stiffness(), inspector);
      const T k = GetCombinedPointContactStiffness(
          pair.id_A, pair.id_B, this->default_contact_stiffness(), inspector);

      // Hunt & Crossley dissipation. Used by TAMSI, ignored by SAP.
      const T d = GetCombinedHuntCrossleyDissipation(
          pair.id_A, pair.id_B, kA, kB, this->default_contact_dissipation(),
          inspector);

      // Dissipation time scale. Used by SAP, ignored by TAMSI.
      const double default_dissipation_time_constant = 0.1;
      const T tau = GetCombinedDissipationTimeConstant(
          pair.id_A, pair.id_B, default_dissipation_time_constant,
          body_A.name(), body_B.name(), inspector);
      const T mu =
          GetCombinedDynamicCoulombFriction(pair.id_A, pair.id_B, inspector);

      // We compute the position of the point contact based on Hertz's theory
      // for contact between two elastic bodies.
      const T denom = kA + kB;
      const T wA = (denom == 0 ? 0.5 : kA / denom);
      const T wB = (denom == 0 ? 0.5 : kB / denom);
      const Vector3<T> p_WC = wA * pair.p_WCa + wB * pair.p_WCb;

      const T phi0 = -pair.depth;
      const T fn0 = k * pair.depth;  // Used by TAMSI, ignored by SAP.

      contact_pairs.AppendPointData(
          DiscreteContactPair<T>{pair.id_A,
                                 pair.id_B,
                                 p_WC,
                                 pair.nhat_BA_W,
                                 phi0,
                                 fn0,
                                 k,
                                 d,
                                 tau,
                                 mu,
                                 {} /* no surface index */,
                                 {} /* no face index */});
    }
  }
}

template <>
void DiscreteUpdateManager<symbolic::Expression>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const drake::systems::Context<symbolic::Expression>&,
        DiscreteContactData<DiscreteContactPair<symbolic::Expression>>*) const {
  throw std::domain_error(
      fmt::format("This method doesn't support T = {}.",
                  NiceTypeName::Get<symbolic::Expression>()));
}

template <typename T>
void DiscreteUpdateManager<T>::AppendDiscreteContactPairsForHydroelasticContact(
    const systems::Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* result) const {
  DiscreteContactData<DiscreteContactPair<T>>& contact_pairs = *result;

  // N.B. For discrete hydro we use a first order quadrature rule. As such,
  // the per-face quadrature point is the face's centroid and the weight is 1.
  // This is compatible with a mesh that is triangle or polygon. If we attempted
  // higher order quadrature, polygons would have to be decomposed into smaller
  // n-gons which can receive an appropriate set of quadrature points.

  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();
  const std::vector<geometry::ContactSurface<T>>& surfaces =
      this->EvalContactSurfaces(context);

  const std::vector<std::vector<int>>& per_tree_unlocked_indices =
      this->EvalJointLockingCache(context).unlocked_velocity_indices_per_tree;
  const MultibodyTreeTopology& topology = this->internal_tree().get_topology();

  const int num_surfaces = surfaces.size();
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    const auto& s = surfaces[surface_index];

    const bool M_is_compliant = s.HasGradE_M();
    const bool N_is_compliant = s.HasGradE_N();
    DRAKE_DEMAND(M_is_compliant || N_is_compliant);

    // Combine dissipation.
    const BodyIndex body_M_index =
        this->geometry_id_to_body_index().at(s.id_M());
    const Body<T>& body_M = plant().get_body(body_M_index);
    const BodyIndex body_N_index =
        this->geometry_id_to_body_index().at(s.id_N());
    const Body<T>& body_N = plant().get_body(body_N_index);

    const TreeIndex& treeM_index = topology.body_to_tree_index(body_M_index);
    const TreeIndex& treeN_index = topology.body_to_tree_index(body_N_index);

    //  For joint locking, filter out contacts between bodies who belong to
    //  trees with 0 degrees of freedom. For a contact to remain in
    //  consideration, at least one of the trees involved has to be valid and
    //  have a non-zero number of DOFs.
    if ((treeM_index.is_valid() &&
         per_tree_unlocked_indices[treeM_index].size() != 0) ||
        (treeN_index.is_valid() &&
         per_tree_unlocked_indices[treeN_index].size() != 0)) {
      // TODO(amcastro-tri): Consider making the modulus required, instead of
      // a default infinite value.
      const T hydro_modulus_M = GetHydroelasticModulus(
          s.id_M(), std::numeric_limits<double>::infinity(), inspector);
      const T hydro_modulus_N = GetHydroelasticModulus(
          s.id_N(), std::numeric_limits<double>::infinity(), inspector);

      // Hunt & Crossley dissipation. Used by TAMSI, ignored by SAP.
      const T d = GetCombinedHuntCrossleyDissipation(
          s.id_M(), s.id_N(), hydro_modulus_M, hydro_modulus_N,
          0.0 /* Default value */, inspector);

      // Dissipation time scale. Used by SAP, ignored by TAMSI.
      const double default_dissipation_time_constant = 0.1;
      const T tau = GetCombinedDissipationTimeConstant(
          s.id_M(), s.id_N(), default_dissipation_time_constant, body_M.name(),
          body_N.name(), inspector);

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
          // guaranteed to point "out of" N and "into" M.
          const Vector3<T>& nhat_W = s.face_normal(face);

          // One dimensional pressure gradient (in Pa/m). Unlike [Masterjohn
          // 2022], for convenience we define both pressure gradients
          // to be positive in the direction "into" the bodies. Therefore,
          // we use the minus sign for gN.
          // [Masterjohn 2022] Velocity Level Approximation of Pressure
          // Field Contact Patches.
          const T gM = M_is_compliant
                           ? s.EvaluateGradE_M_W(face).dot(nhat_W)
                           : T(std::numeric_limits<double>::infinity());
          const T gN = N_is_compliant
                           ? -s.EvaluateGradE_N_W(face).dot(nhat_W)
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

          // Position of quadrature point Q in the world frame (since mesh_W
          // is measured and expressed in W).
          const Vector3<T>& p_WQ = s.centroid(face);
          // For a triangle, its centroid has the fixed barycentric
          // coordinates independent of the shape of the triangle. Using
          // barycentric coordinates to evaluate field value could be
          // faster than using Cartesian coordinates, especially if the
          // TriangleSurfaceMeshFieldLinear<> does not store gradients and
          // has to solve linear equations to convert Cartesian to
          // barycentric coordinates.
          const Vector3<T> tri_centroid_barycentric(1 / 3., 1 / 3., 1 / 3.);
          // Pressure at the quadrature point.
          const T p0 =
              s.is_triangle()
                  ? s.tri_e_MN().Evaluate(face, tri_centroid_barycentric)
                  : s.poly_e_MN().EvaluateCartesian(face, p_WQ);

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

          if (k > 0) {
            contact_pairs.AppendHydroData(DiscreteContactPair<T>{
                s.id_M(), s.id_N(), p_WQ, nhat_W, phi0, fn0, k, d, tau, mu,
                surface_index, face});
          }
        }
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
const ContactResults<T>& DiscreteUpdateManager<T>::EvalContactResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_results)
      .template Eval<ContactResults<T>>(context);
}

template <typename T>
const DiscreteContactData<ContactPairKinematics<T>>&
DiscreteUpdateManager<T>::EvalContactKinematics(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_kinematics)
      .template Eval<DiscreteContactData<ContactPairKinematics<T>>>(context);
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
      context.get_discrete_state(this->multibody_state_index()).value();
  const auto q0 = x0.topRows(nq);

  // Retrieve the rigid velocity for the next time step.
  const VectorX<T>& v_next = results.v_next.head(plant().num_velocities());

  // Update generalized positions.
  VectorX<T> qdot_next(plant().num_positions());
  plant().MapVelocityToQDot(context, v_next, &qdot_next);
  const VectorX<T> q_next = q0 + plant().time_step() * qdot_next;

  VectorX<T> x_next(plant().num_multibody_states());
  x_next << q_next, v_next;
  updates->set_value(this->multibody_state_index(), x_next);
}

template <typename T>
void DiscreteUpdateManager<T>::ExtractModelInfo() {
  // Collect information from each PhysicalModel owned by the plant.
  const std::vector<const multibody::PhysicalModel<T>*> physical_models =
      this->plant().physical_models();
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
    // TODO(xuchenhan-tri): Demote this to a DRAKE_DEMAND when we check for
    //  duplicated model with MbP::AddPhysicalModel.
    if (deformable_driver_ != nullptr) {
      throw std::logic_error(
          fmt::format("{}: A deformable model has already been registered. "
                      "Repeated registration is not allowed.",
                      __func__));
    }
    deformable_driver_ =
        std::make_unique<DeformableDriver<double>>(model, this);
  } else {
    unused(model);
    throw std::logic_error(
        "Only T = double is supported for the simulation of deformable "
        "bodies.");
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
