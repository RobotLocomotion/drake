#include "drake/multibody/contact_solvers/icf/icf_builder.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/topology/graph.h"

using drake::multibody::CalcContactFrictionFromSurfaceProperties;
using drake::multibody::internal::CouplerConstraintSpec;
using drake::multibody::internal::GetCombinedDynamicCoulombFriction;
using drake::multibody::internal::GetCombinedHuntCrossleyDissipation;
using drake::multibody::internal::GetCombinedPointContactStiffness;
using drake::multibody::internal::GetCoulombFriction;
using drake::multibody::internal::GetHuntCrossleyDissipation;
using drake::multibody::internal::GetInternalTree;
using drake::multibody::internal::GetPointContactStiffness;
using drake::multibody::internal::LinkJointGraph;
using drake::multibody::internal::MobodIndex;
using drake::multibody::internal::SpanningForest;
using drake::multibody::internal::TreeIndex;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
T CombineHuntCrossleyDissipation(const T& stiffness_A, const T& stiffness_B,
                                 const T& dA, const T& dB) {
  const double kInf = std::numeric_limits<double>::infinity();
  if (stiffness_A == kInf) return dB;
  if (stiffness_B == kInf) return dA;

  // Return zero dissipation if both stiffness values are zero.
  const T denom = stiffness_A + stiffness_B;
  if (denom == 0.0) return 0.0;

  // At this point we know both geometries are compliant and at least one of
  // them has non-zero stiffness (denom != 0).
  return (stiffness_B / denom) * dA + (stiffness_A / denom) * dB;
}

template <typename T>
void IcfBuilder<T>::CalcGeometryContactData(
    const systems::Context<T>& context) {
  surfaces_.clear();
  point_pairs_.clear();

  auto& query_object = plant()
                           .get_geometry_query_input_port()
                           .template Eval<geometry::QueryObject<T>>(context);

  switch (plant().get_contact_model()) {
    case ContactModel::kPoint: {
      point_pairs_ = query_object.ComputePointPairPenetration();
      break;
    }
    case ContactModel::kHydroelastic: {
      surfaces_ = query_object.ComputeContactSurfaces(
          plant().get_contact_surface_representation());
      break;
    }
    case ContactModel::kHydroelasticWithFallback: {
      query_object.ComputeContactSurfacesWithFallback(
          plant().get_contact_surface_representation(), &surfaces_,
          &point_pairs_);
      break;
    }
  }
}

template <typename T>
IcfBuilder<T>::IcfBuilder(const MultibodyPlant<T>& plant,
                          const systems::Context<T>& context)
    : plant_(&plant), scratch_(plant) {
  using std::isinf;
  const int nv = plant.num_velocities();

  // Define problem cliques based on the spanning forest of the plant. Each tree
  // gets its own clique, and only trees with a non-zero number of velocities
  // are included.
  const SpanningForest& forest = GetInternalTree(plant).forest();
  const LinkJointGraph& graph = GetInternalTree(plant).graph();
  DRAKE_DEMAND(graph.forest_is_valid());

  clique_sizes_.clear();
  tree_to_clique_.resize(forest.num_trees());
  std::fill(tree_to_clique_.begin(), tree_to_clique_.end(), -1);

  for (TreeIndex t(0); t < forest.num_trees(); ++t) {
    const int tree_nv = forest.trees(t).nv();
    if (tree_nv > 0) {
      tree_to_clique_[t] = clique_sizes_.size();
      clique_sizes_.push_back(tree_nv);
    }
  }

  // Iterate over bodies to find the size of each Jacobian, clique membership,
  // floating status (floating bodies have an identity Jacobian), body mass.
  body_jacobian_cols_.resize(plant.num_bodies());
  body_to_clique_.resize(plant.num_bodies());
  body_is_floating_.resize(plant.num_bodies());
  body_mass_.resize(plant.num_bodies());

  for (int b = 0; b < plant.num_bodies(); ++b) {
    const auto& body = plant.get_body(BodyIndex(b));
    const TreeIndex t = forest.link_to_tree_index(BodyIndex(b));

    if (plant.IsAnchored(body)) {
      body_jacobian_cols_[b] = 1;  // dummy column.
      body_to_clique_[b] = -1;
    } else {
      body_jacobian_cols_[b] = forest.trees(t).nv();
      body_to_clique_[b] = tree_to_clique(t);
    }

    // Distinguish between "free floating body" and "free floating base". The
    // former has an identity Jacobian, the latter does not.
    const auto& link = forest.link_by_index(BodyIndex(b));
    const auto& mobod = forest.mobods(link.mobod_index());
    const bool is_free_floating =
        body.is_floating_base_body() && mobod.is_leaf_mobod();
    body_is_floating_[b] = is_free_floating ? 1 : 0;

    // If the body has zero mass, we assign it the mass of its composite.
    // TODO(amcastro-tri): consider using forest.link_composites() in
    // combination with LinkJointGraph::Link::composite() to precompute
    // composite's masses.
    if (body.default_mass() == 0.0) {
      const auto composite = graph.GetLinksWeldedTo(body.index());
      T composite_mass = 0.0;
      for (BodyIndex c : composite) {
        composite_mass += plant.get_body(c).default_mass();
      }
      if (composite_mass == 0.0) {
        throw std::logic_error(
            fmt::format("Composite for body '{}' has zero mass.", body.name()));
      }
      body_mass_[b] = composite_mass;
    } else {
      body_mass_[b] = body.default_mass();
    }
  }

  // Iterate over actuators to find number of actuators per clique, and effort
  // limits for each velocity.
  clique_nu_.assign(clique_sizes_.size(), 0);
  effort_limits_.resize(nv);
  effort_limits_.setZero();

  for (JointActuatorIndex actuator_index : plant.GetJointActuatorIndices()) {
    const JointActuator<T>& actuator = plant.get_joint_actuator(actuator_index);
    const Joint<T>& joint = actuator.joint();
    const int dof = joint.velocity_start();
    const TreeIndex t = forest.v_to_tree_index(dof);
    const int c = tree_to_clique(t);
    DRAKE_ASSERT(c >= 0);
    ++clique_nu_[c];
    effort_limits_[dof] = actuator.effort_limit();
  }

  // Iterate over joints to find cliques with at least one 1-DoF joint with
  // finite limits. Each of these cliques will require a limit constraint.
  limited_clique_sizes_.clear();
  clique_to_limit_constraint_.assign(clique_sizes_.size(), -1);
  limit_constraint_to_clique_.clear();
  for (JointIndex joint_index : plant.GetJointIndices()) {
    const Joint<T>& joint = plant.get_joint(joint_index);

    // Check for 1-DoF joints
    if (joint.num_positions() != 1 || joint.num_velocities() != 1) {
      continue;
    }

    // Check for finite upper or lower limits. Note that this must come after
    // the 1-DoF check above.
    if (isinf(joint.position_lower_limits()[0]) &&
        isinf(joint.position_upper_limits()[0])) {
      continue;
    }

    // Record the size of the clique this joint belongs to, and the mappings
    // from limit constraint index <---> clique index.
    const TreeIndex tree_index = forest.v_to_tree_index(joint.velocity_start());
    const int clique = tree_to_clique(tree_index);
    const int clique_nv = clique_sizes_[clique];
    limited_clique_sizes_.push_back(clique_nv);
    limit_constraint_to_clique_.push_back(clique);
    clique_to_limit_constraint_[clique] = limited_clique_sizes_.size() - 1;
  }

  // Retrieve constant model parameters.
  // TODO(amcastro-tri): This should be retrieved from the default contact
  // properties.
  const double kDefaultDissipation = 50.0;
  const double kDefaultStiffness = 1.0e6;

  const geometry::SceneGraphInspector<T>& inspector =
      plant.EvalSceneGraphInspector(context);

  const std::vector<geometry::GeometryId> geometries =
      inspector.GetAllGeometryIds(geometry::Role::kProximity);

  for (geometry::GeometryId id : geometries) {
    stiffness_[id] = GetPointContactStiffness(id, kDefaultStiffness, inspector);
    friction_[id] = GetCoulombFriction(id, inspector);
    dissipation_[id] =
        GetHuntCrossleyDissipation(id, kDefaultDissipation, inspector);
  }
}

template <typename T>
void IcfBuilder<T>::UpdateModel(
    const systems::Context<T>& context, const T& time_step,
    const IcfLinearFeedbackGains<T>* actuation_feedback,
    const IcfLinearFeedbackGains<T>* external_feedback, IcfModel<T>* model) {
  DRAKE_ASSERT(model != nullptr);
  const SpanningForest& forest = GetInternalTree(plant()).forest();
  const int nv = plant().num_velocities();

  std::unique_ptr<IcfParameters<T>> params = model->ReleaseParameters();

  // Set the time step δt and initial velocities v₀
  params->time_step = time_step;
  params->v0 = plant().GetVelocities(context);

  // Set the (dense) mass matrix M₀.
  MatrixX<T>& M = params->M0;
  M.resize(nv, nv);
  plant().CalcMassMatrix(context, &M);

  // Set joint damping D₀.
  params->D0.resize(nv);
  params->D0 = plant().EvalJointDampingCache(context);

  // Compute nonlinear bias terms k₀.
  params->k0.resize(nv);
  MultibodyForces<T>& forces = scratch_.forces;
  plant().CalcForceElementsContribution(context, &forces);
  const VectorX<T>& acc = scratch_.accelerations.setZero(nv);
  params->k0 = plant().CalcInverseDynamics(context, acc, forces);

  // Set the sparsity pattern for the dynamics matrix A = M + δt D.
  params->clique_sizes = clique_sizes_;
  params->clique_start.resize(clique_sizes_.size() + 1);
  params->clique_start[0] = 0;
  for (size_t c = 0; c < clique_sizes_.size(); ++c) {
    params->clique_start[c + 1] = params->clique_start[c] + clique_sizes_[c];
  }

  // Body mass, clique membership, and body floating status are defined at
  // builder construction time, since they depend only on the plant and not on
  // the current state (q0, v0).
  params->body_mass = body_mass_;
  params->body_to_clique = body_to_clique_;
  params->body_is_floating = body_is_floating_;

  // Compute spatial velocity Jacobians J_WB for all bodies.
  const auto& world_frame = plant().world_frame();
  EigenPool<Matrix6X<T>>& J_WB = params->J_WB;
  J_WB.Resize(plant().num_bodies(), 6, body_jacobian_cols_);

  for (int b = 0; b < plant().num_bodies(); ++b) {
    const auto& body = plant().get_body(BodyIndex(b));

    if (!plant().IsAnchored(body)) {
      // We ignore anchored bodies. Their jacobian is included in J_WB as a
      // dummy column.
      const TreeIndex t = forest.link_to_tree_index(BodyIndex(b));
      const int clique = tree_to_clique(t);
      DRAKE_ASSERT(clique >= 0);
      const bool tree_has_dofs = t.is_valid() && forest.trees(t).has_dofs();
      DRAKE_ASSERT(tree_has_dofs);

      const int vt_start = forest.trees(t).v_start();
      const int nt = forest.trees(t).nv();

      typename EigenPool<Matrix6X<T>>::MatrixView Jv_WBc_W = J_WB[b];
      if (body.is_floating_base_body()) {
        Jv_WBc_W.setIdentity();
      } else {
        Matrix6X<T>& J_V_WB = scratch_.J_V_WB;
        plant().CalcJacobianSpatialVelocity(
            context, JacobianWrtVariable::kV, body.body_frame(),
            Vector3<T>::Zero(), world_frame, world_frame, &J_V_WB);
        Jv_WBc_W = J_V_WB.middleCols(vt_start, nt);
      }
    }
  }

  // Set the model parameters before adding constraints.
  model->ResetParameters(std::move(params));

  // Contact constraints
  CalcGeometryContactData(context);
  AllocatePatchConstraints(model);
  SetPatchConstraintsForPointContact(context, model);
  SetPatchConstraintsForHydroelasticContact(context, model);

  // Coupler constraints
  AllocateCouplerConstraints(model);
  SetCouplerConstraints(context, model);

  // Limit constraints
  AllocateLimitConstraints(model);
  SetLimitConstraints(context, model);

  // Gain and external force constraints
  // N.B. external forces must come first, followed by actuation
  AllocateGainConstraints(model, actuation_feedback != nullptr,
                          external_feedback != nullptr);
  if (external_feedback != nullptr) {
    const VectorX<T>& Ke = external_feedback->K;
    const VectorX<T>& be = external_feedback->b;
    SetExternalGainConstraints(Ke, be, model);
  }
  if (actuation_feedback != nullptr) {
    const VectorX<T>& Ku = actuation_feedback->K;
    const VectorX<T>& bu = actuation_feedback->b;
    // N.B. actuation constraint indices in the pool depend on whether external
    // or not external forces are present in the model.
    SetActuationGainConstraints(Ku, bu, external_feedback != nullptr, model);
  }

  // Define the sparsity pattern, which defines which cliques are connected by
  // constraints.
  model->SetSparsityPattern();
}

template <typename T>
void IcfBuilder<T>::UpdateModel(const T& time_step, IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);
  model->UpdateTimeStep(time_step);
}

template <typename T>
void IcfBuilder<T>::UpdateModel(
    const T& time_step, const IcfLinearFeedbackGains<T>* actuation_feedback,
    const IcfLinearFeedbackGains<T>* external_feedback,
    IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);

  model->UpdateTimeStep(time_step);

  // N.B. external forces must come first, followed by actuation.
#if 0
  // This DRAKE_DEMAND is not correct. We only add gain constraints for cliques
  // that are actuated (see line 685), so it's sometimes the case that the 0 <
  // gain_constraints.num_constraints() < model->num_cliques().
  auto& gain_constraints = model->gain_constraints_pool();
  DRAKE_DEMAND(gain_constraints.num_constraints() ==
               model->num_cliques() *
                   ((external_feedback != nullptr ? 1 : 0) +
                    (actuation_feedback != nullptr ? 1 : 0)));
#endif
  if (external_feedback != nullptr) {
    const VectorX<T>& Ke = external_feedback->K;
    const VectorX<T>& be = external_feedback->b;
    SetExternalGainConstraints(Ke, be, model);
  }
  if (actuation_feedback != nullptr) {
    const VectorX<T>& Ku = actuation_feedback->K;
    const VectorX<T>& bu = actuation_feedback->b;
    // N.B. actuation constraint indices in the pool depend on whether external
    // or not external forces are present in the model.
    SetActuationGainConstraints(Ku, bu, external_feedback != nullptr, model);
  }
}

template <typename T>
void IcfBuilder<T>::AllocatePatchConstraints(IcfModel<T>* model) const {
  // N.B. This assumes that geometry info has already been computed
  DRAKE_ASSERT(model != nullptr);
  PatchConstraintsPool<T>& patches = model->patch_constraints_pool();
  const int num_surfaces = surfaces_.size();

  // First we'll get the number of contact pairs for point contact. There is one
  // pair per contact patch with point contact.
  const int num_point_contacts = point_pairs_.size();
  std::vector<int> num_pairs_per_patch(num_point_contacts, 1);

  // Now we'll do hydro contact. With hydro we typically have multiple contact
  // pairs in each patch.
  for (int surface = 0; surface < num_surfaces; ++surface) {
    const auto& s = surfaces_[surface];
    num_pairs_per_patch.push_back(s.num_faces());
  }

  patches.Resize(num_pairs_per_patch);

  // Set the stiction tolerance (stiction velocity, in m/s). Unlike other
  // contact parameters (stiffness, dissipation, friction coefficients, etc.),
  // this regularization parameter is shared by all contacts.
  patches.set_stiction_tolerance(plant().stiction_tolerance());
}

template <typename T>
void IcfBuilder<T>::AllocateCouplerConstraints(IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);
  const std::map<MultibodyConstraintId, CouplerConstraintSpec>& specs_map =
      plant().get_coupler_constraint_specs();
  CouplerConstraintsPool<T>& couplers = model->coupler_constraints_pool();
  couplers.Resize(specs_map.size());
}

template <typename T>
void IcfBuilder<T>::SetCouplerConstraints(const systems::Context<T>& context,
                                          IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);

  const SpanningForest& forest = GetInternalTree(plant()).forest();
  const std::map<MultibodyConstraintId, CouplerConstraintSpec>& specs_map =
      plant().get_coupler_constraint_specs();

  CouplerConstraintsPool<T>& couplers = model->coupler_constraints_pool();

  int index = 0;
  for (const auto& [id, spec] : specs_map) {
    const Joint<T>& joint0 = plant().get_joint(spec.joint0_index);
    const Joint<T>& joint1 = plant().get_joint(spec.joint1_index);

    const int dof0 = joint0.velocity_start();
    const int dof1 = joint1.velocity_start();
    const TreeIndex tree0 = forest.v_to_tree_index(dof0);
    const TreeIndex tree1 = forest.v_to_tree_index(dof1);

    // Sanity check.
    DRAKE_DEMAND(tree0.is_valid() && tree1.is_valid());

    const int clique0 = tree_to_clique(tree0);
    const int clique1 = tree_to_clique(tree1);

    if (clique0 != clique1) {
      throw std::logic_error(
          "IcfBuilder: Couplers are only allowed within DoFs in the same "
          "tree.");
    }

    const T q0 = joint0.GetOnePosition(context);
    const T q1 = joint1.GetOnePosition(context);

    // DOFs local to their tree.
    const int tree_dof0 = dof0 - forest.trees(tree0).v_start();
    const int tree_dof1 = dof1 - forest.trees(tree0).v_start();

    couplers.Set(index, clique0, tree_dof0, tree_dof1, q0, q1, spec.gear_ratio,
                 spec.offset);
    ++index;
  }
}

template <typename T>
void IcfBuilder<T>::AllocateLimitConstraints(IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);

  LimitConstraintsPool<T>& limits = model->limit_constraints_pool();

  limits.Resize(limited_clique_sizes_);
}

template <typename T>
void IcfBuilder<T>::SetLimitConstraints(const systems::Context<T>& context,
                                        IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);
  using std::isinf;

  const SpanningForest& forest = GetInternalTree(plant()).forest();

  LimitConstraintsPool<T>& limits = model->limit_constraints_pool();

  for (JointIndex joint_index : plant().GetJointIndices()) {
    const Joint<T>& joint = plant().get_joint(joint_index);
    // We only support limits for 1 DOF joints for which we know that q̇ = v.
    if (joint.num_positions() == 1 && joint.num_velocities() == 1) {
      const int velocity_start = joint.velocity_start();
      const TreeIndex tree_index = forest.v_to_tree_index(velocity_start);
      const int tree_nv = forest.trees(tree_index).nv();
      DRAKE_ASSERT(tree_nv >= 0);
      const int tree_velocity_start = forest.trees(tree_index).v_start();

      const int tree_dof = velocity_start - tree_velocity_start;
      const int clique = tree_to_clique(tree_index);

      const double ql = joint.position_lower_limits()[0];
      const double qu = joint.position_upper_limits()[0];
      const T& q0 = joint.GetOnePosition(context);

      if (!isinf(ql) || !isinf(qu)) {
        // Add constraint for this dof in clique.
        const int index = clique_to_limit_constraint_[clique];
        limits.Set(index, clique, tree_dof, q0, ql, qu);
      }
    }
  }
}

template <typename T>
void IcfBuilder<T>::SetPatchConstraintsForPointContact(
    const systems::Context<T>& context, IcfModel<T>* model) const {
  const int num_point_contacts = point_pairs_.size();
  if (num_point_contacts == 0) {
    return;
  }

  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);

  PatchConstraintsPool<T>& patches = model->patch_constraints_pool();

  // Fill in the point contact pairs.
  for (int point_pair_index = 0; point_pair_index < num_point_contacts;
       ++point_pair_index) {
    const geometry::PenetrationAsPointPair<T>& pp =
        point_pairs_[point_pair_index];

    // Retrieve participating geometries and bodies.
    const geometry::GeometryId Mid = pp.id_A;
    const geometry::GeometryId Nid = pp.id_B;
    const geometry::FrameId frame_Mid = inspector.GetFrameId(Mid);
    const geometry::FrameId frame_Nid = inspector.GetFrameId(Nid);
    const RigidBody<T>* bodyM = plant().GetBodyFromFrameId(frame_Mid);
    const RigidBody<T>* bodyN = plant().GetBodyFromFrameId(frame_Nid);
    DRAKE_DEMAND(bodyM != nullptr && bodyN != nullptr);

    const bool M_not_anchored = !plant().IsAnchored(*bodyM);
    const bool N_not_anchored = !plant().IsAnchored(*bodyN);
    // Sanity check at least one body is not anchored.
    DRAKE_DEMAND(M_not_anchored || N_not_anchored);

    // By convention, body B is always not-anchored.
    const RigidBody<T>* bodyB = N_not_anchored ? bodyN : bodyM;
    const RigidBody<T>* bodyA = bodyB == bodyN ? bodyM : bodyN;

    // Kinematics.
    const auto& X_WA = bodyA->EvalPoseInWorld(context);
    const auto& X_WB = bodyB->EvalPoseInWorld(context);
    const Vector3<T>& p_WAo = X_WA.translation();
    const Vector3<T>& p_WBo = X_WB.translation();
    const Vector3<T> p_AB_W = p_WBo - p_WAo;

    // Material properties.
    const T& kM = stiffness_.at(Mid);
    const T& kN = stiffness_.at(Nid);
    const T k = GetCombinedPointContactStiffness(kM, kN);

    const T& dM = dissipation_.at(Mid);
    const T& dN = dissipation_.at(Nid);
    const T d = CombineHuntCrossleyDissipation(kM, kN, dM, dN);

    // Friction properties
    const auto& mu_A = friction_.at(Mid);
    const auto& mu_B = friction_.at(Nid);
    CoulombFriction<double> mu =
        CalcContactFrictionFromSurfaceProperties(mu_A, mu_B);

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kM + kN;
    const T wM = (denom == 0 ? 0.5 : kM / denom);
    const T wN = (denom == 0 ? 0.5 : kN / denom);
    const Vector3<T> p_WC = wM * pp.p_WCa + wN * pp.p_WCb;
    const Vector3<T> p_BoC_W = p_WC - p_WBo;

    // Normal must always point from A to B, by convention.
    const Vector3<T>& nhat_NM_W = pp.nhat_BA_W;  // Points from N into M.
    const Vector3<T> nhat_AB_W = bodyB == bodyN ? -nhat_NM_W : nhat_NM_W;
    const T fn0 = k * pp.depth;

    // For point contact we add single-pair patches.
    patches.SetPatch(point_pair_index, bodyA->index(), bodyB->index(), d,
                     mu.static_friction(), mu.dynamic_friction(), p_AB_W);
    patches.SetPair(point_pair_index, 0, p_BoC_W, nhat_AB_W, fn0, k);
  }
}

template <typename T>
void IcfBuilder<T>::SetPatchConstraintsForHydroelasticContact(
    const systems::Context<T>& context, IcfModel<T>* model) const {
  using std::max;
  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);

  // TODO(amcastro-tri): This should be retrieved from the default contact
  // properties.
  const double kDefaultDissipation = 50.0;

  const int num_surfaces = surfaces_.size();

  PatchConstraintsPool<T>& patches = model->patch_constraints_pool();

  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    // To get the patch index, we need to account for the fact that there may
    // be some point contact pairs that get added before this
    const int patch_index = surface_index + point_pairs_.size();

    const auto& s = surfaces_[surface_index];
    const bool M_is_compliant = s.HasGradE_M();
    const bool N_is_compliant = s.HasGradE_N();
    DRAKE_DEMAND(M_is_compliant || N_is_compliant);

    // Retrieve participating geometries and bodies.
    const geometry::FrameId Mid = inspector.GetFrameId(s.id_M());
    const geometry::FrameId Nid = inspector.GetFrameId(s.id_N());
    const RigidBody<T>* bodyM = plant().GetBodyFromFrameId(Mid);
    const RigidBody<T>* bodyN = plant().GetBodyFromFrameId(Nid);
    DRAKE_DEMAND(bodyM != nullptr && bodyN != nullptr);

    const bool M_not_anchored = !plant().IsAnchored(*bodyM);
    const bool N_not_anchored = !plant().IsAnchored(*bodyN);
    // Sanity check at least one body is not anchored.
    DRAKE_DEMAND(M_not_anchored || N_not_anchored);

    // By convention, body B is always not-anchored.
    const RigidBody<T>* bodyB = N_not_anchored ? bodyN : bodyM;
    const RigidBody<T>* bodyA = bodyB == bodyN ? bodyM : bodyN;

    // Get compliance properties.
    const T Em = multibody::internal::GetHydroelasticModulus(
        s.id_M(), std::numeric_limits<double>::infinity(), inspector);
    const T En = multibody::internal::GetHydroelasticModulus(
        s.id_N(), std::numeric_limits<double>::infinity(), inspector);
    const T d = multibody::internal::GetCombinedHuntCrossleyDissipation(
        s.id_M(), s.id_N(), Em, En, kDefaultDissipation, inspector);

    // Get friction properties
    const auto& mu_A = GetCoulombFriction(s.id_M(), inspector);
    const auto& mu_B = GetCoulombFriction(s.id_N(), inspector);
    CoulombFriction<double> mu =
        CalcContactFrictionFromSurfaceProperties(mu_A, mu_B);

    const auto& X_WA = bodyA->EvalPoseInWorld(context);
    const auto& X_WB = bodyB->EvalPoseInWorld(context);
    const Vector3<T>& p_WAo = X_WA.translation();
    const Vector3<T>& p_WBo = X_WB.translation();
    const Vector3<T> p_AB_W = p_WBo - p_WAo;

    patches.SetPatch(patch_index, bodyA->index(), bodyB->index(), d,
                     mu.static_friction(), mu.dynamic_friction(), p_AB_W);

    for (int face = 0; face < s.num_faces(); ++face) {
      const T Ae = s.area(face);  // Face element area.
      const Vector3<T>& nhat_NM_W = s.face_normal(face);
      const T gM = M_is_compliant ? s.EvaluateGradE_M_W(face).dot(nhat_NM_W)
                                  : std::numeric_limits<double>::infinity();
      const T gN = N_is_compliant ? -s.EvaluateGradE_N_W(face).dot(nhat_NM_W)
                                  : std::numeric_limits<double>::infinity();

      const T g = 1.0 / (1.0 / gM + 1.0 / gN);
      const Vector3<T>& p_WC = s.centroid(face);
      const Vector3<T> p_BoC_W = p_WC - p_WBo;

      // Normal must always point from A to B, by convention.
      const Vector3<T> nhat_AB_W = bodyB == bodyN ? -nhat_NM_W : nhat_NM_W;

      // Pressure at the quadrature point.
      const Vector3<T> tri_centroid_barycentric(1 / 3., 1 / 3., 1 / 3.);
      const T p0 = s.is_triangle()
                       ? s.tri_e_MN().Evaluate(face, tri_centroid_barycentric)
                       : s.poly_e_MN().EvaluateCartesian(face, p_WC);

      const T fn0 = Ae * p0;

      // For hydroelastic contact, it is sometimes (though rarely) the case that
      // Ae * g < 0 due to numerical issues. We therefore clamp the stiffness to
      // ensure that the problem is always convex.
      const T k = max(Ae * g, 0.0);

      patches.SetPair(patch_index, face, p_BoC_W, nhat_AB_W, fn0, k);
    }
  }
}

template <typename T>
void IcfBuilder<T>::AllocateGainConstraints(IcfModel<T>* model, bool actuation,
                                            bool external_forces) const {
  DRAKE_DEMAND(model != nullptr);

  auto& gain_constraints = model->gain_constraints_pool();
  std::vector<int> gain_constraint_sizes(0);

  // The first gain constraints are for external forces, τ = Ke v + be. These
  // are not subject to effort limits.
  if (external_forces) {
    for (int c = 0; c < model->num_cliques(); ++c) {
      gain_constraint_sizes.push_back(model->clique_size(c));
    }
  }

  // Subsequent constraints are for actuation, u = Ku v + bu. These are
  // subject to effort limits.
  if (actuation) {
    for (int c = 0; c < model->num_cliques(); ++c) {
      if (clique_nu_[c] > 0) {
        gain_constraint_sizes.push_back(model->clique_size(c));
      }
    }
  }

  gain_constraints.Resize(gain_constraint_sizes);
}

template <typename T>
void IcfBuilder<T>::SetActuationGainConstraints(const VectorX<T>& Ku,
                                                const VectorX<T>& bu,
                                                bool has_external_forces,
                                                IcfModel<T>* model) const {
  DRAKE_DEMAND(model != nullptr);
  DRAKE_DEMAND(model->num_velocities() == plant().num_velocities());
  const int nv = model->num_velocities();
  DRAKE_DEMAND(Ku.size() == nv);
  DRAKE_DEMAND(bu.size() == nv);

  // Quick no-op exit.
  if (plant().num_actuators() == 0) return;

  auto& gain_constraints = model->gain_constraints_pool();

  int index = 0;  // constraint index
  if (has_external_forces) {
    // When external forces are present, actuation constraints come after all
    // the external force constraints in the pool.
    index = model->num_cliques();
  }

  for (int c = 0; c < model->num_cliques(); ++c) {
    if (clique_nu_[c] == 0) continue;

    const auto Ku_c = model->clique_segment(c, Ku);
    const auto bu_c = model->clique_segment(c, bu);
    const auto e_c = model->clique_segment(c, effort_limits_);

    gain_constraints.Set(index, c, Ku_c, bu_c, e_c);
    ++index;
  }
}

template <typename T>
void IcfBuilder<T>::SetExternalGainConstraints(const VectorX<T>& Ke,
                                               const VectorX<T>& be,
                                               IcfModel<T>* model) const {
  DRAKE_DEMAND(model != nullptr);
  const int nv = model->num_velocities();
  DRAKE_DEMAND(Ke.size() == nv);
  DRAKE_DEMAND(be.size() == nv);

  auto& gain_constraints = model->gain_constraints_pool();

  for (int c = 0; c < model->num_cliques(); ++c) {
    const auto Ke_c = model->clique_segment(c, Ke);
    const auto be_c = model->clique_segment(c, be);

    // We use infinite effort limits for external force gain constraints.
    // TODO(vincekurtz): consider dealing with gain constraints by just adding
    // to A += dt * K and r += dt * b directly instead of using gain
    // constraints.
    const int clique_nv = model->clique_size(c);
    const VectorX<T> e =
        VectorX<T>::Constant(clique_nv, std::numeric_limits<T>::infinity());

    gain_constraints.Set(c, c, Ke_c, be_c, e);
  }
}

template <typename T>
IcfBuilder<T>::Scratch::Scratch(const MultibodyPlant<T>& plant)
    : forces(plant) {
  const int nv = plant.num_velocities();
  J_V_WB.resize(6, nv);
  accelerations.resize(nv);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfBuilder);
