#include "drake/multibody/contact_solvers/icf/icf_builder.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <utility>

#include "drake/geometry/scene_graph_config.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/multibody_plant_icf_attorney.h"
#include "drake/multibody/topology/graph.h"

using drake::multibody::CalcContactFrictionFromSurfaceProperties;
using drake::multibody::internal::CouplerConstraintSpec;
using drake::multibody::internal::GetCombinedHuntCrossleyDissipation;
using drake::multibody::internal::GetCombinedPointContactStiffness;
using drake::multibody::internal::GetCoulombFriction;
using drake::multibody::internal::GetHuntCrossleyDissipation;
using drake::multibody::internal::GetPointContactStiffness;
using drake::multibody::internal::LinkJointGraph;
using drake::multibody::internal::SpanningForest;
using drake::multibody::internal::TreeIndex;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

// TODO(rpoyner-tri): This apparent near-copy of
// GetCombinedHuntCrossleyDissipation is only used for point contact
// calculations. Do we need it?  A likely story is that this rewrite came about
// to avoid the proximity-properties dives hardcoded into the original version.
template <typename T>
T CombinePointContactHuntCrossleyDissipation(const T& stiffness_A,
                                             const T& stiffness_B,
                                             const T& dissipation_A,
                                             const T& dissipation_B) {
  if (stiffness_A == kInf) return dissipation_B;
  if (stiffness_B == kInf) return dissipation_A;

  // Return zero dissipation if both stiffness values are zero.
  const T denom = stiffness_A + stiffness_B;
  if (denom == 0.0) return 0.0;

  // At this point we know both geometries are compliant and at least one of
  // them has non-zero stiffness (denom != 0).
  return (stiffness_B / denom) * dissipation_A +
         (stiffness_A / denom) * dissipation_B;
}
}  // namespace

template <typename T>
IcfBuilder<T>::IcfBuilder(const MultibodyPlant<T>* plant)
    : plant_(DRAKE_DEREF(plant)) {
  ValidatePlant();
}

template <typename T>
void IcfBuilder<T>::UpdateModel(
    const systems::Context<T>& context, const T& time_step,
    const IcfLinearFeedbackGains<T>* actuation_feedback,
    const IcfLinearFeedbackGains<T>* external_feedback, IcfModel<T>* model) {
  DRAKE_ASSERT(model != nullptr);
  ValidateContext(context);
  const SpanningForest& forest = GetInternalTree(plant_).forest();
  const int nv = forest.num_velocities();

  std::unique_ptr<IcfParameters<T>> params = model->ReleaseParameters();
  DRAKE_DEMAND(params != nullptr);
  // Detect if this is the first time we are setting up params.
  if (params->v0.size() == 0) {
    DRAKE_DEMAND(params->M0.size() == 0);
    DRAKE_DEMAND(params->D0.size() == 0);
    DRAKE_DEMAND(params->k0.size() == 0);
    DRAKE_DEMAND(params->J_WB.size() == 0);
    DRAKE_DEMAND(params->body_mass.empty());
    DRAKE_DEMAND(params->body_to_clique.empty());
    DRAKE_DEMAND(params->body_is_floating.empty());
    DRAKE_DEMAND(params->clique_sizes.empty());
    DRAKE_DEMAND(params->clique_start.empty());

    // Yes, Virginia, this is the first time we are setting the params.

    // Allocate memory blocks now, even for values updated on each step.
    params->v0.resize(nv);
    params->M0.resize(nv, nv);
    params->D0.resize(nv);
    params->k0.resize(nv);
    params->body_mass.resize(plant_.num_bodies());
    params->J_WB.Resize(plant_.num_bodies(), 6,
                        plant_facts_.body_jacobian_cols);

    // Clique membership, and body floating status are defined at builder
    // construction time, since they depend only on the plant and not on the
    // current state (q0, v0).
    params->body_to_clique = plant_facts_.body_to_clique;
    params->body_is_floating = plant_facts_.body_is_floating;

    // Set the sparsity pattern for the dynamics matrix A = M + δt D.
    params->clique_sizes = plant_facts_.clique_sizes;
    params->clique_start.resize(plant_facts_.clique_sizes.size() + 1);
    params->clique_start[0] = 0;
    std::partial_sum(params->clique_sizes.begin(), params->clique_sizes.end(),
                     params->clique_start.begin() + 1);

    DRAKE_DEMAND(params->v0.size() == nv);
    DRAKE_DEMAND(params->M0.size() == nv * nv);
    DRAKE_DEMAND(params->D0.size() == nv);
    DRAKE_DEMAND(params->k0.size() == nv);
    DRAKE_DEMAND(params->J_WB.size() == plant_.num_bodies());
    DRAKE_DEMAND(ssize(params->body_mass) == plant_.num_bodies());
    DRAKE_DEMAND(ssize(params->body_to_clique) == plant_.num_bodies());
    DRAKE_DEMAND(ssize(params->body_is_floating) == plant_.num_bodies());
    DRAKE_DEMAND(params->clique_sizes.size() ==
                 plant_facts_.clique_sizes.size());
    DRAKE_DEMAND(params->clique_start.size() ==
                 plant_facts_.clique_sizes.size() + 1);
  }

  // Set the time step δt and initial velocities v₀
  params->time_step = time_step;
  params->v0 = plant_.GetVelocities(context);

  // Set the (dense) mass matrix M₀.
  MatrixX<T>& M = params->M0;
  plant_.CalcMassMatrix(context, &M);

  // Set joint damping D₀.
  params->D0 = plant_.EvalJointDampingCache(context);

  // Compute nonlinear bias terms k₀.
  MultibodyForces<T>& forces = scratch_.forces;
  plant_.CalcForceElementsContribution(context, &forces);
  const VectorX<T>& accelerations = scratch_.accelerations;
  params->k0 = plant_.CalcInverseDynamics(context, accelerations, forces);

  // Compute spatial velocity Jacobians J_WB for all bodies.
  const auto& world_frame = plant_.world_frame();
  EigenPool<Matrix6X<T>>& J_WB = params->J_WB;

  for (int b = 0; b < plant_.num_bodies(); ++b) {
    const auto& body = plant_.get_body(BodyIndex(b));

    if (!plant_.IsAnchored(body)) {
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
        plant_.CalcJacobianSpatialVelocity(
            context, JacobianWrtVariable::kV, body.body_frame(),
            Vector3<T>::Zero(), world_frame, world_frame, &J_V_WB);
        Jv_WBc_W = J_V_WB.middleCols(vt_start, nt);
      }
    }
  }

  // Body mass is affected by context parameters, so needs to be refreshed
  // every step. TODO(rpoyner-tri): consider using a MultibodyPlant cache entry
  // (maybe like frame_body_pose_cache?) to track masses.
  for (int b = 0; b < plant_.num_bodies(); ++b) {
    const auto& body = plant_.get_body(BodyIndex(b));
    // If the body has zero mass, we assign it the mass of its composite.
    // TODO(amcastro-tri): consider using forest.link_composites() in
    // combination with LinkJointGraph::Link::composite() to precompute
    // composite's masses.
    params->body_mass[b] = body.get_mass(context);
    if (params->body_mass[b] == 0.0) {
      const LinkJointGraph& graph = GetInternalTree(plant_).graph();
      const auto composite = graph.GetLinksWeldedTo(body.index());
      T composite_mass = 0.0;
      for (BodyIndex c : composite) {
        composite_mass += plant_.get_body(c).get_mass(context);
      }
      if (composite_mass == 0.0) {
        throw std::logic_error(
            fmt::format("Composite for body '{}' has zero mass.", body.name()));
      }
      params->body_mass[b] = composite_mass;
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
  const bool has_actuation_forces = actuation_feedback != nullptr;
  const bool has_external_forces = external_feedback != nullptr;
  AllocateGainConstraints(model, has_actuation_forces, has_external_forces);
  if (has_external_forces) {
    const VectorX<T>& Ke = external_feedback->K;
    const VectorX<T>& be = external_feedback->b;
    SetExternalGainConstraints(Ke, be, model);
  }
  if (has_actuation_forces) {
    const VectorX<T>& Ku = actuation_feedback->K;
    const VectorX<T>& bu = actuation_feedback->b;
    // N.B. actuation constraint indices in the pool depend on whether or not
    // external forces are present in the model.
    SetActuationGainConstraints(Ku, bu, has_external_forces, model);
  }

  // Define the sparsity pattern, which defines which cliques are connected by
  // constraints.
  model->SetSparsityPattern();
}

template <typename T>
void IcfBuilder<T>::UpdateFeedbackGains(
    const IcfLinearFeedbackGains<T>* actuation_feedback,
    const IcfLinearFeedbackGains<T>* external_feedback,
    IcfModel<T>* model) const {
  // N.B. external forces must come first, followed by actuation.

  const bool has_actuation = actuation_feedback != nullptr;
  const bool has_external_forces = external_feedback != nullptr;
  const auto& gain_constraints = model->gain_constraints_pool();
  DRAKE_DEMAND(gain_constraints.num_constraints() ==
               (model->num_cliques() * has_external_forces) +
                   (plant_facts_.num_actuation_constraints * has_actuation));

  if (has_external_forces) {
    const VectorX<T>& Ke = external_feedback->K;
    const VectorX<T>& be = external_feedback->b;
    SetExternalGainConstraints(Ke, be, model);
  }
  if (has_actuation) {
    const VectorX<T>& Ku = actuation_feedback->K;
    const VectorX<T>& bu = actuation_feedback->b;
    // N.B. actuation constraint indices in the pool depend on whether or not
    // external forces are present in the model.
    SetActuationGainConstraints(Ku, bu, has_external_forces, model);
  }
}

template <typename T>
void IcfBuilder<T>::ValidatePlant() {
  DRAKE_DEMAND(plant_.is_finalized());

  // Revisit this condition when deformable support is implemented. See #23768.
  DRAKE_THROW_UNLESS(plant_.deformable_model().num_bodies() == 0);

  // Revisit this condition as constraints are implemented. See issues #23759,
  // #23760, #23762, #23763.
  if (plant_.num_constraints() - plant_.num_coupler_constraints() > 0) {
    throw std::logic_error(fmt::format(
        "The CENIC integrator does not yet support some constraints, but "
        "they are present in the given MultibodyPlant: {} distance "
        "constraint(s), {} ball constraint(s), {} weld constraint(s), {} "
        "tendon constraint(s)",
        plant_.num_distance_constraints(), plant_.num_ball_constraints(),
        plant_.num_weld_constraints(), plant_.num_tendon_constraints()));
  }
}

template <typename T>
void IcfBuilder<T>::ValidateContext(const systems::Context<T>& context) {
  // Revisit this condition when joint locking support is implemented. See
  // #23764.
  for (const JointIndex& j : plant_.GetJointIndices()) {
    if (plant_.get_joint(j).is_locked(context)) {
      throw std::runtime_error(
          fmt::format("The CENIC integrator does not yet support joint "
                      "locking, but at least joint {} is locked",
                      j));
    }
  }
}

template <typename T>
void IcfBuilder<T>::CalcGeometryContactData(
    const systems::Context<T>& context) {
  surfaces_.clear();
  point_pairs_.clear();

  const auto& query_object =
      plant_.get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);

  switch (plant_.get_contact_model()) {
    case ContactModel::kPoint: {
      point_pairs_ = query_object.ComputePointPairPenetration();
      break;
    }
    case ContactModel::kHydroelastic: {
      surfaces_ = query_object.ComputeContactSurfaces(
          plant_.get_contact_surface_representation());
      break;
    }
    case ContactModel::kHydroelasticWithFallback: {
      query_object.ComputeContactSurfacesWithFallback(
          plant_.get_contact_surface_representation(), &surfaces_,
          &point_pairs_);
      break;
    }
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
  patches.set_stiction_tolerance(plant_.stiction_tolerance());
}

template <typename T>
void IcfBuilder<T>::AllocateCouplerConstraints(IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);
  const std::map<MultibodyConstraintId, CouplerConstraintSpec>& specs_map =
      plant_.get_coupler_constraint_specs();
  CouplerConstraintsPool<T>& couplers = model->coupler_constraints_pool();
  couplers.Resize(specs_map.size());
}

template <typename T>
void IcfBuilder<T>::SetCouplerConstraints(const systems::Context<T>& context,
                                          IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);

  const SpanningForest& forest = GetInternalTree(plant_).forest();
  const std::map<MultibodyConstraintId, CouplerConstraintSpec>& specs_map =
      plant_.get_coupler_constraint_specs();

  CouplerConstraintsPool<T>& couplers = model->coupler_constraints_pool();

  int index = 0;
  for (const auto& [id, spec] : specs_map) {
    const Joint<T>& joint0 = plant_.get_joint(spec.joint0_index);
    const Joint<T>& joint1 = plant_.get_joint(spec.joint1_index);

    const int dof0 = joint0.velocity_start();
    const int dof1 = joint1.velocity_start();
    const TreeIndex tree0 = forest.v_to_tree_index(dof0);
    const TreeIndex tree1 = forest.v_to_tree_index(dof1);

    // Sanity check.
    DRAKE_DEMAND(tree0.is_valid() && tree1.is_valid());

    const int clique0 = tree_to_clique(tree0);
    const int clique1 = tree_to_clique(tree1);

    if (clique0 != clique1) {
      // TODO(#23992): this limitation is a regression from SAP coupler
      // constraints. It should be removed.
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

  limits.Resize(plant_facts_.limited_clique_sizes);
}

template <typename T>
void IcfBuilder<T>::SetLimitConstraints(const systems::Context<T>& context,
                                        IcfModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);
  using std::isinf;

  const SpanningForest& forest = GetInternalTree(plant_).forest();

  LimitConstraintsPool<T>& limits = model->limit_constraints_pool();

  for (JointIndex joint_index : plant_.GetJointIndices()) {
    const Joint<T>& joint = plant_.get_joint(joint_index);
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
        const int index = plant_facts_.clique_to_limit_constraint[clique];
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
      plant_.EvalSceneGraphInspector(context);
  RefreshGeometryDetails(inspector);

  PatchConstraintsPool<T>& patches = model->patch_constraints_pool();

  // Fill in the point contact pairs.
  for (int point_pair_index = 0; point_pair_index < num_point_contacts;
       ++point_pair_index) {
    const geometry::PenetrationAsPointPair<T>& point_pair =
        point_pairs_[point_pair_index];

    // Retrieve participating geometries and bodies.
    const geometry::GeometryId Mid = point_pair.id_A;
    const geometry::GeometryId Nid = point_pair.id_B;
    const geometry::FrameId frame_Mid = inspector.GetFrameId(Mid);
    const geometry::FrameId frame_Nid = inspector.GetFrameId(Nid);
    const RigidBody<T>* bodyM = plant_.GetBodyFromFrameId(frame_Mid);
    const RigidBody<T>* bodyN = plant_.GetBodyFromFrameId(frame_Nid);
    DRAKE_DEMAND(bodyM != nullptr && bodyN != nullptr);

    BodiesSortedByAnchorage sorted(plant_, bodyM, bodyN);

    // Kinematics.
    const auto& X_WA = sorted.bodyA->EvalPoseInWorld(context);
    const auto& X_WB = sorted.bodyB->EvalPoseInWorld(context);
    const Vector3<T>& p_WAo = X_WA.translation();
    const Vector3<T>& p_WBo = X_WB.translation();
    const Vector3<T> p_AB_W = p_WBo - p_WAo;

    // Material properties.
    const GeometryDetails& Mid_details = geometry_details_.at(Mid);
    const GeometryDetails& Nid_details = geometry_details_.at(Nid);
    const T& kM = Mid_details.stiffness;
    const T& kN = Nid_details.stiffness;
    const T k = GetCombinedPointContactStiffness(kM, kN);

    const T& dM = Mid_details.dissipation;
    const T& dN = Nid_details.dissipation;
    const T d = CombinePointContactHuntCrossleyDissipation(kM, kN, dM, dN);

    // Friction properties.
    const CoulombFriction<double> mu = CalcContactFrictionFromSurfaceProperties(
        Mid_details.friction, Nid_details.friction);

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kM + kN;
    const T wM = (denom == 0 ? 0.5 : kM / denom);
    const T wN = (denom == 0 ? 0.5 : kN / denom);
    const Vector3<T> p_WC = wM * point_pair.p_WCa + wN * point_pair.p_WCb;
    const Vector3<T> p_BoC_W = p_WC - p_WBo;

    // Normal must always point from A to B, by convention.
    const Vector3<T>& nhat_NM_W =
        point_pair.nhat_BA_W;  // Points from N into M.
    const Vector3<T> nhat_AB_W = sorted.bodyB == bodyN ? -nhat_NM_W : nhat_NM_W;
    const T fn0 = k * point_pair.depth;

    // For point contact we add single-pair patches.
    patches.SetPatch(point_pair_index, sorted.bodyA->index(),
                     sorted.bodyB->index(), d, mu.static_friction(),
                     mu.dynamic_friction(), p_AB_W);
    patches.SetPair(point_pair_index, 0, p_BoC_W, nhat_AB_W, fn0, k);
  }
}

template <typename T>
void IcfBuilder<T>::SetPatchConstraintsForHydroelasticContact(
    const systems::Context<T>& context, IcfModel<T>* model) const {
  using std::max;
  const geometry::SceneGraphInspector<T>& inspector =
      plant_.EvalSceneGraphInspector(context);

  const double kDefaultDissipation =
      geometry::DefaultProximityProperties{}.hunt_crossley_dissipation.value();

  const int num_surfaces = surfaces_.size();

  PatchConstraintsPool<T>& patches = model->patch_constraints_pool();

  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    // To get the patch index, we need to account for the fact that there may
    // be some point contact pairs that get added before this.
    const int patch_index = surface_index + point_pairs_.size();

    const geometry::ContactSurface<T>& s = surfaces_[surface_index];
    const bool M_is_compliant = s.HasGradE_M();
    const bool N_is_compliant = s.HasGradE_N();
    DRAKE_DEMAND(M_is_compliant || N_is_compliant);

    // Retrieve participating geometries and bodies.
    const geometry::FrameId frame_Mid = inspector.GetFrameId(s.id_M());
    const geometry::FrameId frame_Nid = inspector.GetFrameId(s.id_N());
    const RigidBody<T>* bodyM = plant_.GetBodyFromFrameId(frame_Mid);
    const RigidBody<T>* bodyN = plant_.GetBodyFromFrameId(frame_Nid);
    DRAKE_DEMAND(bodyM != nullptr && bodyN != nullptr);

    BodiesSortedByAnchorage sorted(plant_, bodyM, bodyN);

    // Get compliance properties.
    const T Em =
        multibody::internal::GetHydroelasticModulus(s.id_M(), kInf, inspector);
    const T En =
        multibody::internal::GetHydroelasticModulus(s.id_N(), kInf, inspector);
    const T d = GetCombinedHuntCrossleyDissipation(
        s.id_M(), s.id_N(), Em, En, kDefaultDissipation, inspector);

    // Get friction properties.
    const CoulombFriction<double> mu = CalcContactFrictionFromSurfaceProperties(
        GetCoulombFriction(s.id_M(), inspector),
        GetCoulombFriction(s.id_N(), inspector));

    const auto& X_WA =
        sorted.bodyA->EvalPoseInWorld(context);  // maybe anchored
    const auto& X_WB = sorted.bodyB->EvalPoseInWorld(context);  // dynamic
    const Vector3<T>& p_WAo = X_WA.translation();
    const Vector3<T>& p_WBo = X_WB.translation();
    const Vector3<T> p_AB_W = p_WBo - p_WAo;

    patches.SetPatch(patch_index, sorted.bodyA->index(), sorted.bodyB->index(),
                     d, mu.static_friction(), mu.dynamic_friction(), p_AB_W);

    for (int face = 0; face < s.num_faces(); ++face) {
      const T Ae = s.area(face);  // Face element area.
      const Vector3<T>& nhat_NM_W = s.face_normal(face);
      const T gM =
          M_is_compliant ? s.EvaluateGradE_M_W(face).dot(nhat_NM_W) : kInf;
      const T gN =
          N_is_compliant ? -s.EvaluateGradE_N_W(face).dot(nhat_NM_W) : kInf;

      const T g = 1.0 / (1.0 / gM + 1.0 / gN);
      const Vector3<T>& p_WC = s.centroid(face);
      const Vector3<T> p_BoC_W = p_WC - p_WBo;

      // Normal must always point from A to B, by convention.
      const Vector3<T> nhat_AB_W =
          sorted.bodyB == bodyN ? -nhat_NM_W : nhat_NM_W;

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
void IcfBuilder<T>::AllocateGainConstraints(IcfModel<T>* model,
                                            bool has_actuation,
                                            bool has_external_forces) const {
  DRAKE_DEMAND(model != nullptr);

  auto& gain_constraints = model->gain_constraints_pool();
  std::vector<int> gain_constraint_sizes(
      (model->num_cliques() * has_external_forces) +
      (plant_facts_.num_actuation_constraints * has_actuation));

  // The first gain constraints are for external forces τₑ = -Kₑ⋅v + bₑ. These
  // forces are not subject to effort limits.
  int constraint_index = 0;
  if (has_external_forces) {
    for (int c = 0; c < model->num_cliques(); ++c, ++constraint_index) {
      gain_constraint_sizes[constraint_index] = model->clique_size(c);
    }
  }

  // Subsequent constraints are for actuation, τᵤ = clamp(-Kᵤ⋅v + b, e). These
  // are subject to effort limits.
  if (has_actuation) {
    for (int c = 0; c < model->num_cliques(); ++c) {
      if (plant_facts_.clique_nu[c] > 0) {
        gain_constraint_sizes[constraint_index] = model->clique_size(c);
        ++constraint_index;
      }
    }
  }
  DRAKE_DEMAND(constraint_index == ssize(gain_constraint_sizes));

  gain_constraints.Resize(gain_constraint_sizes);
}

template <typename T>
void IcfBuilder<T>::SetActuationGainConstraints(const VectorX<T>& Ku,
                                                const VectorX<T>& bu,
                                                bool has_external_forces,
                                                IcfModel<T>* model) const {
  DRAKE_DEMAND(model != nullptr);
  DRAKE_DEMAND(model->num_velocities() == plant_.num_velocities());
  const int nv = model->num_velocities();
  DRAKE_DEMAND(Ku.size() == nv);
  DRAKE_DEMAND(bu.size() == nv);

  auto& gain_constraints = model->gain_constraints_pool();

  int constraint_index = 0;
  if (has_external_forces) {
    // When external forces are present, actuation constraints come after all
    // the external force constraints in the pool.
    constraint_index = model->num_cliques();
  }

  // Collect joint effort limits, indexed by velocity.
  VectorX<T>& effort_limits = scratch_.effort_limits;
  effort_limits.setConstant(kInf);
  for (JointActuatorIndex actuator_index : plant_.GetJointActuatorIndices()) {
    const JointActuator<T>& actuator =
        plant_.get_joint_actuator(actuator_index);
    const Joint<T>& joint = actuator.joint();
    effort_limits[joint.velocity_start()] = actuator.effort_limit();
  }

  // Add constraints for cliques with actuation.
  for (int c = 0; c < model->num_cliques(); ++c) {
    if (plant_facts_.clique_nu[c] == 0) {
      continue;
    }

    const auto Ku_c = model->clique_segment(c, Ku);
    const auto bu_c = model->clique_segment(c, bu);
    const auto e_c = model->clique_segment(c, effort_limits);

    gain_constraints.Set(constraint_index, c, Ku_c, bu_c, e_c);
    ++constraint_index;
  }
  // Check that we exactly filled the constraint pool.
  DRAKE_DEMAND(constraint_index == gain_constraints.num_constraints());
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
    // TODO(rpoyner-tri): this spelling of `e` and the signature of Set() cost
    // us at least one needless memory allocation.
    const VectorX<T> e = VectorX<T>::Constant(clique_nv, kInf);

    gain_constraints.Set(c, c, Ke_c, be_c, e);
  }
}

template <typename T>
void IcfBuilder<T>::RefreshGeometryDetails(
    const geometry::SceneGraphInspector<T>& inspector) const {
  if (inspector.geometry_version().IsSameAs(geometry_version_,
                                            geometry::Role::kProximity)) {
    return;
  }
  geometry_version_ = inspector.geometry_version();

  // Retrieve constant model parameters.
  const multibody::internal::ContactByPenaltyMethodParameters& contact_params =
      multibody::internal::MultibodyPlantIcfAttorney<
          T>::GetContactByPenaltyMethodParameters(plant_);
  const double default_dissipation = contact_params.dissipation;
  const double default_stiffness = contact_params.geometry_stiffness;

  const std::vector<geometry::GeometryId> geometries =
      inspector.GetAllGeometryIds(geometry::Role::kProximity);

  for (geometry::GeometryId id : geometries) {
    GeometryDetails details;
    details.stiffness =
        GetPointContactStiffness(id, default_stiffness, inspector);
    details.friction = GetCoulombFriction(id, inspector);
    details.dissipation =
        GetHuntCrossleyDissipation(id, default_dissipation, inspector);
    geometry_details_[id] = details;
  }
}

template <typename T>
IcfBuilder<T>::BodiesSortedByAnchorage::BodiesSortedByAnchorage(
    const MultibodyPlant<T>& plant, const RigidBody<T>* bodyM,
    const RigidBody<T>* bodyN) {
  const bool M_anchored = plant.IsAnchored(*bodyM);
  const bool N_anchored = plant.IsAnchored(*bodyN);
  // Sanity check: at most one body is anchored.
  DRAKE_DEMAND(!(M_anchored && N_anchored));

  // By convention, body B is always not-anchored.
  if (N_anchored) {
    // Flip the order.
    bodyA = bodyN;
    bodyB = bodyM;
  } else {
    bodyA = bodyM;
    bodyB = bodyN;
  }
}

template <typename T>
IcfBuilder<T>::Scratch::Scratch(const MultibodyPlant<T>& plant)
    : effort_limits(VectorX<T>::Zero(plant.num_velocities())),
      accelerations(VectorX<T>::Zero(plant.num_velocities())),
      forces(plant) {
  J_V_WB.resize(6, plant.num_velocities());
}

template <typename T>
IcfBuilder<T>::PlantFacts::PlantFacts(const MultibodyPlant<T>& plant) {
  using std::isinf;

  // Define problem cliques based on the spanning forest of the plant. Each
  // tree gets its own clique, and only trees with a non-zero number of
  // velocities are included.
  const LinkJointGraph& graph = GetInternalTree(plant).graph();
  DRAKE_DEMAND(graph.forest_is_valid());
  const SpanningForest& forest = graph.forest();
  tree_to_clique.resize(forest.num_trees());
  std::fill(tree_to_clique.begin(), tree_to_clique.end(), -1);
  for (TreeIndex t(0); t < forest.num_trees(); ++t) {
    const int tree_nv = forest.trees(t).nv();
    if (tree_nv > 0) {
      tree_to_clique[t] = clique_sizes.size();
      clique_sizes.push_back(tree_nv);
    }
  }

  // Iterate over bodies to find the size of each Jacobian, clique membership,
  // floating status (floating bodies have an identity Jacobian), body mass.
  body_jacobian_cols.resize(plant.num_bodies());
  body_to_clique.resize(plant.num_bodies());
  body_is_floating.resize(plant.num_bodies());
  for (int b = 0; b < plant.num_bodies(); ++b) {
    const auto& body = plant.get_body(BodyIndex(b));
    const TreeIndex t = forest.link_to_tree_index(BodyIndex(b));

    if (plant.IsAnchored(body)) {
      body_jacobian_cols[b] = 1;  // dummy column.
      body_to_clique[b] = -1;
    } else {
      body_jacobian_cols[b] = forest.trees(t).nv();
      body_to_clique[b] = tree_to_clique[t];
    }

    // Distinguish between "free floating body" and "free floating base". The
    // former has an identity Jacobian, the latter does not.
    const auto& link = forest.link_by_index(BodyIndex(b));
    const auto& mobod = forest.mobods(link.mobod_index());
    const bool is_free_floating =
        body.is_floating_base_body() && mobod.is_leaf_mobod();
    body_is_floating[b] = is_free_floating ? 1 : 0;
  }

  // Iterate over actuators to find number of actuators per clique.
  clique_nu.assign(clique_sizes.size(), 0);
  for (JointActuatorIndex actuator_index : plant.GetJointActuatorIndices()) {
    const JointActuator<T>& actuator = plant.get_joint_actuator(actuator_index);
    const Joint<T>& joint = actuator.joint();
    const int dof = joint.velocity_start();
    const TreeIndex t = forest.v_to_tree_index(dof);
    const int c = tree_to_clique[t];
    DRAKE_ASSERT(c >= 0);
    ++clique_nu[c];
  }
  num_actuation_constraints = std::ranges::count_if(clique_nu, [](int k) {
    return k > 0;
  });

  // Iterate over joints to find cliques with at least one 1-DoF joint with
  // finite limits. Each of these cliques will require a limit constraint.
  limited_clique_sizes.clear();
  clique_to_limit_constraint.assign(clique_sizes.size(), -1);
  for (JointIndex joint_index : plant.GetJointIndices()) {
    const Joint<T>& joint = plant.get_joint(joint_index);

    // Check for 1-DoF joints.
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
    const int clique = tree_to_clique[tree_index];
    const int clique_nv = clique_sizes[clique];
    limited_clique_sizes.push_back(clique_nv);
    clique_to_limit_constraint[clique] = ssize(limited_clique_sizes) - 1;
  }
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfBuilder);
