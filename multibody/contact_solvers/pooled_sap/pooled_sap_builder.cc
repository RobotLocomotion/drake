#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_builder.h"

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
using drake::multibody::internal::MultibodyTreeTopology;
using drake::multibody::internal::SpanningForest;
using drake::multibody::internal::TreeIndex;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

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
void PooledSapBuilder<T>::CalcGeometryContactData(
    const systems::Context<T>& context) const {
  auto& surfaces = scratch_.surfaces;
  auto& point_pairs = scratch_.point_pairs;

  surfaces.clear();
  point_pairs.clear();

  auto& query_object = plant()
                           .get_geometry_query_input_port()
                           .template Eval<geometry::QueryObject<T>>(context);

  switch (plant().get_contact_model()) {
    case ContactModel::kPoint: {
      point_pairs = query_object.ComputePointPairPenetration();
      break;
    }
    case ContactModel::kHydroelastic: {
      surfaces = query_object.ComputeContactSurfaces(
          plant().get_contact_surface_representation());
      break;
    }
    case ContactModel::kHydroelasticWithFallback: {
      query_object.ComputeContactSurfacesWithFallback(
          plant().get_contact_surface_representation(), &surfaces,
          &point_pairs);
      break;
    }
  }
}

template <typename T>
PooledSapBuilder<T>::PooledSapBuilder(const MultibodyPlant<T>& plant,
                                      const systems::Context<T>& context)
    : PooledSapBuilder(plant) {
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
PooledSapBuilder<T>::PooledSapBuilder(const MultibodyPlant<T>& plant)
    : plant_(&plant) {
  const int nv = plant.num_velocities();
  scratch_.M.resize(nv, nv);
  scratch_.J_V_WB.resize(6, nv);
  scratch_.u_no_pd.resize(nv);
  scratch_.u_w_pd.resize(nv);
  scratch_.tmp_v1.resize(nv);
  scratch_.forces = std::make_unique<MultibodyForces<T>>(plant);
}

template <typename T>
void PooledSapBuilder<T>::UpdateModel(const systems::Context<T>& context,
                                      const T& time_step,
                                      bool reuse_geometry_data,
                                      PooledSapModel<T>* model) const {
  const MultibodyTreeTopology& topology =
      GetInternalTree(plant()).get_topology();
  const LinkJointGraph& graph = GetInternalTree(plant()).graph();
  DRAKE_DEMAND(graph.forest_is_valid());
  const SpanningForest& forest = graph.forest();
  (void)forest;

  const int nv = plant().num_velocities();
  MatrixX<T>& M = scratch_.M;
  Matrix6X<T>& J_V_WB = scratch_.J_V_WB;

  std::unique_ptr<PooledSapParameters<T>> params = model->ReleaseParameters();
  params->time_step = time_step;
  params->v0 = plant().GetVelocities(context);
  const auto& v0 = params->v0;
  params->A.Clear();

  // Dense linearized dynamics from MbP.
  plant().CalcMassMatrix(context, &M);

  // Implicit damping.
  // N.B. The mass matrix already includes reflected inertia terms.
  M.diagonal() += plant().EvalJointDampingCache(context) * time_step;

  // We only add a clique for tree's with non-zero number of velocities.
  int num_cliques = 0;
  std::vector<int>& tree_clique = params->tree_to_clique;
  tree_clique.resize(topology.num_trees());
  std::fill(tree_clique.begin(), tree_clique.end(), -1);
  for (TreeIndex t(0); t < topology.num_trees(); ++t) {
    const int tree_start_in_v = topology.tree_velocities_start_in_v(t);
    const int tree_nv = topology.num_tree_velocities(t);
    if (tree_nv > 0) {
      tree_clique[t] = num_cliques;
      ++num_cliques;
      typename EigenPool<MatrixX<T>>::ElementView At =
          params->A.Add(tree_nv, tree_nv);
      At = M.block(tree_start_in_v, tree_start_in_v, tree_nv, tree_nv);
    }
  }

  // Update rigid body cliques and body Jacobians.
  const auto& world_frame = plant().world_frame();
  params->D = M.diagonal().cwiseSqrt().cwiseInverse();

  // TODO(vincekurtz): consider reusing mass matrix as well. Note that
  // A = M + hD includes the time step, so M would need to be stored separately.
  if (!reuse_geometry_data) {
    params->J_WB.Clear();
    params->body_cliques.clear();
    params->body_cliques.reserve(plant().num_bodies());
    params->body_mass.clear();
    params->body_mass.reserve(plant().num_bodies());
    params->body_is_floating.clear();
    params->body_is_floating.reserve(plant().num_bodies());
    for (int b = 0; b < plant().num_bodies(); ++b) {
      const auto& body = plant().get_body(BodyIndex(b));

      // Distinguish between "free floating body" and "free floating base". The
      // former has an identity Jacobian, the latter does not.
      const auto& link = forest.link_by_index(BodyIndex(b));
      const auto& mobod = forest.mobods(link.mobod_index());
      const bool is_free_floating = body.is_floating() && mobod.is_leaf_mobod();
      params->body_is_floating.push_back(is_free_floating ? 1 : 0);

      // TODO(amcastro-tri): consider using forest.link_composites() in
      // combination with LinkJointGraph::Link::composite() to precompute
      // composite's masses.

      // If the body has zero mass, we assign it the mass of its composite.
      if (body.default_mass() == 0.0) {
        const auto composite = graph.GetLinksWeldedTo(body.index());
        T composite_mass = 0.0;
        for (BodyIndex c : composite) {
          composite_mass += plant().get_body(c).default_mass();
        }
        if (composite_mass == 0.0) {
          throw std::logic_error(fmt::format(
              "Composite for body '{}' has zero mass.", body.name()));
        }
        params->body_mass.push_back(composite_mass);
      } else {
        params->body_mass.push_back(body.default_mass());
      }

      if (plant().IsAnchored(body)) {
        params->body_cliques.push_back(-1);  // mark as anchored.
        // Empty Jacobian.
        // N.B. Eigen does not like 0-sized matrices. Thus we push a dummy
        // one-column Jacobian.
        params->J_WB.Add(6, 1);
      } else {
        const TreeIndex t = topology.body_to_tree_index(BodyIndex(b));
        const int clique = tree_clique[t];
        DRAKE_ASSERT(clique >= 0);
        const bool tree_has_dofs = topology.tree_has_dofs(t);
        DRAKE_ASSERT(tree_has_dofs);

        const int vt_start = topology.tree_velocities_start_in_v(t);
        const int nt = topology.num_tree_velocities(t);

        params->body_cliques.push_back(clique);
        typename EigenPool<Matrix6X<T>>::ElementView Jv_WBc_W =
            params->J_WB.Add(6, nt);
        if (body.is_floating()) {
          Jv_WBc_W.setIdentity();
        } else {
          plant().CalcJacobianSpatialVelocity(
              context, JacobianWrtVariable::kV, body.body_frame(),
              Vector3<T>::Zero(), world_frame, world_frame, &J_V_WB);
          Jv_WBc_W = J_V_WB.middleCols(vt_start, nt);
        }
      }
    }
  }

  // r = dt⋅(u₀ + τᵉˣ - C(q₀,v₀)) + A⋅v₀
  const T& dt = params->time_step;
  auto& r = params->r;
  r.resize(nv);
  MultibodyForces<T>& forces = *scratch_.forces;
  VectorX<T>& vdot = scratch_.tmp_v1;
  vdot = -v0 / dt;
  plant().CalcForceElementsContribution(context, &forces);

  // TODO(vincekurtz): use a CalcInverseDynamics signature that doesn't allocate
  // a return value.
  r = -dt * plant().CalcInverseDynamics(context, vdot, forces);
  r += dt * plant().EvalJointDampingCache(context).asDiagonal() * v0;

  // Collect effort limits for each clique.
  params->clique_nu.assign(num_cliques, 0);
  params->effort_limits.resize(nv);
  params->effort_limits.setZero();

  for (JointActuatorIndex actuator_index : plant().GetJointActuatorIndices()) {
    const JointActuator<T>& actuator =
        plant().get_joint_actuator(actuator_index);
    const Joint<T>& joint = actuator.joint();
    const int dof = joint.velocity_start();
    const TreeIndex t = topology.velocity_to_tree_index(dof);
    const int c = tree_clique[t];
    DRAKE_ASSERT(c >= 0);
    ++params->clique_nu[c];
    params->effort_limits[dof] = actuator.effort_limit();
  }

  model->ResetParameters(std::move(params));
  model->set_stiction_tolerance(plant().stiction_tolerance());

  // Add contact constraints. We'll need to clear the patch constraints pool
  // here to avoid conflicting hydro and point contact constraints.
  model->patch_constraints_pool().Clear();
  if (!reuse_geometry_data) {
    CalcGeometryContactData(context);
  }
  AddPatchConstraintsForHydroelasticContact(context, model);
  AddPatchConstraintsForPointContact(context, model);

  // Add other constraints to the problem
  AddCouplerConstraints(context, model);
  AddLimitConstraints(context, model);

  model->SetSparsityPattern();
}

template <typename T>
void PooledSapBuilder<T>::AccumulateForceElementForces(
    const systems::Context<T>& context, VectorX<T>* r) const {
  MultibodyForces<T>& forces = *scratch_.forces;
  VectorX<T>& tau_g = scratch_.tmp_v1;
  plant().CalcForceElementsContribution(context, &forces);
  plant().AddAppliedExternalSpatialForces(context, &forces);
  plant().AddAppliedExternalGeneralizedForces(context, &forces);
  plant().CalcGeneralizedForces(context, forces, &tau_g);
  *r += tau_g;
}

template <typename T>
void PooledSapBuilder<T>::CalcActuationInput(
    const systems::Context<T>& context, VectorX<T>* actuation_w_pd,
    VectorX<T>* actuation_wo_pd) const {
  DRAKE_DEMAND(actuation_w_pd != nullptr);
  DRAKE_DEMAND(actuation_w_pd->size() == plant().num_velocities());
  DRAKE_DEMAND(actuation_wo_pd != nullptr);
  DRAKE_DEMAND(actuation_wo_pd->size() == plant().num_velocities());
  actuation_w_pd->setZero();
  actuation_wo_pd->setZero();
  if (plant().num_actuators() > 0) {
    const VectorX<T> u = plant().AssembleActuationInput(context);

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
      actuation[v_index] += u[actuator.input_start()];
    }
  }
}

template <typename T>
void PooledSapBuilder<T>::AddCouplerConstraints(
    const systems::Context<T>& context, PooledSapModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);

  const MultibodyTreeTopology& topology =
      GetInternalTree(plant()).get_topology();
  const std::vector<int>& tree_to_clique = model->params().tree_to_clique;
  const std::map<MultibodyConstraintId, CouplerConstraintSpec>& specs_map =
      plant().get_coupler_constraint_specs();

  typename PooledSapModel<T>::CouplerConstraintsPool& couplers =
      model->coupler_constraints_pool();
  couplers.Reset();

  for (const auto& [id, spec] : specs_map) {
    const Joint<T>& joint0 = plant().get_joint(spec.joint0_index);
    const Joint<T>& joint1 = plant().get_joint(spec.joint1_index);

    const int dof0 = joint0.velocity_start();
    const int dof1 = joint1.velocity_start();
    const TreeIndex tree0 = topology.velocity_to_tree_index(dof0);
    const TreeIndex tree1 = topology.velocity_to_tree_index(dof1);

    // Sanity check.
    DRAKE_DEMAND(tree0.is_valid() && tree1.is_valid());

    const int clique0 = tree_to_clique[tree0];
    const int clique1 = tree_to_clique[tree1];

    if (clique0 != clique1) {
      throw std::logic_error(
          "PooledSapBuilder: Couplers are only allowed within DoFs in the same "
          "tree.");
    }

    const T q0 = joint0.GetOnePosition(context);
    const T q1 = joint1.GetOnePosition(context);

    // DOFs local to their tree.
    const int tree_dof0 = dof0 - topology.tree_velocities_start_in_v(tree0);
    const int tree_dof1 = dof1 - topology.tree_velocities_start_in_v(tree1);

    couplers.Add(clique0, tree_dof0, tree_dof1, q0, q1, spec.gear_ratio,
                 spec.offset);
  }
}

template <typename T>
void PooledSapBuilder<T>::AddLimitConstraints(
    const systems::Context<T>& context, PooledSapModel<T>* model) const {
  DRAKE_ASSERT(model != nullptr);
  using std::isinf;

  const MultibodyTreeTopology& topology =
      GetInternalTree(plant()).get_topology();

  typename PooledSapModel<T>::LimitConstraintsPool& limits =
      model->limit_constraints_pool();

  limits.Reset();

  const std::vector<int>& tree_to_clique = model->params().tree_to_clique;

  for (JointIndex joint_index : plant().GetJointIndices()) {
    const Joint<T>& joint = plant().get_joint(joint_index);
    // We only support limits for 1 DOF joints for which we know that q̇ = v.
    if (joint.num_positions() == 1 && joint.num_velocities() == 1) {
      const int velocity_start = joint.velocity_start();
      const TreeIndex tree_index =
          topology.velocity_to_tree_index(velocity_start);
      const int tree_nv = topology.num_tree_velocities(tree_index);
      DRAKE_ASSERT(tree_nv >= 0);
      const int tree_velocity_start =
          topology.tree_velocities_start_in_v(tree_index);
      const int tree_dof = velocity_start - tree_velocity_start;
      const int clique = tree_to_clique[tree_index];

      const double ql = joint.position_lower_limits()[0];
      const double qu = joint.position_upper_limits()[0];
      const T& q0 = joint.GetOnePosition(context);

      if (!isinf(ql) || !isinf(qu)) {
        // Add constraint for this dof in clique.
        limits.Add(clique, tree_dof, q0, ql, qu);
      }
    }
  }
}

template <typename T>
void PooledSapBuilder<T>::AddPatchConstraintsForPointContact(
    const systems::Context<T>& context, PooledSapModel<T>* model) const {
  const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs =
      scratch_.point_pairs;
  const int num_point_contacts = point_pairs.size();
  if (num_point_contacts == 0) {
    return;
  }

  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);

  typename PooledSapModel<T>::PatchConstraintsPool& patches =
      model->patch_constraints_pool();

  // Fill in the point contact pairs.
  for (int point_pair_index = 0; point_pair_index < num_point_contacts;
       ++point_pair_index) {
    const geometry::PenetrationAsPointPair<T>& pp =
        point_pairs[point_pair_index];

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
    patches.AddPatch(bodyA->index(), bodyB->index(), d, mu.static_friction(),
                     mu.dynamic_friction(), p_AB_W);
    patches.AddPair(p_BoC_W, nhat_AB_W, fn0, k);
  }
}

template <typename T>
void PooledSapBuilder<T>::AddPatchConstraintsForHydroelasticContact(
    const systems::Context<T>& context, PooledSapModel<T>* model) const {
  // Add contact constraints for hydro.
  const std::vector<geometry::ContactSurface<T>>& surfaces = scratch_.surfaces;

  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);

  // TODO(amcastro-tri): This should be retrieved from the default contact
  // properties.
  const double kDefaultDissipation = 50.0;

  // Pre-allocate as needed. No allocation if size is not exceeded.
  int num_pairs = 0;
  for (const auto& s : surfaces) {
    num_pairs += s.num_faces();
  }
  const int num_surfaces = surfaces.size();
  (void)num_pairs;

  typename PooledSapModel<T>::PatchConstraintsPool& patches =
      model->patch_constraints_pool();

  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    const auto& s = surfaces[surface_index];
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

    patches.AddPatch(bodyA->index(), bodyB->index(), d, mu.static_friction(),
                     mu.dynamic_friction(), p_AB_W);

    for (int face = 0; face < s.num_faces(); ++face) {
      const T Ae = s.area(face);  // Face element area.
      const Vector3<T>& nhat_NM_W = s.face_normal(face);
      const T gM = M_is_compliant ? s.EvaluateGradE_M_W(face).dot(nhat_NM_W)
                                  : std::numeric_limits<double>::infinity();
      const T gN = N_is_compliant ? -s.EvaluateGradE_N_W(face).dot(nhat_NM_W)
                                  : std::numeric_limits<double>::infinity();
      constexpr double kGradientEpsilon = 1.0e-14;
      if (gM < kGradientEpsilon || gN < kGradientEpsilon) {
        continue;
      }
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
      const T k = Ae * g;
      patches.AddPair(p_BoC_W, nhat_AB_W, fn0, k);
    }
  }
}

template <typename T>
void PooledSapBuilder<T>::AddActuationGains(const VectorX<T>& Ku,
                                            const VectorX<T>& bu,
                                            PooledSapModel<T>* model) const {
  DRAKE_DEMAND(model != nullptr);
  DRAKE_DEMAND(model->num_velocities() == plant().num_velocities());
  const int nv = model->num_velocities();
  DRAKE_DEMAND(Ku.size() == nv);
  DRAKE_DEMAND(bu.size() == nv);

  // Quick no-op exit.
  if (plant().num_actuators() == 0) return;

  auto& gain_constraints = model->gain_constraints_pool();

  for (int c = 0; c < model->num_cliques(); ++c) {
    if (model->params().clique_nu[c] == 0) continue;

    const auto Ku_c = model->clique_segment(c, Ku);
    const auto bu_c = model->clique_segment(c, bu);
    const auto e_c = model->clique_segment(c, model->params().effort_limits);

    gain_constraints.Add(c, Ku_c, bu_c, e_c);
  }
}

template <typename T>
void PooledSapBuilder<T>::AddExternalGains(const VectorX<T>& Ke,
                                           const VectorX<T>& be,
                                           PooledSapModel<T>* model) const {
  DRAKE_DEMAND(model != nullptr);
  const int nv = model->num_velocities();
  DRAKE_DEMAND(Ke.size() == nv);
  DRAKE_DEMAND(be.size() == nv);

  const T& dt = model->time_step();
  PooledSapParameters<T>& params = model->params();
  for (int c = 0; c < model->num_cliques(); ++c) {
    auto A_c = params.A[c];
    const auto Ke_c = model->clique_segment(c, Ke);
    A_c.diagonal() += dt * Ke_c;

    const auto be_c = model->clique_segment(c, be);
    auto r_c = model->clique_segment(c, &params.r);
    r_c += dt * be_c;
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapBuilder);
