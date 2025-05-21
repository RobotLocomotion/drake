#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_builder.h"

#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/plant/contact_properties.h"

using drake::multibody::internal::GetCombinedDynamicCoulombFriction;
using drake::multibody::internal::GetCombinedHuntCrossleyDissipation;
using drake::multibody::internal::GetCombinedPointContactStiffness;
using drake::multibody::internal::GetPointContactStiffness;

namespace drake {
namespace multibody {

// Temporary hack to gain access to MultibodyPlant's private functions.
class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = default;
  template <typename T>
  static VectorX<T> AssembleActuationInput(const MultibodyPlant<T>& plant,
                                           const systems::Context<T>& context) {
    return plant.AssembleActuationInput(context);
  }
};

namespace contact_solvers {
namespace pooled_sap {

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
PooledSapBuilder<T>::PooledSapBuilder(const MultibodyPlant<T>& plant)
    : plant_(&plant) {
  const int nv = plant.num_velocities();
  scratch_.u_no_pd.resize(nv);
  scratch_.u_w_pd.resize(nv);
  scratch_.tmp_v1.resize(nv);
  scratch_.forces = std::make_unique<MultibodyForces<T>>(plant);
}

template <typename T>
void PooledSapBuilder<T>::UpdateModel(const systems::Context<T>& context,
                                      const T& time_step,
                                      PooledSapModel<T>* model) const {
  // N.B. we can retrieve spanning forest (tree) like so:
  // const SpanningForest& tree = internal::GetInternalTree(plant).forest();

  const int nv = plant().num_velocities();
  VectorX<T>& u_no_pd = scratch_.u_no_pd;
  VectorX<T>& u_w_pd = scratch_.u_w_pd;

  std::unique_ptr<PooledSapParameters<T>> params = model->ReleaseParameters();
  params->time_step = time_step;
  params->v0 = plant().GetVelocities(context);
  const auto& v0 = params->v0;
  params->A.Clear();
  params->J_WB.Clear();

  // Linearized dynamics matrix for a single clique.
  typename EigenPool<MatrixX<T>>::ElementView M = params->A.Add(nv, nv);
  plant().CalcMassMatrix(context, &M);

  // TODO(amcastro-tri): Add effective damping terms.
  // const VectorX<T> diagonal_inertia = plant().CalcEffectiveDamping(context);

  // Update rigid body cliques and body Jacobians.
  const auto& world_frame = plant().world_frame();
  params->body_cliques.clear();
  params->body_cliques.reserve(plant().num_bodies());
  for (int b = 0; b < plant().num_bodies(); ++b) {
    const auto& body = plant().get_body(BodyIndex(b));
    if (plant().IsAnchored(body)) {
      params->body_cliques.push_back(-1);  // mark as anchored.
      // Empty Jacobian.
      // N.B. Eigen does not like 0-sized matrices. Thus we push a dummy
      // one-column Jacobian.
      params->J_WB.Add(6, 1);
    } else {
      params->body_cliques.push_back(0);  // Single clique (dense) setup.
      typename EigenPool<Matrix6X<T>>::ElementView Jv_WBc_W =
          params->J_WB.Add(6, nv);
      plant().CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kV,
                                          body.body_frame(), Vector3<T>::Zero(),
                                          world_frame, world_frame, &Jv_WBc_W);
    }
  }

  // r = u₀ + M⋅v₀ - C(q₀,v₀)
  auto& r = params->r;
  r.resize(nv);
  CalcActuationInput(context, &u_w_pd, &u_no_pd);
  plant().CalcBiasTerm(context, &r);
  r = -r;                                     // r = -C(q₀, v₀)
  r += M * v0;                                // r += M⋅v₀
  r += u_no_pd;                               // r += u.
  AccumulateForceElementForces(context, &r);  // r+= τᵉˣᵗ

  model->ResetParameters(std::move(params));
  CalcGeometryContactData(context);
  AddPatchConstraintsForHydroelasticContact(context, model);
  AddPatchConstraintsForPointContact(context, model);
}

template <typename T>
void PooledSapBuilder<T>::AccumulateForceElementForces(
    const systems::Context<T>& context, VectorX<T>* r) const {
  MultibodyForces<T>& forces = *scratch_.forces;
  VectorX<T>& tau_g = scratch_.tmp_v1;
  plant().CalcForceElementsContribution(context, &forces);
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
    const VectorX<T> u =
        MultibodyPlantTester::AssembleActuationInput(plant(), context);

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

  // TODO(amcastro-tri): This should be retrieved from the default contact
  // properties.
  const double kDefaultDissipation = 50.0;
  const double kDefaultStiffness = 1.0e6;

  typename PooledSapModel<T>::PatchConstraintsPool& patches =
      model->patch_constraints_pool();
  patches.Clear();

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
    const T kM = GetPointContactStiffness(Mid, kDefaultStiffness, inspector);
    const T kN = GetPointContactStiffness(Nid, kDefaultStiffness, inspector);
    const T k = GetCombinedPointContactStiffness(Mid, Nid, kDefaultStiffness,
                                                 inspector);
    const T d = GetCombinedHuntCrossleyDissipation(
        Mid, Nid, kM, kN, kDefaultDissipation, inspector);
    const T mu = GetCombinedDynamicCoulombFriction(Mid, Nid, inspector);

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kM + kN;
    const T wM = (denom == 0 ? 0.5 : kM / denom);
    const T wN = (denom == 0 ? 0.5 : kN / denom);
    const Vector3<T> p_WC = wM * pp.p_WCa + wN * pp.p_WCb;
    const Vector3<T> p_BoC_W = p_WC - p_WBo;

    // Normal must always point from A to B, by convention.
    const Vector3<T>& nhat_MN_W = pp.nhat_BA_W;  // Points from M into N.
    const Vector3<T> nhat_AB_W = bodyB == bodyN ? nhat_MN_W : -nhat_MN_W;
    const T fn0 = k * pp.depth;

    // For point contact we add single-pair patches.
    patches.AddPatch(bodyA->index(), bodyB->index(), d, mu, p_AB_W);
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
  patches.Clear();

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
    const T mu = multibody::internal::GetCombinedDynamicCoulombFriction(
        s.id_M(), s.id_N(), inspector);

    const auto& X_WA = bodyA->EvalPoseInWorld(context);
    const auto& X_WB = bodyB->EvalPoseInWorld(context);
    const Vector3<T>& p_WAo = X_WA.translation();
    const Vector3<T>& p_WBo = X_WB.translation();
    const Vector3<T> p_AB_W = p_WBo - p_WAo;

    patches.AddPatch(bodyA->index(), bodyB->index(), d, mu, p_AB_W);

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

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapBuilder);
