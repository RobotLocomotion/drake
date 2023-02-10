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
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/tamsi_driver.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/systems/framework/context.h"

using drake::geometry::ContactSurface;
using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::math::RotationMatrix;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::internal::DiscreteContactPair;
using drake::multibody::internal::MultibodyTreeTopology;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
AccelerationsDueToExternalForcesCache<T>::AccelerationsDueToExternalForcesCache(
    const MultibodyTreeTopology& topology)
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
void CompliantContactManager<T>::DeclareCacheEntries() {
  // N.B. We use xd_ticket() instead of q_ticket() since discrete
  // multibody plant does not have q's, but rather discrete state.
  // Therefore if we make it dependent on q_ticket() the Jacobian only
  // gets evaluated once at the start of the simulation.

  // Cache discrete contact pairs.
  const auto& discrete_contact_pairs_cache_entry = this->DeclareCacheEntry(
      "Discrete contact pairs.",
      systems::ValueProducer(
          this, &CompliantContactManager<T>::CalcDiscreteContactPairs),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.discrete_contact_pairs =
      discrete_contact_pairs_cache_entry.cache_index();

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

  // Cache contact kinematics.
  const auto& contact_kinematics_cache_entry = this->DeclareCacheEntry(
      "Contact kinematics.",
      systems::ValueProducer(
          this, &CompliantContactManager::CalcContactKinematics),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_kinematics =
      contact_kinematics_cache_entry.cache_index();

  // Accelerations due to non-contact forces.
  // We cache non-contact forces, ABA forces and accelerations into a
  // AccelerationsDueToExternalForcesCache.
  AccelerationsDueToExternalForcesCache<T> non_contact_forces_accelerations(
      this->internal_tree().get_topology());
  const auto& non_contact_forces_accelerations_cache_entry =
      this->DeclareCacheEntry(
          "Non-contact forces accelerations.",
          systems::ValueProducer(
              this, non_contact_forces_accelerations,
              &CompliantContactManager<
                  T>::CalcAccelerationsDueToNonContactForcesCache),
          // Due to issue #12786, we cannot properly mark this entry dependent
          // on inputs. CalcAccelerationsDueToNonContactForcesCache() uses
          // CacheIndexes::non_contact_forces_evaluation_in_progress to guard
          // against algebraic loops.
          {systems::System<T>::xd_ticket(),
           systems::System<T>::all_parameters_ticket()});
  cache_indexes_.non_contact_forces_accelerations =
      non_contact_forces_accelerations_cache_entry.cache_index();

  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->DeclareCacheEntries(this);
    }
  }

  // Discrete updates with SAP are not supported when T = symbolic::Expression.
  if constexpr (!std::is_same_v<T, symbolic::Expression>) {
    if (sap_driver_ != nullptr) sap_driver_->DeclareCacheEntries(this);
  }
}

template <typename T>
std::vector<ContactPairKinematics<T>>
CompliantContactManager<T>::CalcContactKinematics(
    const systems::Context<T>& context) const {
  const std::vector<DiscreteContactPair<T>>& contact_pairs =
      this->EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();
  std::vector<ContactPairKinematics<T>> contact_kinematics;
  contact_kinematics.reserve(num_contacts);

  // Quick no-op exit.
  if (num_contacts == 0) return contact_kinematics;

  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();
  // Scratch workspace variables.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);

  const Frame<T>& frame_W = plant().world_frame();
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = contact_pairs[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    // All contact pairs involving deformable bodies come after pairs involving
    // only rigid bodies. Once we reach a deformable body, we know that there
    // are no rigid-only pairs left.
    if (inspector.IsDeformableGeometry(geometryA_id)) {
      break;
    }
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = this->geometry_id_to_body_index().at(geometryA_id);
    const Body<T>& bodyA = plant().get_body(bodyA_index);
    BodyIndex bodyB_index = this->geometry_id_to_body_index().at(geometryB_id);
    const Body<T>& bodyB = plant().get_body(bodyB_index);

    // Contact normal from point A into B.
    const Vector3<T>& nhat_W = -point_pair.nhat_BA_W;
    const Vector3<T>& p_WC = point_pair.p_WC;

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
      jacobian_blocks.emplace_back(treeA_index, std::move(J));
    }

    // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
    // This contribution must be added only if B is different from A.
    if ((treeB_index.is_valid() && !treeA_index.is_valid()) ||
        (treeB_index.is_valid() && treeB_index != treeA_index)) {
      Matrix3X<T> J = R_WC.matrix().transpose() *
                      Jv_AcBc_W.middleCols(
                          tree_topology().tree_velocities_start(treeB_index),
                          tree_topology().num_tree_velocities(treeB_index));
      jacobian_blocks.emplace_back(treeB_index, std::move(J));
    }

    contact_kinematics.emplace_back(point_pair.phi0, std::move(jacobian_blocks),
                                    std::move(R_WC));
  }
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->AppendContactKinematics(context, &contact_kinematics);
    }
  }

  return contact_kinematics;
}

template <typename T>
const std::vector<ContactPairKinematics<T>>&
CompliantContactManager<T>::EvalContactKinematics(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_kinematics)
      .template Eval<std::vector<ContactPairKinematics<T>>>(context);
}

template <>
void CompliantContactManager<symbolic::Expression>::CalcDiscreteContactPairs(
    const drake::systems::Context<symbolic::Expression>&,
    std::vector<DiscreteContactPair<symbolic::Expression>>*) const {
  // Currently, the computation of contact pairs is not supported when T =
  // symbolic::Expression.
  throw std::domain_error(
      fmt::format("This method doesn't support T = {}.",
                  NiceTypeName::Get<symbolic::Expression>()));
}

template <typename T>
void CompliantContactManager<T>::CalcDiscreteContactPairs(
    const systems::Context<T>& context,
    std::vector<DiscreteContactPair<T>>* contact_pairs) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(contact_pairs != nullptr);

  contact_pairs->clear();
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
  const int num_contact_pairs = num_point_pairs + num_quadrature_pairs;
  contact_pairs->reserve(num_contact_pairs);

  if (contact_model == ContactModel::kPoint ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    AppendDiscreteContactPairsForPointContact(context, contact_pairs);
  }
  if (contact_model == ContactModel::kHydroelastic ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    AppendDiscreteContactPairsForHydroelasticContact(context, contact_pairs);
  }
  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->AppendDiscreteContactPairs(context, contact_pairs);
    }
  }
}

template <typename T>
void CompliantContactManager<T>::AppendDiscreteContactPairsForPointContact(
    const systems::Context<T>& context,
    std::vector<DiscreteContactPair<T>>* result) const {
  std::vector<DiscreteContactPair<T>>& contact_pairs = *result;

  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

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
        pair.id_A, pair.id_B, default_dissipation_time_constant, body_A.name(),
        body_B.name(), inspector);
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

    contact_pairs.push_back({pair.id_A, pair.id_B, p_WC, pair.nhat_BA_W, phi0,
                             fn0, k, d, tau, mu, {} /* no surface index */,
                             {} /* no face index */});
  }
}

template <>
void CompliantContactManager<symbolic::Expression>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const drake::systems::Context<symbolic::Expression>&,
        std::vector<DiscreteContactPair<symbolic::Expression>>*) const {
  throw std::domain_error(
      fmt::format("This method doesn't support T = {}.",
                  NiceTypeName::Get<symbolic::Expression>()));
}

template <typename T>
void CompliantContactManager<T>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const systems::Context<T>& context,
        std::vector<DiscreteContactPair<T>>* result) const {
  std::vector<DiscreteContactPair<T>>& contact_pairs = *result;

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
      // Undamped normal contact force by this polygonal/triangle face
      T fn0;
      // Effective stiffness of the contact pair.
      T stiffness;
      // Surrogate signed distance. It's negative when in contact.
      T phi0;
      if (DiscreteHydroelasticToPointContact(s, face, &fn0, &stiffness,
                                             &phi0)) {
        contact_pairs.push_back({s.id_M(), s.id_N(), s.centroid(face),
                                 s.face_normal(face), phi0, fn0, stiffness,
                                 d, tau, mu, surface_index, face});
      }
    }
  }
}

template <typename T>
bool CompliantContactManager<T>::DiscreteHydroelasticToPointContact(
    const geometry::ContactSurface<T>& contact_patch, const int face,
    T* normal_force, T* stiffness, T* surrogate_signed_distance) {
  const T& Ae = contact_patch.area(face);  // Face element area.

  // We found out that the hydroelastic query might report
  // infinitesimally small triangles (consider for instance an initial
  // condition that perfectly places an object at zero distance from the
  // ground.) While the area of zero sized triangles is not a problem by
  // itself, the badly computed normal on these triangles leads to
  // problems when computing the contact Jacobians (since we need to
  // obtain an orthonormal basis based on that normal.)
  // We therefore ignore infinitesimally small triangles. The tolerance
  // below is somehow arbitrary and could possibly be tightened.
  if (Ae <= 1.0e-14) {
    return false;
  }

  const bool M_is_compliant = contact_patch.HasGradE_M();
  const bool N_is_compliant = contact_patch.HasGradE_N();

  // From ContactSurface'contact_patch documentation: The normal of each face is
  // guaranteed to point "out of" N and "into" M.
  const Vector3<T>& nhat_W = contact_patch.face_normal(face);

  // One dimensional pressure gradient (in Pa/m). Unlike [Masterjohn
  // 2022], for convenience we define both pressure gradients
  // to be positive in the direction "into" the bodies. Therefore,
  // we use the minus sign for gN.
  // [Masterjohn 2022] Velocity Level Approximation of Pressure
  // Field Contact Patches.
  const T gM = M_is_compliant
                   ? contact_patch.EvaluateGradE_M_W(face).dot(nhat_W)
                   : T(std::numeric_limits<double>::infinity());
  const T gN = N_is_compliant
                   ? -contact_patch.EvaluateGradE_N_W(face).dot(nhat_W)
                   : T(std::numeric_limits<double>::infinity());

  constexpr double kGradientEpsilon = 1.0e-14;
  if (gM < kGradientEpsilon || gN < kGradientEpsilon) {
    // Mathematically g = gN*gM/(gN+gM) and therefore g = 0 when
    // either gradient on one of the bodies is zero. A zero gradient
    // means there is no contact constraint, and therefore we
    // ignore it to avoid numerical problems in the discrete solver.
    return false;
  }

  // Effective hydroelastic pressure gradient g result of
  // compliant-compliant interaction, see [Masterjohn 2022].
  // The expression below is mathematically equivalent to g =
  // gN*gM/(gN+gM) but it has the advantage of also being valid if
  // one of the gradients is infinity.
  const T g = 1.0 / (1.0 / gM + 1.0 / gN);

  // Position of quadrature point Q in the world frame (since mesh_W
  // is measured and expressed in W).
  const Vector3<T>& p_WQ = contact_patch.centroid(face);
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
      contact_patch.is_triangle()
          ? contact_patch.tri_e_MN().Evaluate(face, tri_centroid_barycentric)
          : contact_patch.poly_e_MN().EvaluateCartesian(face, p_WQ);

  // Force contribution by this quadrature point.
  *normal_force = Ae * p0;

  // Effective compliance in the normal direction for the given
  // discrete patch, refer to [Masterjohn 2022] for details.
  // [Masterjohn 2022] Masterjohn J., Guoy D., Shepherd J. and
  // Castro A., 2022. Velocity Level Approximation of Pressure Field
  // Contact Patches. Available at https://arxiv.org/abs/2110.04157.
  *stiffness = Ae * g;

  // phi < 0 when in penetration.
  *surrogate_signed_distance = -p0 / g;

  if (*stiffness <= 0) {
    return false;
  }

  return true;
}

template <typename T>
void CompliantContactManager<T>::CalcNonContactForcesExcludingJointLimits(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(plant()));
  // Compute forces applied through force elements. Note that this resets
  // forces to empty so must come first.
  this->CalcForceElementsContribution(context, forces);
  this->AddInForcesFromInputPorts(context, forces);
}

template <typename T>
void CompliantContactManager<T>::CalcAccelerationsDueToNonContactForcesCache(
    const systems::Context<T>& context,
    AccelerationsDueToExternalForcesCache<T>* forward_dynamics_cache) const {
  DRAKE_DEMAND(forward_dynamics_cache != nullptr);
  ScopeExit guard = this->ThrowIfNonContactForceInProgress(context);

  // N.B. Joint limits are modeled as constraints. Therefore here we only add
  // all other external forces.
  CalcNonContactForcesExcludingJointLimits(context,
                                           &forward_dynamics_cache->forces);

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
  const VectorX<T> diagonal_inertia =
      plant().EvalReflectedInertiaCache(context) +
      joint_damping_ * plant().time_step();

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
const std::vector<DiscreteContactPair<T>>&
CompliantContactManager<T>::EvalDiscreteContactPairs(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.discrete_contact_pairs)
      .template Eval<std::vector<DiscreteContactPair<T>>>(context);
}

template <typename T>
const multibody::internal::AccelerationKinematicsCache<T>&
CompliantContactManager<T>::EvalAccelerationsDueToNonContactForcesCache(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.non_contact_forces_accelerations)
      .template Eval<AccelerationsDueToExternalForcesCache<T>>(context)
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
void CompliantContactManager<T>::DoCalcDiscreteValues(
    const drake::systems::Context<T>& context,
    drake::systems::DiscreteValues<T>* updates) const {
  const ContactSolverResults<T>& results =
      this->EvalContactSolverResults(context);

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

  if constexpr (std::is_same_v<T, double>) {
    if (deformable_driver_ != nullptr) {
      deformable_driver_->CalcDiscreteStates(context, updates);
    }
  }
}

template <typename T>
void CompliantContactManager<T>::AppendContactResultsForPointContact(
    const drake::systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      plant().EvalPointPairPenetrations(context);
  const std::vector<internal::DiscreteContactPair<T>>& discrete_pairs =
      this->EvalDiscreteContactPairs(context);
  const std::vector<ContactPairKinematics<T>>& contact_kinematics =
      this->EvalContactKinematics(context);
  const contact_solvers::internal::ContactSolverResults<T>& solver_results =
      this->EvalContactSolverResults(context);

  const VectorX<T>& fn = solver_results.fn;
  const VectorX<T>& ft = solver_results.ft;
  const VectorX<T>& vt = solver_results.vt;
  const VectorX<T>& vn = solver_results.vn;


  const int num_contacts = point_pairs.size();
  DRAKE_DEMAND(fn.size() >= num_contacts);
  DRAKE_DEMAND(ft.size() >= 2 * num_contacts);
  DRAKE_DEMAND(vn.size() >= num_contacts);
  DRAKE_DEMAND(vt.size() >= 2 * num_contacts);

  // The correspondence between `discrete_pairs` and `point_pairs` depends on
  // strict ordering of CompliantContactManager::CalcDiscreteContactPairs.
  // All point contacts must come first and their order in discrete_pairs
  // corresponds to their order in point_pairs.
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& discrete_pair = discrete_pairs[icontact];
    const auto& point_pair = point_pairs[icontact];

    const GeometryId geometryA_id = discrete_pair.id_A;
    const GeometryId geometryB_id = discrete_pair.id_B;

    const BodyIndex bodyA_index = this->FindBodyByGeometryId(geometryA_id);
    const BodyIndex bodyB_index = this->FindBodyByGeometryId(geometryB_id);

    const RotationMatrix<T>& R_WC = contact_kinematics[icontact].R_WC;

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

  const std::vector<internal::DiscreteContactPair<T>>& discrete_pairs =
      this->EvalDiscreteContactPairs(context);
  const std::vector<ContactPairKinematics<T>>& contact_kinematics =
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

  // If the point contact model is used, hydroelastic contact pairs are appended
  // after point contact pairs.
  // TODO(amcastro-tri): right now EvalPointPairPenetrations() throws an
  // exception if only hydroelastic is used. Consider a solution in which this
  // method always returns a valid result, possibly empty.
  // A possible solution could be moving this method into DiscreteUpdateManager
  // if the plant no longer uses it.
  const int num_point_contacts =
      plant().get_contact_model() == ContactModel::kHydroelastic
          ? 0
          : plant().EvalPointPairPenetrations(context).size();
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
  for (int icontact = num_point_contacts; icontact < num_contacts; ++icontact) {
    const auto& pair = discrete_pairs[icontact];
    // Quadrature point Q.
    const Vector3<T>& p_WQ = pair.p_WC;
    const RotationMatrix<T>& R_WC = contact_kinematics[icontact].R_WC;

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
    // N.B. CompliantContactManager<T>::CalcContactKinematics() uses the
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

  // Update contact info to include the correct contact forces.
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    contact_info->emplace_back(
        &all_surfaces[surface_index], F_Ao_W_per_surface[surface_index],
        std::move(quadrature_data[surface_index]));
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
void CompliantContactManager<T>::ExtractModelInfo() {
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
        sap_driver_ = std::make_unique<SapDriver<T>>(this);
      }
      break;
    case DiscreteContactSolver::kTamsi:
      // N.B. We do allow discrete updates with TAMSI when T =
      // symbolic::Expression, but only when there is no contact.
      tamsi_driver_ = std::make_unique<TamsiDriver<T>>(this);
      break;
  }

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
}

template <typename T>
void CompliantContactManager<T>::ExtractConcreteModel(
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

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
