#include "drake/multibody/plant/compliant_contact_manager.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/scope_exit.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/systems/framework/context.h"

using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
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
CompliantContactManager<T>::CompliantContactManager() {}

template <typename T>
CompliantContactManager<T>::~CompliantContactManager() {}

template <typename T>
void CompliantContactManager<T>::set_sap_solver_parameters(
    const contact_solvers::internal::SapSolverParameters& parameters) {
  sap_driver_->set_sap_solver_parameters(parameters);
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

  sap_driver_->DeclareCacheEntries(this);
}

template <typename T>
T CompliantContactManager<T>::GetPointContactStiffness(
    geometry::GeometryId id,
    const geometry::SceneGraphInspector<T>& inspector) const {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  // N.B. Here we rely on the resolution of #13289 and #5454 to get properties
  // with the proper scalar type T. This will not work on scalar converted
  // models until those issues are resolved.
  return prop->template GetPropertyOrDefault<T>(
      geometry::internal::kMaterialGroup, geometry::internal::kPointStiffness,
      this->default_contact_stiffness());
}

template <typename T>
T CompliantContactManager<T>::GetDissipationTimeConstant(
    geometry::GeometryId id,
    const geometry::SceneGraphInspector<T>& inspector) const {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);

  auto provide_context_string =
      [this, &inspector](geometry::GeometryId geometry_id) -> std::string {
    const BodyIndex body_index =
        this->geometry_id_to_body_index().at(geometry_id);
    const Body<T>& body = plant().get_body(body_index);
    return fmt::format("For geometry {} on body {}.",
                       inspector.GetName(geometry_id), body.name());
  };

  // N.B. Here we rely on the resolution of #13289 and #5454 to get properties
  // with the proper scalar type T. This will not work on scalar converted
  // models until those issues are resolved.
  const T relaxation_time = prop->template GetPropertyOrDefault<double>(
      geometry::internal::kMaterialGroup, "relaxation_time", 0.1);
  if (relaxation_time < 0.0) {
    const std::string message = fmt::format(
        "Relaxation time must be non-negative and relaxation_time "
        "= {} was provided. {}",
        relaxation_time, provide_context_string(id));
    throw std::runtime_error(message);
  }
  return relaxation_time;
}

template <typename T>
double CompliantContactManager<T>::GetCoulombFriction(
    geometry::GeometryId id,
    const geometry::SceneGraphInspector<T>& inspector) const {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  DRAKE_THROW_UNLESS(prop->HasProperty(geometry::internal::kMaterialGroup,
                                       geometry::internal::kFriction));
  return prop
      ->GetProperty<CoulombFriction<double>>(geometry::internal::kMaterialGroup,
                                             geometry::internal::kFriction)
      .dynamic_friction();
}

template <typename T>
T CompliantContactManager<T>::CombineStiffnesses(const T& k1, const T& k2) {
  // Simple utility to detect 0 / 0. As it is used in this method, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](const T& num, const T& denom) {
    return denom == 0.0 ? 0.0 : num / denom;
  };
  return safe_divide(k1 * k2, k1 + k2);
}

template <typename T>
T CompliantContactManager<T>::CombineDissipationTimeConstant(const T& tau1,
                                                             const T& tau2) {
  return tau1 + tau2;
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

  // Simple utility to detect 0 / 0. As it is used in this method, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](const T& num, const T& denom) {
    return denom == 0.0 ? T(0.0) : num / denom;
  };

  // Fill in the point contact pairs.
  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      plant().EvalPointPairPenetrations(context);
  for (const PenetrationAsPointPair<T>& pair : point_pairs) {
    const T kA = GetPointContactStiffness(pair.id_A, inspector);
    const T kB = GetPointContactStiffness(pair.id_B, inspector);
    const T k = CombineStiffnesses(kA, kB);
    const T tauA = GetDissipationTimeConstant(pair.id_A, inspector);
    const T tauB = GetDissipationTimeConstant(pair.id_B, inspector);
    const T tau = CombineDissipationTimeConstant(tauA, tauB);

    // Combine friction coefficients.
    const double muA = GetCoulombFriction(pair.id_A, inspector);
    const double muB = GetCoulombFriction(pair.id_B, inspector);
    const T mu = T(safe_divide(2.0 * muA * muB, muA + muB));

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kA + kB;
    const T wA = (denom == 0 ? 0.5 : kA / denom);
    const T wB = (denom == 0 ? 0.5 : kB / denom);
    const Vector3<T> p_WC = wA * pair.p_WCa + wB * pair.p_WCb;

    const T phi0 = -pair.depth;
    const T fn0 = NAN;  // not used.
    const T d = NAN;    // not used.
    contact_pairs.push_back(
        {pair.id_A, pair.id_B, p_WC, pair.nhat_BA_W, phi0, fn0, k, d, tau, mu});
  }
}

// Most of the calculation in this function should be the same as in
// MultibodyPlant<T>::CalcDiscreteContactPairs().
template <typename T>
void CompliantContactManager<T>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const systems::Context<T>& context,
        std::vector<DiscreteContactPair<T>>* result) const {
  std::vector<DiscreteContactPair<T>>& contact_pairs = *result;

  // Simple utility to detect 0 / 0. As it is used in this method, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](const T& num, const T& denom) {
    return denom == 0.0 ? 0.0 : num / denom;
  };

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
  for (const auto& s : surfaces) {
    const bool M_is_compliant = s.HasGradE_M();
    const bool N_is_compliant = s.HasGradE_N();
    DRAKE_DEMAND(M_is_compliant || N_is_compliant);

    // Combine dissipation.
    const T tau_M = GetDissipationTimeConstant(s.id_M(), inspector);
    const T tau_N = GetDissipationTimeConstant(s.id_N(), inspector);
    const T tau = CombineDissipationTimeConstant(tau_M, tau_N);

    // Combine friction coefficients.
    const double muA = GetCoulombFriction(s.id_M(), inspector);
    const double muB = GetCoulombFriction(s.id_N(), inspector);
    const T mu = T(safe_divide(2.0 * muA * muB, muA + muB));

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
        // et al. 2021], for convenience we define both pressure gradients
        // to be positive in the direction "into" the bodies. Therefore,
        // we use the minus sign for gN.
        // [Masterjohn et al., 2021] Discrete Approximation of Pressure
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
        // compliant-compliant interaction, see [Masterjohn et al., 2021].
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
        // faster than using Cartesian coordiantes, especially if the
        // TriangleSurfaceMeshFieldLinear<> does not store gradients and
        // has to solve linear equations to convert Cartesian to
        // barycentric coordinates.
        const Vector3<T> tri_centroid_barycentric(1 / 3., 1 / 3., 1 / 3.);
        // Pressure at the quadrature point.
        const T p0 = s.is_triangle()
                         ? s.tri_e_MN().Evaluate(face, tri_centroid_barycentric)
                         : s.poly_e_MN().EvaluateCartesian(face, p_WQ);

        // Effective compliance in the normal direction for the given
        // discrete patch, refer to [Masterjohn et al., 2021] for details.
        // [Masterjohn, 2021] Masterjohn J., Guoy D., Shepherd J. and Castro
        // A., 2021. Discrete Approximation of Pressure Field Contact Patches.
        // Available at https://arxiv.org/abs/2110.04157.
        const T k = Ae * g;

        // phi < 0 when in penetration.
        const T phi0 = -p0 / g;

        if (k > 0) {
          const T fn0 = NAN;  // not used.
          const T d = NAN;    // not used.
          contact_pairs.push_back(
              {s.id_M(), s.id_N(), p_WQ, nhat_W, phi0, fn0, k, d, tau, mu});
        }
      }
    }
  }
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
  switch (plant().get_discrete_contact_solver()) {
    case DiscreteContactSolver::kTamsi:
      throw std::runtime_error(
          "CompliantContactManager does not support TAMSI");
      break;
    case DiscreteContactSolver::kSap:
      sap_driver_->CalcContactSolverResults(context, contact_results);
      break;
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

  // Retrieve the solution velocity for the next time step.
  const VectorX<T>& v_next = results.v_next;

  // Update generalized positions.
  VectorX<T> qdot_next(plant().num_positions());
  plant().MapVelocityToQDot(context, v_next, &qdot_next);
  const VectorX<T> q_next = q0 + plant().time_step() * qdot_next;

  VectorX<T> x_next(plant().num_multibody_states());
  x_next << q_next, v_next;
  updates->set_value(this->multibody_state_index(), x_next);
}

// TODO(xuchenhan-tri): Consider a scalar converting constructor to cut down
// repeated code in CloneToDouble() and CloneToAutoDiffXd().
template <typename T>
std::unique_ptr<DiscreteUpdateManager<double>>
CompliantContactManager<T>::CloneToDouble() const {
  auto clone = std::make_unique<CompliantContactManager<double>>();
  // N.B. we should copy/clone all members except for those overwritten in
  // ExtractModelInfo and DeclareCacheEntries.
  return clone;
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>>
CompliantContactManager<T>::CloneToAutoDiffXd() const {
  auto clone = std::make_unique<CompliantContactManager<AutoDiffXd>>();
  // N.B. we should copy/clone all members except for those overwritten in
  // ExtractModelInfo and DeclareCacheEntries.
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

  switch (plant().get_discrete_contact_solver()) {
    case DiscreteContactSolver::kTamsi:
      throw std::runtime_error(
          "CompliantContactManager dos not support the TAMSI solver.");
      break;
    case DiscreteContactSolver::kSap:
      sap_driver_ = std::make_unique<SapDriver<T>>(this);
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

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
