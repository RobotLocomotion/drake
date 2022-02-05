#include "drake/multibody/plant/compliant_contact_manager.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/scope_exit.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_coupler_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_distance_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::math::RotationMatrix;
using drake::multibody::internal::MultibodyTreeTopology;
using drake::multibody::Body;
using drake::multibody::Joint;
using drake::systems::Context;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapFrictionConeConstraint;
using drake::multibody::contact_solvers::internal::SapCouplerConstraint;
using drake::multibody::contact_solvers::internal::SapDistanceConstraint;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
AccelerationsDueToExternalForcesCache<T>::AccelerationsDueToExternalForcesCache(
    const MultibodyTreeTopology& topology)
    : forces(topology.num_bodies(), topology.num_velocities()),
      aba_forces(topology),
      ac(topology) {}

template <typename T>
CompliantContactManager<T>::CompliantContactManager(
    std::unique_ptr<contact_solvers::internal::ContactSolver<T>> contact_solver)
    : contact_solver_(std::move(contact_solver)) {
  //DRAKE_DEMAND(contact_solver_ != nullptr);
}

template <typename T>
CompliantContactManager<T>::~CompliantContactManager() {}

template <typename T>
void CompliantContactManager<T>::AddCouplerConstraint(
    const Joint<T>& joint0, const Joint<T>& joint1, const T& gear_ratio,
    const T& stiffness, const T& dissipation_time_scale) {
  DRAKE_THROW_UNLESS(joint0.num_velocities() == 1);
  DRAKE_THROW_UNLESS(joint1.num_velocities() == 1);
  coupler_constraints_info_.push_back(
      CouplerConstraintInfo{joint0.velocity_start(), joint1.velocity_start(),
                            gear_ratio, stiffness, dissipation_time_scale});
}

template <typename T>
void CompliantContactManager<T>::AddDistanceConstraint(
    const Body<T>& body_A, const Vector3<T>& p_AP, const Body<T>& body_B,
    const Vector3<T>& p_BQ, const T& distance, const T& stiffness,
    const T& dissipation_time_scale) {
  distance_constraints_info_.push_back(
      DistanceConstraintInfo{body_A.index(), p_AP, body_B.index(), p_BQ,
                             distance, stiffness, dissipation_time_scale});
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

  // Contact Jacobian for the discrete pairs computed with
  // CompliantContactManager::CalcDiscreteContactPairs().
  auto& contact_jacobian_cache_entry = this->DeclareCacheEntry(
      std::string("Contact Jacobian."),
      systems::ValueProducer(
          this, &CompliantContactManager<T>::CalcContactJacobianCache),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_jacobian = contact_jacobian_cache_entry.cache_index();

  std::vector<MatrixX<T>> Amodel(num_trees());
  for (int t = 0; t < num_trees(); ++t) {
    const int nt = num_tree_velocities_[t];
    Amodel[t].resize(nt, nt);
  }
  auto& linear_dynamics_matrix_cache_entry = this->DeclareCacheEntry(
      std::string("Linear dynamics matrix, A."),
      systems::ValueProducer(
          this, Amodel, &CompliantContactManager<T>::CalcLinearDynamicsMatrix),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.linear_dynamics_matrix =
      linear_dynamics_matrix_cache_entry.cache_index();

  auto& free_motion_velocities_cache_entry = this->DeclareCacheEntry(
      std::string("Free motion velocities, v*."),
      systems::ValueProducer(
          this, 
          VectorX<T>(plant().num_velocities()),
          &CompliantContactManager<T>::CalcFreeMotionVelocities),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.free_motion_velocities =
      free_motion_velocities_cache_entry.cache_index();

  // Due to issue #12786, we cannot mark
  // CacheIndexes::non_contact_forces_accelerations dependent on the
  // MultibodyPlant's inputs, as it should. However if we remove this
  // dependency, we run the risk of having an undetected algebraic loop. We use
  // this cache entry to signal when the computation of non-contact forces is in
  // progress so that we can detect an algebraic loop.
  auto& non_contact_forces_evaluation_in_progress = this->DeclareCacheEntry(
      "Evaluation of non-contact forces and accelerations is in progress.",
      // N.B. This flag is set to true only when the computation is in progress.
      // Therefore its default value is `false`.
      systems::ValueProducer(false, &systems::ValueProducer::NoopCalc),
      {systems::System<T>::nothing_ticket()});
  cache_indexes_.non_contact_forces_evaluation_in_progress =
      non_contact_forces_evaluation_in_progress.cache_index();

  // Accelerations due to non-contact forces.
  // We cache non-contact forces, ABA forces and accelerations into a
  // AccelerationsDueToExternalForcesCache.
  AccelerationsDueToExternalForcesCache<T> non_contact_forces_accelerations(
      this->internal_tree().get_topology());
  auto& non_contact_forces_accelerations_cache_entry = this->DeclareCacheEntry(
      std::string("Non-contact forces accelerations."),
      systems::ValueProducer(
          this, non_contact_forces_accelerations,
          &CompliantContactManager<
              T>::CalcAccelerationsDueToNonContactForcesCache),
      // Due to issue #12786, we cannot properly mark this entry dependent on
      // inputs. CalcAccelerationsDueToNonContactForcesCache() uses
      // CacheIndexes::non_contact_forces_evaluation_in_progress to guard
      // against algebraic loops.
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.non_contact_forces_accelerations =
      non_contact_forces_accelerations_cache_entry.cache_index();
}

template <typename T>
void CompliantContactManager<T>::CalcContactJacobianCache(
    const systems::Context<T>& context,
    internal::ContactJacobianCache<T>* cache) const {
  DRAKE_DEMAND(cache != nullptr);

  const std::vector<internal::DiscreteContactPair<T>>& contact_pairs =
      EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  MatrixX<T>& Jc = cache->Jc;
  Jc.resize(3 * num_contacts, plant().num_velocities());

  std::vector<drake::math::RotationMatrix<T>>& R_WC_set = cache->R_WC_list;
  R_WC_set.clear();
  R_WC_set.reserve(num_contacts);

  std::vector<std::pair<TreeJacobian<T>, TreeJacobian<T>>>& tree_blocks =
      cache->tree_blocks;
  tree_blocks.resize(num_contacts);

  // Quick no-op exit.
  if (num_contacts == 0) return;

  const int nv = plant().num_velocities();

  // Scratch workspace variables.
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);
  // Jacobian for the relative velocity measured in frame W and expressed in C.
  Matrix3X<T> Jv_W_AcBc_C(3, nv);

  const Frame<T>& frame_W = plant().world_frame();
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
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
    const math::RotationMatrix<T> R_WC =
        math::RotationMatrix<T>::MakeFromOneVector(nhat_W, 2);
    R_WC_set.push_back(R_WC);
    Jv_W_AcBc_C.noalias() = R_WC.matrix().transpose() * Jv_AcBc_W;

    const int treeA_index = body_to_tree_index_[bodyA_index];
    const int treeB_index = body_to_tree_index_[bodyB_index];

    // Sanity check, at least one must be positive.
    DRAKE_DEMAND(treeA_index >= 0 || treeB_index >= 0);

    if (treeA_index >= 0) {
      TreeJacobian<T>& tree_jacobian = tree_blocks[icontact].first;
      tree_jacobian.tree = treeA_index;
      tree_jacobian.J =
          Jv_W_AcBc_C.middleCols(tree_velocities_start_[treeA_index],
                                 num_tree_velocities_[treeA_index]);
    }

    // N.B. For self contact, when treeA_index = treeB_index, we only need to
    // extract the block once. Therefore we ignore this block of code for the
    // self-contact case.
    if (treeB_index >= 0 && treeB_index != treeA_index) {
      TreeJacobian<T>& tree_jacobian = tree_blocks[icontact].second;
      tree_jacobian.tree = treeB_index;
      tree_jacobian.J =
          Jv_W_AcBc_C.middleCols(tree_velocities_start_[treeB_index],
                                 num_tree_velocities_[treeB_index]);
    }

    Jc.template middleRows<3>(3 * icontact) = Jv_W_AcBc_C;
  }
}

template <typename T>
void CompliantContactManager<T>::AddContactConstraints(
    const systems::Context<T>& context, SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // TODO: make these parameters accessible.
  const double beta = 1.0;
  const double sigma = 1.0e-3;

  const std::vector<internal::DiscreteContactPair<T>>& contact_pairs =
      EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();  

  // Quick no-op exit.
  if (num_contacts == 0) return;

  const internal::ContactJacobianCache<T>& contact_cache =
      EvalContactJacobianCache(context);
  const std::vector<std::pair<TreeJacobian<T>, TreeJacobian<T>>>& tree_blocks =
      contact_cache.tree_blocks;

  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = contact_pairs[icontact];

    const T stiffness = point_pair.stiffness;
    const T dissipation_time_scale = point_pair.damping;
    const T friction = point_pair.mu;
    const T phi0 = point_pair.phi0;
    const auto& tree_jacobian_pair = tree_blocks[icontact];

    const typename SapFrictionConeConstraint<T>::Parameters parameters{
        friction, stiffness, dissipation_time_scale, beta, sigma};

    if (tree_jacobian_pair.first.tree < 0 ||
        tree_jacobian_pair.second.tree < 0) {
      // Contat only involves a single tree (contact with the world or
      // self-contact.)
      const TreeJacobian<T>& tree_jacobian = tree_jacobian_pair.first.tree >= 0
                                                 ? tree_jacobian_pair.first
                                                 : tree_jacobian_pair.second;
      problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<T>>(
          parameters, tree_jacobian.tree, tree_jacobian.J, phi0));
    } else {
      // Contact involves two distinct trees.
      const TreeJacobian<T>& treeA_jacobian = tree_jacobian_pair.first;
      const TreeJacobian<T>& treeB_jacobian = tree_jacobian_pair.second;
      problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<T>>(
          parameters, treeA_jacobian.tree, treeB_jacobian.tree,
          treeA_jacobian.J, treeB_jacobian.J, phi0));
    }
  }
}

template <typename T>
void CompliantContactManager<T>::AddCouplerConstraints(
    const systems::Context<T>& context, SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // Previous time step positions.
  const VectorX<T> q0 = plant().GetPositions(context);  

  for (const CouplerConstraintInfo& info : coupler_constraints_info_) {
    const int c0 = dof_to_tree_index_[info.q0];
    const int c1 = dof_to_tree_index_[info.q1];

    // Constraint function.
    const T g0 = q0[info.q0] - info.gear_ratio * q0[info.q1];

    // TODO: expose this parameter.
    const double beta = 0.1;

    const typename SapCouplerConstraint<T>::Parameters parameters{
        info.gear_ratio, info.stiffness, info.dissipation_time_scale, beta};

    if (c0 == c1) {
      const int nv = num_tree_velocities_[c0];
      MatrixX<T> J = MatrixX<T>::Zero(1, nv);
      // J = dg/dv
      J(0, info.q0) = 1.0;
      J(0, info.q1) = -info.gear_ratio;
      
      problem->AddConstraint(std::make_unique<SapCouplerConstraint<T>>(
          parameters, c0, J, g0));
    } else {
      const int nv0 = num_tree_velocities_[c0];
      const int nv1 = num_tree_velocities_[c1];
      MatrixX<T> J0 = MatrixX<T>::Zero(1, nv0);
      MatrixX<T> J1 = MatrixX<T>::Zero(1, nv1);
      J0(0, info.q0) = 1.0;
      J1(0, info.q1) = -info.gear_ratio;
      problem->AddConstraint(std::make_unique<SapCouplerConstraint<T>>(
          parameters, c0, c1, J0, J1, g0));
    }
  }

}

template <typename T>
void CompliantContactManager<T>::AddDistanceConstraints(
    const systems::Context<T>& context, SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
  // soft_tolerance. Using the soft norm we define the tangent vector as t̂ =
  // γₜ/‖γₜ‖ₛ, which is well defined event for γₜ = 0. Also gradients are well
  // defined and follow the same equations presented in [Castro et al., 2021]
  // where regular norms are simply replaced by soft norms.
  constexpr double kSoftTolerance = 1.0e-7;
  auto soft_norm =
      [eps = kSoftTolerance](const Eigen::Ref<const VectorX<T>>& x) -> T {
    using std::sqrt;
    return sqrt(x.squaredNorm() + eps * eps);
  };

  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAp_W(3, nv);
  Matrix3X<T> Jv_WBq_W(3, nv);
  MatrixX<T> Jddot(1, nv);

  const Frame<T>& frame_W = plant().world_frame();
  for (const DistanceConstraintInfo& info : distance_constraints_info_) {
    const Body<T>& body_A = plant().get_body(info.body_A);
    const Body<T>& body_B = plant().get_body(info.body_B);

    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, body_A);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, body_B);
    const Vector3<T>& p_WP = X_WA * info.p_AP;
    const Vector3<T>& p_WQ = X_WB * info.p_BQ;

    // Distance as the soft norm of p_PQ_W = p_WQ - p_WP
    const Vector3<T> p_PQ_W = p_WQ - p_WP;
    const T d0 = soft_norm(p_PQ_W);
    const Vector3<T> p_hat_W = p_PQ_W / d0;  // (soft) unit vector.

    PRINT_VAR(d0);

    // Dense Jacobian.
    this->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W, p_WP,
        frame_W, frame_W, &Jv_WAp_W);
    this->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W, p_WQ,
        frame_W, frame_W, &Jv_WBq_W);
    Jddot = p_hat_W.transpose() * (Jv_WBq_W - Jv_WAp_W);  // ddot = Jddot * v.

    PRINT_VAR(Jddot);

    // TODO: expose this parameter.
    const double beta = 0.1;
    const typename SapDistanceConstraint<T>::Parameters parameters{
        info.distance, info.stiffness, info.dissipation_time_scale, beta};

    PRINT_VAR(info.body_A);
    PRINT_VAR(info.body_B);

    const int treeA_index = body_to_tree_index_[info.body_A];
    const int treeB_index = body_to_tree_index_[info.body_B];

    PRINT_VAR(treeA_index);
    PRINT_VAR(treeB_index);

    // Sanity check, at least one must be positive.
    DRAKE_DEMAND(treeA_index >= 0 || treeB_index >= 0);

    // Self constraint or with the world.
    if (treeA_index < 0 || treeB_index < 0 || treeA_index == treeB_index) {
      const int tree_index = treeA_index >= 0 ? treeA_index : treeB_index;
      const MatrixX<T> J = Jddot.middleCols(tree_velocities_start_[tree_index],
                                            num_tree_velocities_[tree_index]);
      PRINT_VARn(J);      
      problem->AddConstraint(std::make_unique<SapDistanceConstraint<T>>(
          parameters, tree_index, J, d0));
    } else {
      const MatrixX<T> JA =
          Jddot.middleCols(tree_velocities_start_[treeA_index],
                           num_tree_velocities_[treeA_index]);
      const MatrixX<T> JB =
          Jddot.middleCols(tree_velocities_start_[treeB_index],
                           num_tree_velocities_[treeB_index]);
      problem->AddConstraint(std::make_unique<SapDistanceConstraint<T>>(
          parameters, treeA_index, treeB_index, JA, JB, d0));
    }
  }
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
  // N.B. Here we rely on the resolution of #13289 and #5454 to get properties
  // with the proper scalar type T. This will not work on scalar converted
  // models until those issues are resolved.
  return prop->template GetPropertyOrDefault<T>(
      geometry::internal::kMaterialGroup, "dissipation_time_constant",
      plant().time_step());
}

template <typename T>
const CoulombFriction<double>& CompliantContactManager<T>::GetCoulombFriction(
    geometry::GeometryId id,
    const geometry::SceneGraphInspector<T>& inspector) const {
  if constexpr (std::is_same_v<symbolic::Expression, T>) {
    throw std::domain_error(
        "This method doesn't support T = symbolic::Expression.");
  }
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  DRAKE_THROW_UNLESS(prop->HasProperty(geometry::internal::kMaterialGroup,
                                       geometry::internal::kFriction));
  return prop->GetProperty<CoulombFriction<double>>(
      geometry::internal::kMaterialGroup, geometry::internal::kFriction);
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
    std::vector<internal::DiscreteContactPair<T>>* contact_pairs) const {
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
    std::vector<internal::DiscreteContactPair<T>>* result) const {
  std::vector<internal::DiscreteContactPair<T>>& contact_pairs = *result;

  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

  // Simple utility to detect 0 / 0. As it is used in this method, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](const T& num, const T& denom) {
    return denom == 0.0 ? 0.0 : num / denom;
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
    const double muA =
        GetCoulombFriction(pair.id_A, inspector).dynamic_friction();
    const double muB =
        GetCoulombFriction(pair.id_B, inspector).dynamic_friction();
    const T mu = T(safe_divide(2.0 * muA * muB, muA + muB));

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kA + kB;
    const T wA = (denom == 0 ? 0.5 : kA / denom);
    const T wB = (denom == 0 ? 0.5 : kB / denom);
    const Vector3<T> p_WC = wA * pair.p_WCa + wB * pair.p_WCb;

    const T phi0 = -pair.depth;
    const T fn0 = -k * phi0;
    contact_pairs.push_back(
        {pair.id_A, pair.id_B, p_WC, pair.nhat_BA_W, phi0, fn0, k, tau, mu});
  }
}

// Most of the calculation in this function should be the same as in
// MultibodyPlant<T>::CalcDiscreteContactPairs().
template <typename T>
void CompliantContactManager<T>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const systems::Context<T>& context,
        std::vector<internal::DiscreteContactPair<T>>* result) const {
  std::vector<internal::DiscreteContactPair<T>>& contact_pairs = *result;

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

    const T tau_M = GetDissipationTimeConstant(s.id_M(), inspector);
    const T tau_N = GetDissipationTimeConstant(s.id_N(), inspector);
    const T tau = CombineDissipationTimeConstant(tau_M, tau_N);

    // Combine friction coefficients.
    const double muA =
        GetCoulombFriction(s.id_M(), inspector).dynamic_friction();
    const double muB =
        GetCoulombFriction(s.id_N(), inspector).dynamic_friction();
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
                     ? s.tri_e_MN().Evaluate(
                face, tri_centroid_barycentric)
                     : s.poly_e_MN().EvaluateCartesian(face, p_WQ);

        // Force contribution by this quadrature point.
        const T fn0 = Ae * p0;

        // Effective compliance in the normal direction for the given
        // discrete patch, refer to [Masterjohn et al., 2021] for details.
        // [Masterjohn, 2021] Masterjohn J., Guoy D., Shepherd J. and Castro
        // A., 2021. Discrete Approximation of Pressure Field Contact Patches.
        // Available at https://arxiv.org/abs/2110.04157.
        const T k = Ae * g;

        // phi < 0 when in penetration.
        const T phi0 = -p0 / g;

        if (k > 0) {
          contact_pairs.push_back(
              {s.id_M(), s.id_N(), p_WQ, nhat_W, phi0, fn0, k, tau, mu});
        }        
      }
    }
  }
}

template <typename T>
void CompliantContactManager<T>::CalcAccelerationsDueToNonContactForcesCache(
    const systems::Context<T>& context,
    AccelerationsDueToExternalForcesCache<T>* no_contact_accelerations_cache)
    const {
  // To overcame issue #12786, we use this additional cache entry
  // to detect algebraic loops.
  systems::CacheEntryValue& value =
      plant()
          .get_cache_entry(
              cache_indexes_.non_contact_forces_evaluation_in_progress)
          .get_mutable_cache_entry_value(context);
  bool& evaluation_in_progress = value.GetMutableValueOrThrow<bool>();
  if (evaluation_in_progress) {
    const char* error_message =
        "Algebraic loop detected. This situation is caused when connecting the "
        "input of your MultibodyPlant to the output of a feedback system which "
        "is an algebraic function of a feedthrough output of the plant. Ways "
        "to remedy this: 1. Revisit the model for your feedback system. "
        "Consider if its output can be written in terms of other inputs. 2. "
        "Break the algebraic loop by adding state to the controller, typically "
        "to 'remember' a previous input. 3. Break the algebraic loop by adding "
        "a zero-order hold system between the output of the plant and your "
        "feedback system. This effectively delays the input signal to the "
        "controller.";
    throw std::runtime_error(error_message);
  }
  // Mark the start of the computation. If within an algebraic
  // loop, pulling from the plant's input ports during the
  // computation will trigger the recursive evaluation of this
  // method and the exception above will be thrown.
  evaluation_in_progress = true;
  // If the exception above is triggered, we will leave this method and the
  // computation will no longer be "in progress". We use a scoped guard so that
  // we have a chance to mark it as such when we leave this scope.
  ScopeExit guard(
      [&evaluation_in_progress]() { evaluation_in_progress = false; });

  this->CalcNonContactForces(context, &no_contact_accelerations_cache->forces);
  this->internal_tree().CalcArticulatedBodyForceCache(
      context, no_contact_accelerations_cache->forces,
      &no_contact_accelerations_cache->aba_forces);
  this->internal_tree().CalcArticulatedBodyAccelerations(
      context, no_contact_accelerations_cache->aba_forces,
      &no_contact_accelerations_cache->ac);

  // Mark the end of the computation.
  evaluation_in_progress = false;
}

template <typename T>
void CompliantContactManager<T>::CalcLinearDynamicsMatrix(
    const systems::Context<T>& context, std::vector<MatrixX<T>>* A) const {
  DRAKE_DEMAND(A != nullptr);  
  A->resize(num_trees());
  const int nv = plant().num_velocities();

  // TODO(amcastro-tri): implicitly include force elements such as joint
  // dissipation and/or stiffness.
  // TODO(amcastro-tri): consider placing the computation of the dense mass
  // matrix  in a cache entry to minimize heap allocations or better yet,
  // implement a MultibodyPlant method to compute the per-tree mass matrices.
  MatrixX<T> M(nv, nv);
  plant().CalcMassMatrix(context, &M);

  int v = 0;
  for (int t = 0; t < num_trees(); ++t) {
    const int nt = num_tree_velocities_[t];
    (*A)[t].resize(nt, nt);
    (*A)[t] = M.block(v, v, nt, nt);
    v += nt;
  }
}

template <typename T>
void CompliantContactManager<T>::CalcFreeMotionVelocities(
    const systems::Context<T>& context, VectorX<T>* v_star) const {
  DRAKE_DEMAND(v_star != nullptr);
  // N.B. Forces are evaluated at the previous time step state. This is
  // consistent with the explicit Euler and symplectic Euler schemes.
  // TODO(amcastro-tri): Implement free-motion velocities update based on the
  // theta-method, as in the SAP paper.
  const VectorX<T>& vdot0 =
      EvalAccelerationsDueToNonContactForcesCache(context).get_vdot();
  const double dt = this->plant().time_step();
  const VectorX<T>& x0 =
      context.get_discrete_state(this->multibody_state_index()).value();
  const auto v0 = x0.bottomRows(this->plant().num_velocities());
  *v_star = v0 + dt * vdot0;
}

template <typename T>
const std::vector<internal::DiscreteContactPair<T>>&
CompliantContactManager<T>::EvalDiscreteContactPairs(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.discrete_contact_pairs)
      .template Eval<std::vector<internal::DiscreteContactPair<T>>>(context);
}

template <typename T>
const internal::ContactJacobianCache<T>&
CompliantContactManager<T>::EvalContactJacobianCache(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_jacobian)
      .template Eval<internal::ContactJacobianCache<T>>(context);
}

template <typename T>
const std::vector<MatrixX<T>>&
CompliantContactManager<T>::EvalLinearDynamicsMatrix(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.linear_dynamics_matrix)
      .template Eval<std::vector<MatrixX<T>>>(context);
}

template <typename T>
const VectorX<T>& CompliantContactManager<T>::EvalFreeMotionVelocities(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.free_motion_velocities)
      .template Eval<VectorX<T>>(context);
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
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  // In the absence of contact, v_next = v*.
  VectorX<T> v_star = EvalFreeMotionVelocities(context);
  std::vector<MatrixX<T>> A = EvalLinearDynamicsMatrix(context);

  const double time_step = plant().time_step();
  // TODO: notice that above this (move) constructor requires making a copy
  // (heap allocation) of both A and v_star. Consider a constructor that keeps a
  // reference to valid data in this scope.
  auto problem = std::make_unique<SapContactProblem<T>>(
        time_step, std::move(A), std::move(v_star));

  // Setup constraints.  
  AddContactConstraints(context, problem.get());
  AddCouplerConstraints(context, problem.get());
  AddDistanceConstraints(context, problem.get());

  // We use the velocity stored in the current context as initial guess.
  const VectorX<T>& x0 =
      context.get_discrete_state(this->multibody_state_index()).value();
  const auto v0 = x0.bottomRows(this->plant().num_velocities());

  // Solve contact problem.
  drake::multibody::contact_solvers::internal::SapSolverParameters params;
  //params.rel_tolerance = 1.0e-6;
  SapSolver<T> sap;
  sap.set_parameters(params);
  const drake::multibody::contact_solvers::internal::ContactSolverStatus
      status = sap.SolveWithGuess(*problem, v0, results);
  if (status != drake::multibody::contact_solvers::internal::
                    ContactSolverStatus::kSuccess) {
    throw std::runtime_error("SAP solver failed.");
  }

  //const typename SapSolver<T>::SolverStats& stats = sap.get_statistics();
  //PRINT_VAR(stats.num_iters);
}

template <typename T>
void CompliantContactManager<T>::DoCalcDiscreteValues(
    const drake::systems::Context<T>& context,
    drake::systems::DiscreteValues<T>* updates) const {
  const contact_solvers::internal::ContactSolverResults<T>& results =
      this->EvalContactSolverResults(context);

  // Previous time step positions.
  const int nq = plant().num_positions();
  const auto x0 =
      context.get_discrete_state(this->multibody_state_index()).get_value();
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

template <typename T>
void CompliantContactManager<T>::ExtractModelInfo() {
  const MultibodyTreeTopology& topology =  
      internal::GetInternalTree(this->plant()).get_topology();

  // Sanity check: I expect always the first node to be connected to the world.
  // Otherwise something went terribly wrong.
  // TODO: remove. Note that the check assumes at least one tree and therefore
  // not work for empty models.
  //const BodyNodeTopology& first_tree_base =
  //    topology.get_body_node(BodyNodeIndex(1));
  //DRAKE_DEMAND(first_tree_base.level == 1);

  const BodyNodeTopology& root = topology.get_body_node(BodyNodeIndex(0));
  const int num_trees = root.child_nodes.size();
  num_tree_velocities_.resize(num_trees, 0);
  body_to_tree_index_.resize(plant().num_bodies(), -1);
  dof_to_tree_index_.resize(plant().num_velocities(), -1);

  int t = -1;  // current tree.
  // Traverse nodes in their DFT order, skiping the world.
  for (BodyNodeIndex node_index(1); node_index < topology.get_num_body_nodes();
       ++node_index) {
    const BodyNodeTopology& node = topology.get_body_node(node_index);    
    if (node.level == 1) ++t;
    num_tree_velocities_[t] += node.num_mobilizer_velocities;
    body_to_tree_index_[node.body] = t;
    for (int i = 0; i < node.num_mobilizer_velocities; ++i) {
      const int v = node.mobilizer_velocities_start_in_v + i;
      dof_to_tree_index_[v] = t;
    }
  }

  // When I'm done t should point at the last tree.
  // TODO: remove.
  DRAKE_DEMAND(t == (num_trees - 1));  // sanity check.

  PRINT_VAR(num_tree_velocities_.size());
  for (int nt : num_tree_velocities_) {
    PRINT_VAR(nt);
  }

  tree_velocities_start_.resize(num_trees, 0);
  for (t = 1; t < num_trees; ++t) {
    tree_velocities_start_[t] =
        tree_velocities_start_[t - 1] + num_tree_velocities_[t - 1];
  }

  // Double check all dofs were assigned a tree.
  PRINT_VAR(dof_to_tree_index_.size());
  for (auto ti : dof_to_tree_index_) {
    PRINT_VAR(ti);
    DRAKE_DEMAND(ti >= 0);
  }

}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
