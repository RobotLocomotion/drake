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
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::MatrixBlock;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace internal {
namespace {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
constexpr auto kDiscreteContactSolverTamsi = DiscreteContactSolver::kTamsi;
#pragma GCC diagnostic pop
}  // namespace

template <typename T>
AccelerationsDueNonConstraintForcesCache<
    T>::AccelerationsDueNonConstraintForcesCache(const internal::SpanningForest&
                                                     forest)
    : forces(forest.num_links(), forest.num_velocities()),
      abic(forest),
      Zb_Bo_W(forest.num_mobods()),
      aba_forces(forest),
      ac(forest) {}

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

  // Accelerations due to non-constraint forces.
  // We cache non-contact forces, ABA forces and accelerations into an
  // AccelerationsDueNonConstraintForcesCache.
  AccelerationsDueNonConstraintForcesCache<T>
      non_constraint_forces_accelerations(this->internal_tree().forest());
  const auto& non_constraint_forces_accelerations_cache_entry =
      this->DeclareCacheEntry(
          "Non-constraint forces and induced accelerations.",
          systems::ValueProducer(
              this, non_constraint_forces_accelerations,
              &CompliantContactManager<
                  T>::CalcAccelerationsDueToNonConstraintForcesCache),
          // This includes contribution from force elements, which could
          // involve user-injected dependencies. So we need to include all
          // possible tickets that users can choose to depend on.
          {systems::System<T>::all_input_ports_ticket(),
           systems::System<T>::xd_ticket(),
           systems::System<T>::all_parameters_ticket(),
           systems::System<T>::time_ticket(),
           systems::System<T>::accuracy_ticket()});
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
      plant().EvalJointDampingCache(context) * plant().time_step();
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

  if (plant().get_discrete_contact_solver() == kDiscreteContactSolverTamsi) {
    DRAKE_DEMAND(tamsi_driver_ != nullptr);
    tamsi_driver_->CalcContactSolverResults(context, contact_results);
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
    case kDiscreteContactSolverTamsi:
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
  // Discard the deformable velocities (keeping only the MbT velocities).
  const auto& v_next = results.v_next.head(plant().num_velocities());

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
      plant().get_discrete_contact_solver() == kDiscreteContactSolverTamsi);

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

  if (plant().get_discrete_contact_solver() == kDiscreteContactSolverTamsi) {
    DRAKE_DEMAND(tamsi_driver_ != nullptr);
    tamsi_driver_->CalcDiscreteUpdateMultibodyForces(context, forces);
  }
}

template <typename T>
void CompliantContactManager<T>::DoCalcActuation(
    const systems::Context<T>& context, VectorX<T>* actuation) const {
  // Thus far only TAMSI and SAP are supported. Verify this is true.
  DRAKE_DEMAND(
      plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap ||
      plant().get_discrete_contact_solver() == kDiscreteContactSolverTamsi);

  if (plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap) {
    if constexpr (std::is_same_v<T, symbolic::Expression>) {
      throw std::logic_error(
          "Discrete updates with the SAP solver are not supported for T = "
          "symbolic::Expression");
    } else {
      DRAKE_DEMAND(sap_driver_ != nullptr);
      sap_driver_->CalcActuation(context, actuation);
    }
  }

  if (plant().get_discrete_contact_solver() == kDiscreteContactSolverTamsi) {
    DRAKE_DEMAND(tamsi_driver_ != nullptr);
    // TAMSI does not model additional actuation terms as SAP does.
    *actuation = this->EvalActuationInput(context, /* effort_limit = */ true);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
