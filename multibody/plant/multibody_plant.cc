#include "drake/multibody/plant/multibody_plant.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include <fmt/ranges.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/render/render_label.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/hydroelastics/hydroelastic_engine.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/geometry_contact_data.h"
#include "drake/multibody/plant/hydroelastic_contact_forces_continuous_cache_data.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/plant/make_discrete_update_manager.h"
#include "drake/multibody/plant/slicing_and_indexing.h"
#include "drake/multibody/topology/graph.h"
#include "drake/multibody/tree/door_hinge.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"

namespace drake {
namespace multibody {

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

using drake::geometry::CollisionFilterDeclaration;
using drake::geometry::CollisionFilterScope;
using drake::geometry::ContactSurface;
using drake::geometry::FrameId;
using drake::geometry::FramePoseVector;
using drake::geometry::GeometryFrame;
using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::GeometrySet;
using drake::geometry::PenetrationAsPointPair;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::SceneGraphInspector;
using drake::geometry::SourceId;
using drake::geometry::render::RenderLabel;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::internal::AccelerationKinematicsCache;
using drake::multibody::internal::ArticulatedBodyForceCache;
using drake::multibody::internal::ArticulatedBodyInertiaCache;
using drake::multibody::internal::DiscreteStepMemory;
using drake::multibody::internal::GeometryContactData;
using drake::multibody::internal::NestedGeometryContactData;
using drake::multibody::internal::PositionKinematicsCache;
using drake::multibody::internal::VelocityKinematicsCache;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DependencyTicket;
using drake::systems::InputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPort;
using drake::systems::OutputPortIndex;
using drake::systems::State;

namespace internal {
// This is a helper struct used to estimate the parameters used in the penalty
// method to enforce joint limits.
// The penalty method applies at each joint, a spring-damper force with
// parameters estimated by this struct.
// Once a joint reaches a limit (either lower or upper), the governing equations
// for that joint's coordinate can be approximated by a harmonic oscillator with
// stiffness and damping corresponding to the penalty parameters for that joint
// as:  q̈ + 2ζω₀ q̇ + ω₀² q = 0, where ω₀² = k / m̃ is the characteristic
// numerical stiffness frequency and m̃ is an inertia term computed differently
// for prismatic and revolute joints.
// The numerical frequency is defined as ω₀ = 2π/τ₀ with τ₀ = αδt a numerical
// stiffness time scale set to be proportional to the time step of the discrete
// model. The damping ratio ζ is set to one, corresponding to a critically
// damped oscillator and thus so that the penalty method emulates the effect of
// a "hard" limit.
// Knowing ω₀ (from the time step) and m̃ (a function of the bodies connected by
// the joint), it is possible, from the equations for a harmonic oscillator, to
// estimate the stiffness k and damping d parameters for the penalty method.
// Finally, MultibodyPlant uses a value of α to guarantee the stability of the
// method (from a stability analysis of the time stepping method for the
// model of a harmonic oscillator).
// Using this estimation procedure, the stiffness k is shown to be proportional
// to the inverse of the time step squared, i.e. k ∝ 1/δt².
// Since, at steady state, the violation of the joint limit is inversely
// proportional to the stiffness parameter, this violation turns out being
// proportional to the time step squared, that is, Δq ∝ δt².
// Therefore the convergence of the joint limit violation is expected to be
// quadratic with the time step.
template <typename T>
struct JointLimitsPenaltyParametersEstimator {
  // This helper method returns a pair (k, d) (in that order) for a harmonic
  // oscillator given the period τ₀ of the oscillator and the inertia m̃. d is
  // computed for a critically damped oscillator.
  // The harmonic oscillator model corresponds to:
  //    m̃q̈ + d q̇ + k q = 0
  // or equivalently:
  //    q̈ + 2ζω₀ q̇ + ω₀² q = 0
  // with ω₀ = sqrt(k/m̃) and ζ = d/sqrt(km̃)/2 the damping ratio, which is one
  // for critically damped oscillators.
  static std::pair<double, double>
  CalcCriticallyDampedHarmonicOscillatorParameters(double period,
                                                   double inertia) {
    const double damping_ratio = 1.0;  // Critically damped.
    const double omega0 = 2.0 * M_PI / period;
    const double stiffness = inertia * omega0 * omega0;
    const double damping = 2.0 * damping_ratio * std::sqrt(inertia * stiffness);
    return std::make_pair(stiffness, damping);
  }

  // This method combines a pair of penalty parameters params1 and params2.
  // The combination law is very simple, this method returns the set of
  // parameters with the smallest stiffness, and thus it favors the stiffness
  // leading to the lower numerical stiffness (thus guaranteeing stability).
  static std::pair<double, double> PickLessStiffPenaltyParameters(
      const std::pair<double, double>& params1,
      const std::pair<double, double>& params2) {
    const double stiffness1 = params1.first;
    const double stiffness2 = params2.first;
    if (stiffness1 < stiffness2) {
      return params1;
    } else {
      return params2;
    }
  }

  // Helper method to estimate the penalty parameters for a prismatic joint.
  // The strategy consists in computing a set of penalty parameters for each
  // body connected by joint as if the other body was welded and ignoring
  // any other bodies in the system. This leads to a spring mass system where
  // the inertia m̃ corresponds to the mass of the body in consideration.
  // Then the penalty parameters estimated for each body are combined with
  // PickLessStiffPenaltyParameters() leading to a single set of parameters.
  static std::pair<double, double> CalcPrismaticJointPenaltyParameters(
      const PrismaticJoint<T>& joint, double numerical_time_scale) {
    // Penalty parameters for the parent body (child fixed).
    const double parent_mass = joint.parent_body().index() == world_index()
                                   ? std::numeric_limits<double>::infinity()
                                   : joint.parent_body().default_mass();
    const auto parent_params = CalcCriticallyDampedHarmonicOscillatorParameters(
        numerical_time_scale, parent_mass);
    // Penalty parameters for the child body (parent fixed).
    const double child_mass = joint.child_body().index() == world_index()
                                  ? std::numeric_limits<double>::infinity()
                                  : joint.child_body().default_mass();
    const auto child_params = CalcCriticallyDampedHarmonicOscillatorParameters(
        numerical_time_scale, child_mass);

    // Return the combined penalty parameters of the two bodies.
    auto params = PickLessStiffPenaltyParameters(parent_params, child_params);

    return params;
  }

  // Helper method to estimate the penalty parameters for a revolute joint.
  // The strategy consists in computing a set of penalty parameters for each
  // body connected by joint as if the other body was welded and ignoring
  // any other bodies in the system. This leads to a torsional spring system
  // for which the inertia m̃ corresponds to the rotational inertia of the body
  // in consideration, computed about the axis of the joint.
  // Then the penalty parameters estimated for each body are combined with
  // PickLessStiffPenaltyParameters() leading to a single set of parameters.
  static std::pair<double, double> CalcRevoluteJointPenaltyParameters(
      const RevoluteJoint<T>& joint, double numerical_time_scale) {
    // For the body attached to `frame` (one of the parent/child frames of
    // `joint`), this helper lambda computes the rotational inertia of the body
    // about the axis of the joint.
    // That is, it computes Iₐ = âᵀ⋅Iᴮ⋅â where Iᴮ is the rotational inertia of
    // the body, â is the axis of the joint, and Iₐ is the (scalar) rotational
    // inertia of the body computed about the joint's axis. Iₐ is the inertia
    // that must be considered for the problem of a pendulum oscillating about
    // an axis â, leading to the equations for a harmonic oscillator when we
    // apply the penalty forces.
    // For further details on Iₐ, the interested reader can refer to
    // [Goldstein, 2014, §5.3].
    //
    // [Goldstein, 2014] Goldstein, H., Poole, C.P. and Safko, J.L., 2014.
    //                   Classical Mechanics: Pearson New International Edition.
    //                   Pearson Higher Ed.
    auto CalcRotationalInertiaAboutAxis = [&joint](const Frame<T>& frame) {
      const RigidBody<T>* body =
          dynamic_cast<const RigidBody<T>*>(&frame.body());
      DRAKE_THROW_UNLESS(body != nullptr);

      // This check is needed for such models for which the user leaves the
      // spatial inertias unspecified (i.e. initialized to NaN). A user
      // might do this when only interested in performing kinematics
      // computations.
      if (std::isnan(body->default_mass())) {
        return std::numeric_limits<double>::infinity();
      }

      const SpatialInertia<T>& M_PPo_P =
          body->default_spatial_inertia().template cast<T>();
      const RigidTransform<T> X_PJ = frame.GetFixedPoseInBodyFrame();
      const Vector3<T>& p_PJ = X_PJ.translation();
      const math::RotationMatrix<T>& R_PJ = X_PJ.rotation();
      const SpatialInertia<T> M_PJo_J = M_PPo_P.Shift(p_PJ).ReExpress(R_PJ);
      const RotationalInertia<T> I_PJo_J = M_PJo_J.CalcRotationalInertia();
      // Rotational inertia about the joint axis.
      const Vector3<T>& axis = joint.revolute_axis();
      const T I_a = axis.transpose() * (I_PJo_J * axis);
      return ExtractDoubleOrThrow(I_a);
    };

    // Rotational inertia about the joint's axis for the parent body.
    const double I_Pa =
        joint.parent_body().index() == world_index()
            ? std::numeric_limits<double>::infinity()
            : CalcRotationalInertiaAboutAxis(joint.frame_on_parent());
    auto parent_params = CalcCriticallyDampedHarmonicOscillatorParameters(
        numerical_time_scale, I_Pa);

    // Rotational inertia about the joint's axis for the child body.
    const double I_Ca =
        joint.child_body().index() == world_index()
            ? std::numeric_limits<double>::infinity()
            : CalcRotationalInertiaAboutAxis(joint.frame_on_child());
    auto child_params = CalcCriticallyDampedHarmonicOscillatorParameters(
        numerical_time_scale, I_Ca);

    // Return the combined penalty parameters of the two bodies.
    return PickLessStiffPenaltyParameters(parent_params, child_params);
  }
};
}  // namespace internal

namespace {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
constexpr auto kDiscreteContactSolverTamsi = DiscreteContactSolver::kTamsi;
constexpr auto kDiscreteContactApproximationTamsi =
    DiscreteContactApproximation::kTamsi;
#pragma GCC diagnostic push

// Hack to fully qualify frame names, pending resolution of #9128. Used by
// geometry registration routines. When this hack is removed, also undo the
// de-hacking step within internal_geometry_names.cc. Note that unlike the
// ScopedName convention, here the world and default model instances do not
// use any scoping.
template <typename T>
std::string GetScopedName(const MultibodyPlant<T>& plant,
                          ModelInstanceIndex model_instance,
                          const std::string& name) {
  if (model_instance != world_model_instance() &&
      model_instance != default_model_instance()) {
    return plant.GetModelInstanceName(model_instance) + "::" + name;
  } else {
    return name;
  }
}

// Helper that returns `true` iff any joint actuator in the model is PD
// controlled.
template <typename T>
bool AnyActuatorHasPdControl(const MultibodyPlant<T>& plant) {
  for (JointActuatorIndex a : plant.GetJointActuatorIndices()) {
    if (plant.get_joint_actuator(a).has_controller()) return true;
  }
  return false;
}

// Retrieves the DiscreteStepMemory pointer from state in the given `context`.
// If there is no memory (e.g., has never been updated by a step), returns null.
// @pre The context is from a plant with use_sampled_output_ports() == true.
template <typename T>
const DiscreteStepMemory::Data<T>* get_discrete_step_memory(
    const Context<T>& context) {
  return context.template get_abstract_state<DiscreteStepMemory>(0)
      .template get<T>();
}

// Checks the given vector for NaNs, unless the vector is symbolic in which case
// always returns false.
template <typename EigenMatrix>
bool HasNaN(const EigenMatrix& x) {
  if constexpr (scalar_predicate<typename EigenMatrix::Scalar>::is_bool) {
    return x.hasNaN();
  } else {
    return false;
  }
}

}  // namespace

template <typename T>
MultibodyPlant<T>::MultibodyPlant(double time_step)
    : MultibodyPlant(nullptr, time_step) {
  // Cross-check that the Config default matches our header file default.
  DRAKE_DEMAND(contact_model_ == ContactModel::kHydroelasticWithFallback);
  DRAKE_DEMAND(MultibodyPlantConfig{}.contact_model ==
               "hydroelastic_with_fallback");
  DRAKE_DEMAND(discrete_contact_approximation_ ==
               DiscreteContactApproximation::kLagged);
  DRAKE_DEMAND(MultibodyPlantConfig{}.discrete_contact_approximation ==
               "lagged");
}

template <typename T>
MultibodyPlant<T>::MultibodyPlant(
    std::unique_ptr<internal::MultibodyTree<T>> tree_in, double time_step)
    : internal::MultibodyTreeSystem<T>(systems::SystemTypeTag<MultibodyPlant>{},
                                       std::move(tree_in), time_step > 0),
      contact_surface_representation_(
          GetDefaultContactSurfaceRepresentation(time_step)),
      time_step_(time_step),
      use_sampled_output_ports_(time_step > 0) {
  DRAKE_THROW_UNLESS(time_step >= 0);
  // TODO(eric.cousineau): Combine all of these elements into one struct, make
  // it less brittle.
  visual_geometries_.emplace_back();  // Entries for the "world" body.
  collision_geometries_.emplace_back();

  AddDeformableModel();
  DeclareSceneGraphPorts();
}

template <typename T>
template <typename U>
MultibodyPlant<T>::MultibodyPlant(const MultibodyPlant<U>& other)
    : internal::MultibodyTreeSystem<T>(
          systems::SystemTypeTag<MultibodyPlant>{},
          other.internal_tree().template CloneToScalar<T>(),
          other.is_discrete()) {
  DRAKE_THROW_UNLESS(other.is_finalized());

  // Here we step through every member field one by one, in the exact order
  // they are declared in the header, so that a reader could mindlessly compare
  // this function to the private fields, and check that every single field got
  // a mention.
  // For each field, this function will either:
  // (1) Copy the field directly.
  // (2) Place a forward-reference comment like "We initialize
  //     geometry_query_port_ during DeclareSceneGraphPorts, below."
  // (3) Place a disclaimer comment why that field does not need to be copied
  {
    source_id_ = other.source_id_;
    penalty_method_contact_parameters_ =
        other.penalty_method_contact_parameters_;
    penetration_allowance_ = other.penetration_allowance_;
    // Copy over the friction model if it is initialized. Otherwise, a default
    // value will be set in FinalizePlantOnly().
    // Note that stiction_tolerance is the only real data field in
    // `friction_model_`, so setting the stiction tolerance is equivalent to
    // copying `friction_model_`.
    if (other.friction_model_.stiction_tolerance() > 0) {
      friction_model_.set_stiction_tolerance(
          other.friction_model_.stiction_tolerance());
    }
    // joint_limit_parameters_ is set in SetUpJointLimitsParameters() in
    // FinalizePlantOnly().
    body_index_to_frame_id_ = other.body_index_to_frame_id_;
    frame_id_to_body_index_ = other.frame_id_to_body_index_;
    geometry_id_to_body_index_ = other.geometry_id_to_body_index_;
    visual_geometries_ = other.visual_geometries_;
    num_visual_geometries_ = other.num_visual_geometries_;
    collision_geometries_ = other.collision_geometries_;
    num_collision_geometries_ = other.num_collision_geometries_;
    contact_model_ = other.contact_model_;
    discrete_contact_approximation_ = other.discrete_contact_approximation_;
    sap_near_rigid_threshold_ = other.sap_near_rigid_threshold_;
    contact_surface_representation_ = other.contact_surface_representation_;
    // geometry_query_port_ is set during DeclareSceneGraphPorts() below.
    // geometry_pose_port_ is set during DeclareSceneGraphPorts() below.
    // scene_graph_ is set to nullptr in FinalizePlantOnly() below.

    time_step_ = other.time_step_;
    use_sampled_output_ports_ = other.use_sampled_output_ports_;
    // discrete_update_manager_ is copied below after FinalizePlantOnly().

    // Copy over physical models.
    // Note: The physical models must be cloned before `FinalizePlantOnly()` is
    // called because `FinalizePlantOnly()` has to allocate system resources
    // requested by physical models.
    physical_models_ = other.physical_models_->template CloneToScalar<T>(this);
    this->RemoveUnsupportedScalars(*physical_models_);

    coupler_constraints_specs_ = other.coupler_constraints_specs_;
    distance_constraints_params_ = other.distance_constraints_params_;
    ball_constraints_specs_ = other.ball_constraints_specs_;
    weld_constraints_specs_ = other.weld_constraints_specs_;
    tendon_constraints_specs_ = other.tendon_constraints_specs_;

    adjacent_bodies_collision_filters_ =
        other.adjacent_bodies_collision_filters_;

    // These are set as part of FinalizePlantOnly().
    // - input_port_indices_
    // - output_port_indices_
    // - cache_indices_
    // - parameter_indices_
  }

  DeclareSceneGraphPorts();

  // MultibodyTree::CloneToScalar() already called MultibodyTree::Finalize()
  // on the new MultibodyTree on U. Therefore we only Finalize the plant's
  // internals (and not the MultibodyTree).
  FinalizePlantOnly();

  // Note: The discrete update manager needs to be copied *after* the plant is
  // finalized.
  if (is_discrete()) {
    DRAKE_DEMAND(other.discrete_update_manager_ != nullptr);
    SetDiscreteUpdateManager(
        other.discrete_update_manager_->template CloneToScalar<T>());
  }
}

template <typename T>
MultibodyPlant<T>::~MultibodyPlant() = default;

template <typename T>
void MultibodyPlant<T>::SetUseSampledOutputPorts(
    bool use_sampled_output_ports) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  if (!is_discrete()) {
    DRAKE_THROW_UNLESS(use_sampled_output_ports == false);
  }
  use_sampled_output_ports_ = use_sampled_output_ports;
}

template <typename T>
std::vector<MultibodyConstraintId> MultibodyPlant<T>::GetConstraintIds() const {
  std::vector<MultibodyConstraintId> ids;
  // This list must be kept up to date with the types of user-addable
  // constraints. See #21415 for a potentially more robust / sustainable
  // solution.
  for (const auto& [id, _] : coupler_constraints_specs_) {
    ids.push_back(id);
  }
  for (const auto& [id, _] : distance_constraints_params_) {
    ids.push_back(id);
  }
  for (const auto& [id, _] : ball_constraints_specs_) {
    ids.push_back(id);
  }
  for (const auto& [id, _] : weld_constraints_specs_) {
    ids.push_back(id);
  }
  for (const auto& [id, _] : tendon_constraints_specs_) {
    ids.push_back(id);
  }

  return ids;
}

template <typename T>
bool MultibodyPlant<T>::GetConstraintActiveStatus(
    const systems::Context<T>& context, MultibodyConstraintId id) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  const std::map<MultibodyConstraintId, bool>& constraint_active_status =
      this->GetConstraintActiveStatus(context);
  DRAKE_THROW_UNLESS(constraint_active_status.contains(id));
  return constraint_active_status.at(id);
}

template <typename T>
void MultibodyPlant<T>::SetConstraintActiveStatus(systems::Context<T>* context,
                                                  MultibodyConstraintId id,
                                                  bool status) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  std::map<MultibodyConstraintId, bool>& constraint_active_status =
      this->GetMutableConstraintActiveStatus(context);
  DRAKE_THROW_UNLESS(constraint_active_status.contains(id));
  constraint_active_status[id] = status;
}

template <typename T>
MultibodyConstraintId MultibodyPlant<T>::AddCouplerConstraint(
    const Joint<T>& joint0, const Joint<T>& joint1, double gear_ratio,
    double offset) {
  // N.B. The manager is setup at Finalize() and therefore we must require
  // constraints to be added pre-finalize.
  DRAKE_MBP_THROW_IF_FINALIZED();

  if (is_discrete()) {
    switch (get_discrete_contact_solver()) {
      case kDiscreteContactSolverTamsi:
        // TAMSI does not support coupler constraints.
        throw std::runtime_error(
            "Currently this MultibodyPlant is set to use the TAMSI solver. "
            "TAMSI does not support coupler constraints. Use "
            "set_discrete_contact_approximation() to set a model approximation "
            "that uses the SAP solver instead (kSap, kSimilar, or kLagged).");
      case DiscreteContactSolver::kSap:
        // SAP supports coupler constraints.
        break;
    }
  }
  // Feature support for continuous time plants depends on the integrator used.

  if (joint0.num_velocities() != 1 || joint1.num_velocities() != 1) {
    const std::string message = fmt::format(
        "Coupler constraints can only be defined on single-DOF joints. "
        "However joint '{}' has {} DOFs and joint '{}' has {} "
        "DOFs.",
        joint0.name(), joint0.num_velocities(), joint1.name(),
        joint1.num_velocities());
    throw std::runtime_error(message);
  }

  const MultibodyConstraintId constraint_id =
      MultibodyConstraintId::get_new_id();

  coupler_constraints_specs_[constraint_id] = internal::CouplerConstraintSpec{
      joint0.index(), joint1.index(), gear_ratio, offset, constraint_id};

  return constraint_id;
}

template <typename T>
MultibodyConstraintId MultibodyPlant<T>::AddDistanceConstraint(
    const RigidBody<T>& body_A, const Vector3<double>& p_AP,
    const RigidBody<T>& body_B, const Vector3<double>& p_BQ, double distance,
    double stiffness, double damping) {
  // N.B. The manager is setup at Finalize() and therefore we must require
  // constraints to be added pre-finalize.
  DRAKE_MBP_THROW_IF_FINALIZED();

  if (is_discrete()) {
    switch (get_discrete_contact_solver()) {
      case kDiscreteContactSolverTamsi:
        // TAMSI does not support distance constraints.
        throw std::runtime_error(
            "Currently this MultibodyPlant is set to use the TAMSI solver. "
            "TAMSI does not support distance constraints. Use "
            "set_discrete_contact_approximation() to set a model approximation "
            "that uses the SAP solver instead (kSap, kSimilar, or kLagged).");
      case DiscreteContactSolver::kSap:
        // SAP supports distance constraints.
        break;
    }
  }
  // Feature support for continuous time plants depends on the integrator used.

  const MultibodyConstraintId constraint_id =
      MultibodyConstraintId::get_new_id();

  DistanceConstraintParams params(body_A.index(), p_AP, body_B.index(), p_BQ,
                                  distance, stiffness, damping);
  distance_constraints_params_[constraint_id] = params;

  return constraint_id;
}

template <typename T>
const std::map<MultibodyConstraintId, DistanceConstraintParams>&
MultibodyPlant<T>::GetDefaultDistanceConstraintParams() const {
  return distance_constraints_params_;
}

template <typename T>
const std::map<MultibodyConstraintId, DistanceConstraintParams>&
MultibodyPlant<T>::GetDistanceConstraintParams(
    const systems::Context<T>& context) const {
  this->ValidateContext(context);
  return context.get_parameters()
      .template get_abstract_parameter<internal::DistanceConstraintParamsMap>(
          parameter_indices_.distance_constraints)
      .map;
}

template <typename T>
std::map<MultibodyConstraintId, DistanceConstraintParams>&
MultibodyPlant<T>::GetMutableDistanceConstraintParams(
    systems::Context<T>* context) const {
  return context->get_mutable_parameters()
      .template get_mutable_abstract_parameter<
          internal::DistanceConstraintParamsMap>(
          parameter_indices_.distance_constraints)
      .map;
}

template <typename T>
const DistanceConstraintParams& MultibodyPlant<T>::GetDistanceConstraintParams(
    const systems::Context<T>& context, MultibodyConstraintId id) const {
  this->ValidateContext(context);
  if (!distance_constraints_params_.contains(id)) {
    throw std::runtime_error(
        fmt::format("The constraint id {} does not match any distance "
                    "constraint registered with this plant. ",
                    id));
  }
  const std::map<MultibodyConstraintId, DistanceConstraintParams>& all_params =
      GetDistanceConstraintParams(context);
  DRAKE_ASSERT(all_params.contains(id));
  return all_params.at(id);
}

template <typename T>
void MultibodyPlant<T>::SetDistanceConstraintParams(
    systems::Context<T>* context, MultibodyConstraintId id,
    DistanceConstraintParams params) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->ValidateContext(*context);
  if (!distance_constraints_params_.contains(id)) {
    throw std::runtime_error(
        fmt::format("The constraint id {} does not match any distance "
                    "constraint registered with this plant. ",
                    id));
  }

  if (!has_body(params.bodyA())) {
    throw std::runtime_error(
        fmt::format("Index {} provided for body A does not correspond to a "
                    "rigid body in this MultibodyPlant.",
                    params.bodyA()));
  }

  if (!has_body(params.bodyB())) {
    throw std::runtime_error(
        fmt::format("Index {} provided for body B does not correspond to a "
                    "rigid body in this MultibodyPlant.",
                    params.bodyB()));
  }

  std::map<MultibodyConstraintId, DistanceConstraintParams>& all_params =
      GetMutableDistanceConstraintParams(context);
  DRAKE_ASSERT(all_params.contains(id));
  all_params.at(id) = std::move(params);
}

template <typename T>
MultibodyConstraintId MultibodyPlant<T>::AddBallConstraint(
    const RigidBody<T>& body_A, const Vector3<double>& p_AP,
    const RigidBody<T>& body_B, const std::optional<Vector3<double>>& p_BQ) {
  // N.B. The manager is set up at Finalize() and therefore we must require
  // constraints to be added pre-finalize.
  DRAKE_MBP_THROW_IF_FINALIZED();

  if (is_discrete()) {
    switch (get_discrete_contact_solver()) {
      case kDiscreteContactSolverTamsi:
        // TAMSI does not support ball constraints.
        throw std::runtime_error(
            "Currently this MultibodyPlant is set to use the TAMSI solver. "
            "TAMSI does not support ball constraints. Use "
            "set_discrete_contact_approximation() to set a model approximation "
            "that uses the SAP solver instead (kSap, kSimilar, or kLagged).");
      case DiscreteContactSolver::kSap:
        // SAP supports ball constraints.
        break;
    }
  }
  // Feature support for continuous time plants depends on the integrator used.

  const MultibodyConstraintId constraint_id =
      MultibodyConstraintId::get_new_id();

  internal::BallConstraintSpec spec{body_A.index(), p_AP, body_B.index(), p_BQ,
                                    constraint_id};
  if (!spec.IsValid()) {
    const std::string msg = fmt::format(
        "Invalid set of parameters for constraint between bodies '{}' and "
        "'{}'. For a ball constraint, points P and Q must be on two distinct "
        "bodies, i.e. body_A != body_B must be satisfied.",
        body_A.name(), body_B.name());
    throw std::logic_error(msg);
  }

  ball_constraints_specs_[constraint_id] = spec;

  return constraint_id;
}

template <typename T>
MultibodyConstraintId MultibodyPlant<T>::AddWeldConstraint(
    const RigidBody<T>& body_A, const math::RigidTransform<double>& X_AP,
    const RigidBody<T>& body_B, const math::RigidTransform<double>& X_BQ) {
  // N.B. The manager is set up at Finalize() and therefore we must require
  // constraints to be added pre-finalize.
  DRAKE_MBP_THROW_IF_FINALIZED();

  if (is_discrete()) {
    switch (get_discrete_contact_solver()) {
      case kDiscreteContactSolverTamsi:
        // TAMSI does not support weld constraints.
        throw std::runtime_error(
            "Currently this MultibodyPlant is set to use the TAMSI solver. "
            "TAMSI does not support weld constraints. Use "
            "set_discrete_contact_approximation() to set a model approximation "
            "that uses the SAP solver instead (kSap, kSimilar, or kLagged).");
      case DiscreteContactSolver::kSap:
        // SAP supports weld constraints.
        break;
    }
  }
  // Feature support for continuous time plants depends on the integrator used.

  const MultibodyConstraintId constraint_id =
      MultibodyConstraintId::get_new_id();

  internal::WeldConstraintSpec spec{body_A.index(), X_AP, body_B.index(), X_BQ,
                                    constraint_id};
  if (!spec.IsValid()) {
    const std::string msg = fmt::format(
        "AddWeldConstraint(): Invalid set of parameters for constraint between "
        "bodies '{}' and '{}'. For a weld constraint, frames P and Q must be "
        "on two distinct bodies, i.e. body_A != body_B must be satisfied.",
        body_A.name(), body_B.name());
    throw std::logic_error(msg);
  }

  weld_constraints_specs_[constraint_id] = spec;

  return constraint_id;
}

template <typename T>
MultibodyConstraintId MultibodyPlant<T>::AddTendonConstraint(
    std::vector<JointIndex> joints, std::vector<double> a,
    std::optional<double> offset, std::optional<double> lower_limit,
    std::optional<double> upper_limit, std::optional<double> stiffness,
    std::optional<double> damping) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  // N.B. The manager is set up at Finalize() and therefore we must require
  // constraints to be added pre-finalize.
  DRAKE_MBP_THROW_IF_FINALIZED();

  if (is_discrete()) {
    switch (get_discrete_contact_solver()) {
      case kDiscreteContactSolverTamsi:
        // TAMSI does not support tendon constraints.
        throw std::runtime_error(
            "Currently this MultibodyPlant is set to use the TAMSI solver. "
            "TAMSI does not support tendon constraints. Use "
            "set_discrete_contact_approximation() to set a model approximation "
            "that uses the SAP solver instead (kSap, kSimilar, or kLagged).");
      case DiscreteContactSolver::kSap:
        // SAP supports tendon constraints.
        break;
    }
  }
  // Feature support for continuous time plants depends on the integrator used.

  DRAKE_THROW_UNLESS(joints.size() > 0);

  // Detect if `joints` contains a unique set of JointIndex.
  std::vector<JointIndex> sorted_joints = joints;
  std::sort(sorted_joints.begin(), sorted_joints.end());
  auto last = std::unique(sorted_joints.begin(), sorted_joints.end());
  if (last != sorted_joints.end()) {
    throw std::runtime_error(
        "AddTendonConstraint(): Duplicated joint in `joints`. `joints` must be "
        "a unique set of JointIndex.");
  }

  DRAKE_THROW_UNLESS(a.size() == joints.size());

  for (int i = 0; i < ssize(joints); ++i) {
    DRAKE_THROW_UNLESS(this->has_joint(joints[i]));
    DRAKE_THROW_UNLESS(this->get_joint(joints[i]).num_velocities() == 1);
  }

  if (!offset.has_value()) {
    offset = 0.0;
  }

  if (lower_limit.has_value()) {
    DRAKE_THROW_UNLESS(*lower_limit < kInf);
  } else {
    lower_limit = -kInf;
  }

  if (upper_limit.has_value()) {
    DRAKE_THROW_UNLESS(*upper_limit > -kInf);
  } else {
    upper_limit = kInf;
  }

  DRAKE_THROW_UNLESS(*lower_limit != -kInf || *upper_limit != kInf);
  DRAKE_THROW_UNLESS(*lower_limit <= *upper_limit);

  if (stiffness.has_value()) {
    DRAKE_THROW_UNLESS(*stiffness > 0.0);
  } else {
    stiffness = kInf;
  }

  if (damping.has_value()) {
    DRAKE_THROW_UNLESS(*damping >= 0.0);
  } else {
    damping = 0.0;
  }

  const MultibodyConstraintId constraint_id =
      MultibodyConstraintId::get_new_id();

  internal::TendonConstraintSpec spec{
      std::move(joints), std::move(a), *offset,  *lower_limit,
      *upper_limit,      *stiffness,   *damping, constraint_id};

  tendon_constraints_specs_[constraint_id] = spec;

  return constraint_id;
}

template <typename T>
void MultibodyPlant<T>::RemoveConstraint(MultibodyConstraintId id) {
  // N.B. The manager and parameters are set up at Finalize() and therefore we
  // must require constraints to be removed pre-finalize.
  DRAKE_MBP_THROW_IF_FINALIZED();

  int num_removed = 0;
  num_removed += coupler_constraints_specs_.erase(id);
  num_removed += distance_constraints_params_.erase(id);
  num_removed += ball_constraints_specs_.erase(id);
  num_removed += weld_constraints_specs_.erase(id);
  num_removed += tendon_constraints_specs_.erase(id);
  if (num_removed != 1) {
    throw std::runtime_error(fmt::format(
        "RemoveConstraint(): The constraint id {} does not match "
        "any constraint registered with this plant. Note that this method does "
        "not check constraints registered with DeformableModel.",
        id));
  }
}

template <typename T>
std::string MultibodyPlant<T>::GetTopologyGraphvizString() const {
  std::string graphviz = "digraph MultibodyPlant {\n";
  graphviz += "label=\"" + this->get_name() + "\";\n";
  graphviz += "rankdir=BT;\n";
  graphviz += "labelloc=t;\n";
  // Create a subgraph for each model instance, with the bodies as nodes.
  // Note that the subgraph name must have the "cluster" prefix in order to
  // have the box drawn.
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    graphviz += fmt::format("subgraph cluster{} {{\n", model_instance_index);
    graphviz += fmt::format(" label=\"{}\";\n",
                            GetModelInstanceName(model_instance_index));
    for (const BodyIndex& body_index : GetBodyIndices(model_instance_index)) {
      const RigidBody<T>& body = get_body(body_index);
      graphviz +=
          fmt::format(" body{} [label=\"{}\"];\n", body.index(), body.name());
    }
    graphviz += "}\n";
  }
  // Add the graph edges (via the joints).
  for (JointIndex joint_index : GetJointIndices()) {
    const Joint<T>& joint = get_joint(joint_index);
    graphviz += fmt::format(
        "body{} -> body{} [label=\"{} [{}]\"];\n", joint.child_body().index(),
        joint.parent_body().index(), joint.name(), joint.type_name());
  }
  // TODO(russt): Consider adding actuators, frames, forces, etc.
  graphviz += "}\n";
  return graphviz;
}

template <typename T>
void MultibodyPlant<T>::set_contact_model(ContactModel model) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  contact_model_ = model;
}

template <typename T>
DiscreteContactSolver MultibodyPlant<T>::get_discrete_contact_solver() const {
  // Only the TAMSI approximation uses the TAMSI solver.
  if (discrete_contact_approximation_ == kDiscreteContactApproximationTamsi)
    return kDiscreteContactSolverTamsi;
  // All other approximations use the SAP solver.
  return DiscreteContactSolver::kSap;
}

template <typename T>
void MultibodyPlant<T>::set_discrete_contact_approximation(
    DiscreteContactApproximation approximation) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(is_discrete());

  if (approximation == kDiscreteContactApproximationTamsi &&
      num_constraints() > 0) {
    throw std::runtime_error(fmt::format(
        "You selected TAMSI as the contact approximation, but you have "
        "constraints registered with this model (num_constraints() == {}). "
        "TAMSI does not support constraints.",
        num_constraints()));
  }

  discrete_contact_approximation_ = approximation;
}

template <typename T>
DiscreteContactApproximation
MultibodyPlant<T>::get_discrete_contact_approximation() const {
  return discrete_contact_approximation_;
}

template <typename T>
void MultibodyPlant<T>::set_sap_near_rigid_threshold(
    double near_rigid_threshold) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(near_rigid_threshold >= 0.0);
  sap_near_rigid_threshold_ = near_rigid_threshold;
}

template <typename T>
double MultibodyPlant<T>::get_sap_near_rigid_threshold() const {
  return sap_near_rigid_threshold_;
}

template <typename T>
ContactModel MultibodyPlant<T>::get_contact_model() const {
  return contact_model_;
}

template <typename T>
int MultibodyPlant<T>::num_collision_geometries() const {
  int result = num_collision_geometries_;
  if constexpr (std::is_same_v<T, double>) {
    const DeformableModel<T>* deformable_model =
        physical_models_->deformable_model();
    if (deformable_model != nullptr) {
      // We assume that all deformable bodies have proximity properties.
      result += deformable_model->num_bodies();
    }
  }
  return result;
}

template <typename T>
void MultibodyPlant<T>::SetFreeBodyRandomRotationDistributionToUniform(
    const RigidBody<T>& body) {
  RandomGenerator generator;
  auto q_FM = math::UniformlyRandomQuaternion<symbolic::Expression>(&generator);
  SetFreeBodyRandomRotationDistribution(body, q_FM);
}

template <typename T>
void MultibodyPlant<T>::RemoveJoint(const Joint<T>& joint) {
  // Check for non-zero number of user-added force elements. (There is always 1
  // force element, the gravity field added when the tree is constructed.)
  if (num_force_elements() > 1) {
    throw std::logic_error(fmt::format(
        "{}: This plant has {} user-added force elements. This plant must have "
        "0 user-added force elements in order to remove joint with index {}.",
        __func__, num_force_elements() - 1, joint.index()));
  }

  // Check for non-zero number of user-added constraints.
  if (num_constraints() > 0) {
    throw std::logic_error(fmt::format(
        "{}: This plant has {} user-added constraints. This plant must have "
        "0 user-added constraints in order to remove joint with index {}.",
        __func__, num_constraints(), joint.index()));
  }

  // Check for dependent JointActuators in the plant. Throw if any actuator
  // depends on the joint to be removed.
  // TODO(#21415): Find a way to make checking an element for dependency on a
  // Joint more robust and uniform across all MultibodyElements and constraints.
  std::vector<std::string> dependent_elements;

  // Check JointActuators.
  for (JointActuatorIndex index : GetJointActuatorIndices()) {
    const JointActuator<T>& actuator = get_joint_actuator(index);
    if (actuator.joint().index() == joint.index()) {
      dependent_elements.push_back(
          fmt::format("JointActuator(name: {}, index: {})", actuator.name(),
                      actuator.index()));
    }
  }

  if (dependent_elements.size() > 0) {
    throw std::logic_error(fmt::format(
        "{}: joint with index {} has the following dependent model elements "
        "which must be removed prior to joint removal: [{}].",
        __func__, joint.index(), fmt::join(dependent_elements, ", ")));
  }

  this->mutable_tree().RemoveJoint(joint);
}

template <typename T>
const WeldJoint<T>& MultibodyPlant<T>::WeldFrames(
    const Frame<T>& frame_on_parent_F, const Frame<T>& frame_on_child_M,
    const math::RigidTransform<double>& X_FM) {
  const std::string joint_name =
      frame_on_parent_F.name() + "_welds_to_" + frame_on_child_M.name();
  return AddJoint(std::make_unique<WeldJoint<T>>(joint_name, frame_on_parent_F,
                                                 frame_on_child_M, X_FM));
}

template <typename T>
const JointActuator<T>& MultibodyPlant<T>::AddJointActuator(
    const std::string& name, const Joint<T>& joint, double effort_limit) {
  if (joint.num_velocities() != 1) {
    throw std::logic_error(fmt::format(
        "Calling AddJointActuator with joint {} failed -- this joint has "
        "{} degrees of freedom, and MultibodyPlant currently only "
        "supports actuators for single degree-of-freedom joints. "
        "See https://stackoverflow.com/q/71477852/9510020 for "
        "the common workarounds.",
        joint.name(), joint.num_velocities()));
  }
  return this->mutable_tree().AddJointActuator(name, joint, effort_limit);
}

template <typename T>
void MultibodyPlant<T>::RemoveJointActuator(const JointActuator<T>& actuator) {
  this->mutable_tree().RemoveJointActuator(actuator);
}

template <typename T>
void MultibodyPlant<T>::RemoveAllJointActuatorEffortLimits() {
  for (const JointActuatorIndex& i : GetJointActuatorIndices()) {
    get_mutable_joint_actuator(i).set_effort_limit(
        std::numeric_limits<double>::infinity());
  }
}

template <typename T>
geometry::SourceId MultibodyPlant<T>::RegisterAsSourceForSceneGraph(
    SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(!geometry_source_is_registered());
  // Save the GS pointer so that on later geometry registrations can use this
  // instance. This will be nullified at Finalize().
  scene_graph_ = scene_graph;
  source_id_ = scene_graph_->RegisterSource(this->get_name());
  const geometry::FrameId world_frame_id = scene_graph_->world_frame_id();
  body_index_to_frame_id_[world_index()] = world_frame_id;
  frame_id_to_body_index_[world_frame_id] = world_index();
  // In case any bodies were added before registering scene graph, make sure the
  // bodies get their corresponding geometry frame ids.
  RegisterGeometryFramesForAllBodies();
  return source_id_.value();
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name) {
  return RegisterVisualGeometry(body, X_BG, shape, name,
                                geometry::IllustrationProperties());
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const Vector4<double>& diffuse_color) {
  return RegisterVisualGeometry(
      body, X_BG, shape, name,
      geometry::MakePhongIllustrationProperties(diffuse_color));
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const geometry::IllustrationProperties& properties) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());

  // Note: the geometry name will be scoped in the subsequent call to
  // RegisterVisualGeometry().
  auto instance =
      std::make_unique<geometry::GeometryInstance>(X_BG, shape, name);
  instance->set_illustration_properties(properties);
  instance->set_perception_properties(
      geometry::PerceptionProperties(properties));

  const std::optional<geometry::GeometryId> id =
      RegisterVisualGeometry(body, std::move(instance));
  // We assigned properties, so this must be defined.
  DRAKE_DEMAND(id.has_value());

  return *id;
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const RigidBody<T>& body,
    std::unique_ptr<geometry::GeometryInstance> geometry) {
  DRAKE_THROW_UNLESS(geometry != nullptr);
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());

  if (geometry->perception_properties() == nullptr &&
      geometry->illustration_properties() == nullptr) {
    geometry->set_illustration_properties(geometry::IllustrationProperties());
    geometry->set_perception_properties(geometry::PerceptionProperties());
  }

  // Map ("label", "id") to body index as necessary.
  geometry::PerceptionProperties* percep =
      geometry->mutable_perception_properties();
  if (percep != nullptr) {
    if (!percep->HasProperty("label", "id")) {
      percep->AddProperty("label", "id", RenderLabel(body.index()));
    }
  }

  geometry->set_name(
      GetScopedName(*this, body.model_instance(), geometry->name()));

  const geometry::GeometryId id = RegisterGeometry(body, std::move(geometry));

  DRAKE_ASSERT(ssize(visual_geometries_) == num_bodies());
  visual_geometries_[body.index()].push_back(id);
  ++num_visual_geometries_;

  return id;
}

template <typename T>
const std::vector<geometry::GeometryId>&
MultibodyPlant<T>::GetVisualGeometriesForBody(const RigidBody<T>& body) const {
  return visual_geometries_[body.index()];
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterCollisionGeometry(
    const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    geometry::ProximityProperties properties) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());

  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register geometry that has a fixed path to world to the world body (i.e.,
  // as anchored geometry).
  GeometryId id = RegisterGeometry(
      body,
      std::make_unique<geometry::GeometryInstance>(
          X_BG, shape, GetScopedName(*this, body.model_instance(), name)));

  scene_graph_->AssignRole(*source_id_, id, std::move(properties));
  DRAKE_ASSERT(ssize(collision_geometries_) == num_bodies());
  collision_geometries_[body.index()].push_back(id);
  ++num_collision_geometries_;
  return id;
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterCollisionGeometry(
    const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const CoulombFriction<double>& coulomb_friction) {
  geometry::ProximityProperties props;
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kFriction, coulomb_friction);
  return RegisterCollisionGeometry(body, X_BG, shape, name, std::move(props));
}

template <typename T>
const std::vector<geometry::GeometryId>&
MultibodyPlant<T>::GetCollisionGeometriesForBody(
    const RigidBody<T>& body) const {
  DRAKE_ASSERT(body.index() < num_bodies());
  return collision_geometries_[body.index()];
}

template <typename T>
geometry::GeometrySet MultibodyPlant<T>::CollectRegisteredGeometries(
    const std::vector<const RigidBody<T>*>& bodies) const {
  DRAKE_THROW_UNLESS(geometry_source_is_registered());

  geometry::GeometrySet geometry_set;
  for (const RigidBody<T>* body : bodies) {
    std::optional<FrameId> frame_id = GetBodyFrameIdIfExists(body->index());
    if (frame_id) {
      geometry_set.Add(frame_id.value());
    }
  }
  return geometry_set;
}

template <typename T>
const SceneGraphInspector<T>& MultibodyPlant<T>::EvalSceneGraphInspector(
    const systems::Context<T>& context) const {
  // TODO(jwnimmer-tri) The "geometry_query" input port is invalidated anytime
  // the configuration_ticket changes, but really we only need to invalidate the
  // inspector when the scene graph topology or properties change. If we find
  // this is a performance bottleneck, this is something we could clean up.
  return EvalGeometryQueryInput(context, __func__).inspector();
}

template <typename T>
std::vector<const RigidBody<T>*> MultibodyPlant<T>::GetBodiesWeldedTo(
    const RigidBody<T>& body) const {
  const internal::LinkJointGraph& graph = internal_tree().graph();
  const std::set<BodyIndex> island =
      graph.forest_is_valid() ? graph.GetLinksWeldedTo(body.index())
                              : graph.CalcLinksWeldedTo(body.index());
  // Map body indices to pointers.
  std::vector<const RigidBody<T>*> sub_graph_bodies;
  for (BodyIndex body_index : island) {
    sub_graph_bodies.push_back(&get_body(body_index));
  }
  return sub_graph_bodies;
}

template <typename T>
std::vector<BodyIndex> MultibodyPlant<T>::GetBodiesKinematicallyAffectedBy(
    const std::vector<JointIndex>& joint_indexes) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  for (const JointIndex& joint : joint_indexes) {
    if (!has_joint(joint)) {
      throw std::logic_error(
          fmt::format("{}: No joint with index {} has been registered or it "
                      "has been removed.",
                      __func__, joint));
    }
  }
  const std::set<BodyIndex> links =
      internal_tree().GetBodiesKinematicallyAffectedBy(joint_indexes);
  // TODO(sherm1) Change the return type to set to avoid this copy.
  return std::vector<BodyIndex>(links.cbegin(), links.cend());
}

template <typename T>
std::unordered_set<BodyIndex> MultibodyPlant<T>::GetFloatingBaseBodies() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::unordered_set<BodyIndex> floating_bodies;
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (body.is_floating_base_body()) floating_bodies.insert(body.index());
  }
  return floating_bodies;
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterGeometry(
    const RigidBody<T>& body,
    std::unique_ptr<geometry::GeometryInstance> instance) {
  DRAKE_ASSERT(!is_finalized());
  DRAKE_ASSERT(geometry_source_is_registered());
  DRAKE_ASSERT(body_has_registered_frame(body));

  // Register geometry in the body frame.
  GeometryId geometry_id = scene_graph_->RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.index()],
      std::move(instance));
  geometry_id_to_body_index_[geometry_id] = body.index();
  return geometry_id;
}

template <typename T>
void MultibodyPlant<T>::RegisterGeometryFramesForAllBodies() {
  DRAKE_ASSERT(geometry_source_is_registered());
  // Loop through the bodies to make sure that all bodies get a geometry frame.
  // If not, create and attach one.
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const auto& body = get_body(body_index);
    RegisterRigidBodyWithSceneGraph(body);
  }
}

template <typename T>
void MultibodyPlant<T>::RegisterRigidBodyWithSceneGraph(
    const RigidBody<T>& body) {
  if (geometry_source_is_registered()) {
    // If not already done, register a frame for this body.
    if (!body_has_registered_frame(body)) {
      FrameId frame_id = scene_graph_->RegisterFrame(
          source_id_.value(),
          GeometryFrame(
              GetScopedName(*this, body.model_instance(), body.name()),
              /* TODO(@SeanCurtis-TRI): Add test coverage for this
               * model-instance support as requested in #9390. */
              body.model_instance()));
      body_index_to_frame_id_[body.index()] = frame_id;
      frame_id_to_body_index_[frame_id] = body.index();
    }
  }
}

template <typename T>
void MultibodyPlant<T>::SetFloatingBaseBodyPoseInWorldFrame(
    systems::Context<T>* context, const RigidBody<T>& body,
    const math::RigidTransform<T>& X_WB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(body.is_floating_base_body());
  this->ValidateContext(context);
  internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context);
}

template <typename T>
void MultibodyPlant<T>::SetFloatingBaseBodyPoseInAnchoredFrame(
    systems::Context<T>* context, const Frame<T>& frame_F,
    const RigidBody<T>& body, const math::RigidTransform<T>& X_FB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(body.is_floating_base_body());
  this->ValidateContext(context);

  if (!internal_tree()
           .graph()
           .link_by_index(frame_F.body().index())
           .is_anchored()) {
    throw std::logic_error("Frame '" + frame_F.name() +
                           "' must be anchored to the world frame.");
  }

  // Pose of frame F in its parent body frame P (not state dependent).
  const RigidTransform<T>& X_PF = frame_F.EvalPoseInBodyFrame(*context);
  // Pose of frame F's parent body P in the world.
  // TODO(sherm1) This shouldn't be state dependent since F is anchored, but
  //  it currently is due to the way we evaluate poses.
  const RigidTransform<T>& X_WP = EvalBodyPoseInWorld(*context, frame_F.body());
  // Pose of floating base body C's body frame in the world frame.
  const RigidTransform<T> X_WB = X_WP * X_PF * X_FB;
  SetFloatingBaseBodyPoseInWorldFrame(context, body, X_WB);
}

template <typename T>
void MultibodyPlant<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context, const VectorX<T>& known_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  this->ValidateContext(context);
  DRAKE_THROW_UNLESS(A_WB_array != nullptr);
  DRAKE_THROW_UNLESS(ssize(*A_WB_array) == num_bodies());
  const internal::MultibodyTree<T>& multibody_tree = internal_tree();
  const internal::SpanningForest& forest = multibody_tree.forest();

  // TODO(eric.cousineau): Remove dynamic allocations. Making this in-place
  //  still required dynamic allocation for recording permutation indices.
  //  Can change implementation once MultibodyTree becomes fully internal.
  std::vector<SpatialAcceleration<T>> A_WB_array_mobod(forest.num_mobods());
  multibody_tree.CalcSpatialAccelerationsFromVdot(
      context, multibody_tree.EvalPositionKinematics(context),
      multibody_tree.EvalVelocityKinematics(context), known_vdot,
      &A_WB_array_mobod);

  // Permute MobodIndex -> BodyIndex.
  for (const auto& mobod : forest.mobods()) {
    // TODO(sherm1) Need to calculate accelerations (optionally?) for the
    //  inactive links on a link composite following this mobod.
    // Make sure there aren't any inactives for now.
    DRAKE_DEMAND(ssize(mobod.follower_link_ordinals()) == 1);
    const BodyIndex active_link_index =
        forest.links(mobod.link_ordinal()).index();
    (*A_WB_array)[active_link_index] = A_WB_array_mobod[mobod.index()];
  }
}

template <typename T>
void MultibodyPlant<T>::CalcForceElementsContribution(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  DRAKE_THROW_UNLESS(forces != nullptr);
  DRAKE_THROW_UNLESS(forces->CheckHasRightSizeForModel(internal_tree()));
  internal_tree().CalcForceElementsContribution(
      context, EvalPositionKinematics(context), EvalVelocityKinematics(context),
      forces);
}

template <typename T>
void MultibodyPlant<T>::RenameModelInstance(ModelInstanceIndex model_instance,
                                            const std::string& name) {
  DRAKE_THROW_UNLESS(!is_finalized());
  const std::string old_name(GetModelInstanceName(model_instance));
  if (old_name == name) {
    return;
  }
  this->mutable_tree().RenameModelInstance(model_instance, name);
  if (geometry_source_is_registered()) {
    // Re-spam frame and geometry names, if they contain the model name as
    // constructed by GetScopedName(). Otherwise, leave non-matching names
    // alone.
    auto& inspector = scene_graph_->model_inspector();

    const std::string old_prefix(old_name + "::");
    const std::string new_prefix(name + "::");
    auto maybe_make_new_name = [&](auto id) {
      std::string existing_name(inspector.GetName(id));
      if (existing_name.starts_with(old_prefix)) {
        // Replace old prefix with new prefix.
        return new_prefix + existing_name.substr(old_prefix.size());
      }
      // Return empty string to signal no match found.
      return std::string();
    };

    std::string new_name;
    for (auto& frame_id : inspector.FramesForSource(*source_id_)) {
      if (inspector.GetFrameGroup(frame_id) != model_instance) {
        continue;
      }
      new_name = maybe_make_new_name(frame_id);
      if (!new_name.empty()) {
        scene_graph_->RenameFrame(frame_id, new_name);
      }

      auto geoms = inspector.GetGeometryIds(GeometrySet(frame_id));
      for (auto& geom_id : geoms) {
        new_name = maybe_make_new_name(geom_id);
        if (!new_name.empty()) {
          scene_graph_->RenameGeometry(geom_id, new_name);
        }
      }
    }
  }
}

template <typename T>
void MultibodyPlant<T>::SetBaseBodyJointType(
    BaseBodyJointType joint_type,
    std::optional<ModelInstanceIndex> model_instance) {
  std::optional<internal::ForestBuildingOptions> options;
  switch (joint_type) {
    case BaseBodyJointType::kQuaternionFloatingJoint:
      options = internal::ForestBuildingOptions::kDefault;
      break;
    case BaseBodyJointType::kRpyFloatingJoint:
      options = internal::ForestBuildingOptions::kUseRpyFloatingJoints;
      break;
    case BaseBodyJointType::kWeldJoint:
      options = internal::ForestBuildingOptions::kUseFixedBase;
      break;
  }
  DRAKE_DEMAND(options.has_value());
  DRAKE_THROW_UNLESS(!is_finalized());
  internal::LinkJointGraph& graph = mutable_tree().mutable_graph();
  if (model_instance.has_value()) {
    graph.SetForestBuildingOptions(*model_instance, *options);
  } else {
    graph.SetGlobalForestBuildingOptions(*options);
  }
}

template <typename T>
BaseBodyJointType MultibodyPlant<T>::GetBaseBodyJointType(
    std::optional<ModelInstanceIndex> model_instance) const {
  const internal::LinkJointGraph& graph = internal_tree().graph();
  const internal::ForestBuildingOptions options =
      model_instance.has_value()
          ? graph.get_forest_building_options_in_use(*model_instance)
          : graph.get_global_forest_building_options();
  if (static_cast<bool>(options &
                        internal::ForestBuildingOptions::kUseRpyFloatingJoints))
    return BaseBodyJointType::kRpyFloatingJoint;
  if (static_cast<bool>(options &
                        internal::ForestBuildingOptions::kUseFixedBase))
    return BaseBodyJointType::kWeldJoint;
  return BaseBodyJointType::kQuaternionFloatingJoint;
}

template <typename T>
void MultibodyPlant<T>::Finalize() {
  // After finalizing the base class, tree is read-only.
  internal::MultibodyTreeSystem<T>::Finalize();

  if (geometry_source_is_registered()) {
    ApplyDefaultCollisionFilters();
  }
  FinalizePlantOnly();

  // Make the manager of discrete updates.
  if (is_discrete()) {
    std::unique_ptr<internal::DiscreteUpdateManager<T>> manager =
        internal::MakeDiscreteUpdateManager<T>(get_discrete_contact_solver());
    if (manager) {
      SetDiscreteUpdateManager(std::move(manager));
    }
  }

  if (!is_discrete() && AnyActuatorHasPdControl(*this)) {
    throw std::logic_error(
        "Continuous model with PD controlled joint actuators. This feature is "
        "only supported for discrete models. Refer to MultibodyPlant's "
        "documentation for further details.");
  }
}

template <typename T>
void MultibodyPlant<T>::SetUpJointLimitsParameters() {
  for (JointIndex joint_index : GetJointIndices()) {
    // Currently MultibodyPlant applies these "compliant" joint limit forces
    // using an explicit Euler strategy. Stability analysis of the explicit
    // Euler applied to the harmonic oscillator (the model used for these
    // compliant forces) shows the scheme to be stable for kAlpha > 2π. We take
    // a significantly larger kAlpha so that we are well within the stability
    // region of the scheme.
    // TODO(amcastro-tri): Decrease the value of kAlpha to be closer to one when
    // the time stepping scheme is updated to be implicit in the joint limits.
    const double kAlpha = 20 * M_PI;

    const Joint<T>& joint = get_joint(joint_index);
    auto revolute_joint = dynamic_cast<const RevoluteJoint<T>*>(&joint);
    auto prismatic_joint = dynamic_cast<const PrismaticJoint<T>*>(&joint);
    // Currently MBP only supports limits for prismatic and revolute joints.
    if (!(revolute_joint || prismatic_joint)) continue;

    const double penalty_time_scale = kAlpha * time_step();

    if (revolute_joint) {
      const double lower_limit = revolute_joint->position_lower_limits()(0);
      const double upper_limit = revolute_joint->position_upper_limits()(0);
      // We only compute parameters if joints do have upper/lower bounds.
      if (!std::isinf(lower_limit) || !std::isinf(upper_limit)) {
        joint_limits_parameters_.joints_with_limits.push_back(
            revolute_joint->index());

        // Store joint limits.
        joint_limits_parameters_.lower_limit.push_back(lower_limit);
        joint_limits_parameters_.upper_limit.push_back(upper_limit);
        // Estimate penalty parameters.
        auto penalty_parameters =
            internal::JointLimitsPenaltyParametersEstimator<
                T>::CalcRevoluteJointPenaltyParameters(*revolute_joint,
                                                       penalty_time_scale);
        joint_limits_parameters_.stiffness.push_back(penalty_parameters.first);
        joint_limits_parameters_.damping.push_back(penalty_parameters.second);
      }
    }

    if (prismatic_joint) {
      const double lower_limit = prismatic_joint->position_lower_limits()(0);
      const double upper_limit = prismatic_joint->position_upper_limits()(0);
      // We only compute parameters if joints do have upper/lower bounds.
      if (!std::isinf(lower_limit) || !std::isinf(upper_limit)) {
        joint_limits_parameters_.joints_with_limits.push_back(
            prismatic_joint->index());

        // Store joint limits.
        joint_limits_parameters_.lower_limit.push_back(lower_limit);
        joint_limits_parameters_.upper_limit.push_back(upper_limit);

        // Estimate penalty parameters.
        auto penalty_parameters =
            internal::JointLimitsPenaltyParametersEstimator<
                T>::CalcPrismaticJointPenaltyParameters(*prismatic_joint,
                                                        penalty_time_scale);
        joint_limits_parameters_.stiffness.push_back(penalty_parameters.first);
        joint_limits_parameters_.damping.push_back(penalty_parameters.second);
      }
    }
  }

  // Since currently MBP only handles joint limits for discrete models, we
  // verify that there are no joint limits when the model is continuous.  If
  // there are limits defined, we prepare a warning message that will be logged
  // iff the user attempts to do anything that would have needed them.
  // TODO(#24074) revisit this warning when CENIC net actuation output
  // calculations are corrected.
  if (!is_discrete() && !joint_limits_parameters_.joints_with_limits.empty()) {
    std::string joint_names_with_limits;
    for (auto joint_index : joint_limits_parameters_.joints_with_limits) {
      joint_names_with_limits +=
          fmt::format(", '{}'", get_joint(joint_index).name());
    }
    joint_names_with_limits = joint_names_with_limits.substr(2);  // Nix ", ".
    joint_limits_parameters_.pending_warning_message =
        "Currently MultibodyPlant does not handle joint limits for continuous "
        "models. However some joints do specify limits. Consider setting a "
        "non-zero time step in the MultibodyPlant constructor; this will put "
        "the plant in discrete-time mode, which does support joint limits. "
        "Joints that specify limits are: " +
        joint_names_with_limits;
  }
}

template <typename T>
void MultibodyPlant<T>::FinalizeConstraints() {
  for (auto& [constraint_id, spec] : ball_constraints_specs_) {
    if (!spec.p_BQ.has_value()) {
      // Then compute p_BQ using the default context.
      Vector3<T> p_BQ;
      auto context = this->CreateDefaultContext();
      CalcPointsPositions(*context, get_body(spec.body_A).body_frame(),
                          spec.p_AP.template cast<T>(),
                          get_body(spec.body_B).body_frame(), &p_BQ);
      spec.p_BQ = ExtractDoubleOrThrow(p_BQ);
    }
  }
}

template <typename T>
void MultibodyPlant<T>::FinalizePlantOnly() {
  DeclareInputPorts();
  DeclareParameters();
  DeclareCacheEntries();
  DeclareStateUpdate();
  DeclareOutputPorts();
  physical_models_->DeclareSystemResources();
  if (num_collision_geometries() > 0 &&
      penalty_method_contact_parameters_.time_scale < 0)
    EstimatePointContactParameters(penetration_allowance_);
  if (num_collision_geometries() > 0 &&
      friction_model_.stiction_tolerance() < 0)
    set_stiction_tolerance();
  SetUpJointLimitsParameters();
  if (use_sampled_output_ports_) {
    auto cache = std::make_unique<AccelerationKinematicsCache<T>>(
        internal_tree().forest());
    for (SpatialAcceleration<T>& A_WB : cache->get_mutable_A_WB_pool()) {
      A_WB.SetZero();
    }
    zero_acceleration_kinematics_placeholder_ = std::move(cache);
  }
  FinalizeConstraints();
  scene_graph_ = nullptr;  // must not be used after Finalize().
}

template <typename T>
MatrixX<T> MultibodyPlant<T>::MakeActuationMatrix() const {
  MatrixX<T> B = MatrixX<T>::Zero(num_velocities(), num_actuated_dofs());
  for (JointActuatorIndex actuator_index : GetJointActuatorIndices()) {
    const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
    // This method assumes actuators on single dof joints. Assert this
    // condition.
    DRAKE_DEMAND(actuator.joint().num_velocities() == 1);
    B(actuator.joint().velocity_start(), actuator.input_start()) = 1;
  }
  return B;
}

template <typename T>
Eigen::SparseMatrix<double>
MultibodyPlant<T>::MakeActuationMatrixPseudoinverse() const {
  // We leverage here the assumption that B (the actuation matrix) is a
  // permutation matrix, so Bᵀ is the pseudoinverse.
  std::vector<Eigen::Triplet<double>> triplets;
  for (JointActuatorIndex actuator_index : GetJointActuatorIndices()) {
    const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
    // This method assumes actuators on single dof joints. Assert this
    // condition.
    DRAKE_DEMAND(actuator.joint().num_velocities() == 1);
    triplets.push_back(Eigen::Triplet<double>(
        actuator.input_start(), actuator.joint().velocity_start(), 1.0));
  }

  Eigen::SparseMatrix<double> Bplus(num_actuated_dofs(), num_velocities());
  Bplus.setFromTriplets(triplets.begin(), triplets.end());
  return Bplus;
}

namespace {

void ThrowForDisconnectedGeometryPort(std::string_view explanation) {
  throw std::logic_error(
      std::string(explanation) +
      "\n\nThe provided context doesn't show a connection for the plant's "
      "query input port (see MultibodyPlant::get_geometry_query_input_port())"
      ". See https://drake.mit.edu/troubleshooting.html"
      "#mbp-unconnected-query-object-port for help.");
}

}  // namespace

template <typename T>
const geometry::QueryObject<T>& MultibodyPlant<T>::EvalGeometryQueryInput(
    const systems::Context<T>& context, std::string_view explanation) const {
  this->ValidateContext(context);
  if (!get_geometry_query_input_port().HasValue(context)) {
    ThrowForDisconnectedGeometryPort(explanation);
  }
  return get_geometry_query_input_port()
      .template Eval<geometry::QueryObject<T>>(context);
}

template <typename T>
void MultibodyPlant<T>::ValidateGeometryInput(
    const systems::Context<T>& context, std::string_view explanation) const {
  if (!IsValidGeometryInput(context)) {
    ThrowForDisconnectedGeometryPort(explanation);
  }
}

template <typename T>
void MultibodyPlant<T>::ValidateGeometryInput(
    const systems::Context<T>& context,
    const systems::OutputPort<T>& output_port) const {
  if (!IsValidGeometryInput(context)) {
    ThrowForDisconnectedGeometryPort(fmt::format(
        "You've tried evaluating MultibodyPlant's '{}' output port.",
        output_port.get_name()));
  }
}

template <typename T>
bool MultibodyPlant<T>::IsValidGeometryInput(
    const systems::Context<T>& context) const {
  return num_collision_geometries() == 0 ||
         get_geometry_query_input_port().HasValue(context);
}

template <typename T>
std::pair<T, T> MultibodyPlant<T>::GetPointContactParameters(
    geometry::GeometryId id, const SceneGraphInspector<T>& inspector) const {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  return std::pair(prop->template GetPropertyOrDefault<T>(
                       geometry::internal::kMaterialGroup,
                       geometry::internal::kPointStiffness,
                       penalty_method_contact_parameters_.geometry_stiffness),
                   prop->template GetPropertyOrDefault<T>(
                       geometry::internal::kMaterialGroup,
                       geometry::internal::kHcDissipation,
                       penalty_method_contact_parameters_.dissipation));
}

template <typename T>
const CoulombFriction<double>& MultibodyPlant<T>::GetCoulombFriction(
    geometry::GeometryId id, const SceneGraphInspector<T>& inspector) const {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  DRAKE_THROW_UNLESS(prop->HasProperty(geometry::internal::kMaterialGroup,
                                       geometry::internal::kFriction));
  return prop->GetProperty<CoulombFriction<double>>(
      geometry::internal::kMaterialGroup, geometry::internal::kFriction);
}

template <typename T>
void MultibodyPlant<T>::ApplyDefaultCollisionFilters() {
  DRAKE_DEMAND(geometry_source_is_registered());
  if (adjacent_bodies_collision_filters_) {
    // Disallow collisions between adjacent bodies. Adjacency is implied by the
    // existence of a joint between bodies, except in the case of 6-dof joints
    // or joints in which the parent body is `world`.
    for (JointIndex j : GetJointIndices()) {
      const Joint<T>& joint = get_joint(j);
      const RigidBody<T>& child = joint.child_body();
      const RigidBody<T>& parent = joint.parent_body();
      if (parent.index() == world_index()) continue;
      if (joint.type_name() == QuaternionFloatingJoint<T>::kTypeName) continue;
      std::optional<FrameId> child_id = GetBodyFrameIdIfExists(child.index());
      std::optional<FrameId> parent_id = GetBodyFrameIdIfExists(parent.index());

      if (child_id && parent_id) {
        scene_graph_->collision_filter_manager().Apply(
            CollisionFilterDeclaration(CollisionFilterScope::kOmitDeformable)
                .ExcludeBetween(geometry::GeometrySet(*child_id),
                                geometry::GeometrySet(*parent_id)));
      }
    }
  }
  // We explicitly exclude collisions within welded subgraphs.
  std::vector<std::set<BodyIndex>> subgraphs =
      internal_tree().graph().GetSubgraphsOfWeldedLinks();
  for (const auto& subgraph : subgraphs) {
    // Only operate on non-trivial weld subgraphs.
    if (subgraph.size() <= 1) {
      continue;
    }
    // Map body indices to pointers.
    std::vector<const RigidBody<T>*> subgraph_bodies;
    for (BodyIndex body_index : subgraph) {
      subgraph_bodies.push_back(&get_body(body_index));
    }
    auto geometries = CollectRegisteredGeometries(subgraph_bodies);
    scene_graph_->collision_filter_manager().Apply(
        CollisionFilterDeclaration(CollisionFilterScope::kOmitDeformable)
            .ExcludeWithin(geometries));
  }
}

template <typename T>
void MultibodyPlant<T>::ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
    const std::pair<std::string, geometry::GeometrySet>&
        collision_filter_group_a,
    const std::pair<std::string, geometry::GeometrySet>&
        collision_filter_group_b) {
  DRAKE_DEMAND(!is_finalized());
  DRAKE_DEMAND(geometry_source_is_registered());

  if (collision_filter_group_a.first == collision_filter_group_b.first) {
    scene_graph_->collision_filter_manager().Apply(
        CollisionFilterDeclaration(CollisionFilterScope::kAll)
            .ExcludeWithin(collision_filter_group_a.second));
  } else {
    scene_graph_->collision_filter_manager().Apply(
        CollisionFilterDeclaration(CollisionFilterScope::kAll)
            .ExcludeBetween(collision_filter_group_a.second,
                            collision_filter_group_b.second));
  }
}

template <typename T>
BodyIndex MultibodyPlant<T>::FindBodyByGeometryId(
    GeometryId geometry_id) const {
  if (!geometry_id.is_valid()) {
    throw std::logic_error(
        "MultibodyPlant: geometry_query input port receives contact results "
        "from SceneGraph that involve invalid GeometryId.");
  }
  const auto iter = geometry_id_to_body_index_.find(geometry_id);
  if (iter != geometry_id_to_body_index_.end()) {
    return iter->second;
  }
  throw std::logic_error(
      fmt::format("MultibodyPlant: geometry_query input port receives contact "
                  "results from SceneGraph that involve GeometryId {}, but "
                  "that ID is not known to this plant",
                  geometry_id));
}

template <typename T>
void MultibodyPlant<T>::SetDiscreteUpdateManager(
    std::unique_ptr<internal::DiscreteUpdateManager<T>> manager) {
  // N.B. This requirement is really more important on the side of the
  // manager's constructor, since most likely it'll need MBP's topology at
  // least to build the contact problem. However, here we play safe and demand
  // finalization right here.
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(is_discrete());
  DRAKE_DEMAND(manager != nullptr);
  manager->SetOwningMultibodyPlant(this);
  discrete_update_manager_ = std::move(manager);
  RemoveUnsupportedScalars(*discrete_update_manager_);
}

template <typename T>
internal::DummyPhysicalModel<T>& MultibodyPlant<T>::AddDummyModel(
    std::unique_ptr<internal::DummyPhysicalModel<T>> model) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(&model->plant() == this);
  return physical_models_->AddDummyModel(std::move(model));
}

template <typename T>
std::vector<const PhysicalModel<T>*> MultibodyPlant<T>::physical_models()
    const {
  std::vector<const PhysicalModel<T>*> result;
  for (const auto& model : physical_models_->owned_models()) {
    result.push_back(model.get());
  }
  return result;
}

template <typename T>
void MultibodyPlant<T>::set_penetration_allowance(
    double penetration_allowance) {
  if (penetration_allowance <= 0) {
    throw std::logic_error(
        "set_penetration_allowance(): penetration_allowance must be strictly "
        "positive.");
  }

  penetration_allowance_ = penetration_allowance;
  // We update the point contact parameters when this method is called
  // post-finalize.
  if (this->is_finalized())
    EstimatePointContactParameters(penetration_allowance);
}

template <typename T>
VectorX<T> MultibodyPlant<T>::GetDefaultPositions() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  VectorX<T> q;
  q.setConstant(num_positions(), std::numeric_limits<double>::quiet_NaN());
  for (JointIndex i : GetJointIndices()) {
    const Joint<T>& joint = get_joint(i);
    q.segment(joint.position_start(), joint.num_positions()) =
        joint.default_positions();
  }
  return q;
}

template <typename T>
VectorX<T> MultibodyPlant<T>::GetDefaultPositions(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  const VectorX<T> q = GetDefaultPositions();
  const VectorX<T> q_instance =
      internal_tree().GetPositionsFromArray(model_instance, q);
  return q_instance;
}

template <typename T>
void MultibodyPlant<T>::SetDefaultPositions(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(q.size() == num_positions());
  DRAKE_THROW_UNLESS(AllFinite(q));
  for (JointIndex i : GetJointIndices()) {
    Joint<T>& joint = get_mutable_joint(i);
    joint.set_default_positions(
        q.segment(joint.position_start(), joint.num_positions()));
  }
}

template <typename T>
void MultibodyPlant<T>::SetDefaultPositions(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const Eigen::VectorXd>& q_instance) {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(q_instance.size() == num_positions(model_instance));
  DRAKE_THROW_UNLESS(AllFinite(q_instance));
  VectorX<T> q_T(num_positions());
  internal_tree().SetPositionsInArray(model_instance, q_instance.cast<T>(),
                                      &q_T);
  Eigen::VectorXd q = ExtractDoubleOrThrow(q_T);
  for (JointIndex i : GetJointIndices(model_instance)) {
    Joint<T>& joint = get_mutable_joint(i);
    joint.set_default_positions(
        q.segment(joint.position_start(), joint.num_positions()));
  }
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetPositionNames(
    bool add_model_instance_prefix, bool always_add_suffix) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::vector<std::string> names(num_positions());

  for (JointIndex joint_index : GetJointIndices()) {
    const Joint<T>& joint = get_joint(joint_index);
    if (joint.num_positions() == 0) continue;  // Skip welds.

    const std::string prefix =
        add_model_instance_prefix
            ? fmt::format("{}_", GetModelInstanceName(joint.model_instance()))
            : "";
    for (int i = 0; i < joint.num_positions(); ++i) {
      const std::string suffix =
          always_add_suffix || joint.num_positions() > 1
              ? fmt::format("_{}", joint.position_suffix(i))
              : "";
      names[joint.position_start() + i] =
          fmt::format("{}{}{}", prefix, joint.name(), suffix);
    }
  }
  return names;
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetPositionNames(
    ModelInstanceIndex model_instance, bool add_model_instance_prefix,
    bool always_add_suffix) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::vector<std::string> names(num_positions(model_instance));
  std::vector<JointIndex> joint_indices = GetJointIndices(model_instance);
  // The offset into the position array is the position_start of the first
  // mobilizer in the tree; here we just take the minimum.
  int position_offset = num_positions();
  for (const auto& joint_index : joint_indices) {
    position_offset =
        std::min(position_offset, get_joint(joint_index).position_start());
  }

  for (const auto& joint_index : joint_indices) {
    const Joint<T>& joint = get_joint(joint_index);
    if (joint.num_positions() == 0) continue;  // Skip welds.

    // Sanity check: joint positions are in range.
    DRAKE_DEMAND(joint.position_start() >= position_offset);
    DRAKE_DEMAND(joint.position_start() + joint.num_positions() -
                     position_offset <=
                 ssize(names));

    const std::string prefix =
        add_model_instance_prefix
            ? fmt::format("{}_", GetModelInstanceName(model_instance))
            : "";
    for (int i = 0; i < joint.num_positions(); ++i) {
      const std::string suffix =
          always_add_suffix || joint.num_positions() > 1
              ? fmt::format("_{}", joint.position_suffix(i))
              : "";
      names[joint.position_start() + i - position_offset] =
          fmt::format("{}{}{}", prefix, joint.name(), suffix);
    }
  }
  return names;
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetVelocityNames(
    bool add_model_instance_prefix, bool always_add_suffix) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::vector<std::string> names(num_velocities());

  for (JointIndex joint_index : GetJointIndices()) {
    const Joint<T>& joint = get_joint(joint_index);
    if (joint.num_positions() == 0) continue;  // Skip welds.

    const std::string prefix =
        add_model_instance_prefix
            ? fmt::format("{}_", GetModelInstanceName(joint.model_instance()))
            : "";
    for (int i = 0; i < joint.num_velocities(); ++i) {
      const std::string suffix =
          always_add_suffix || joint.num_velocities() > 1
              ? fmt::format("_{}", joint.velocity_suffix(i))
              : "";
      names[joint.velocity_start() + i] =
          fmt::format("{}{}{}", prefix, joint.name(), suffix);
    }
  }
  return names;
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetVelocityNames(
    ModelInstanceIndex model_instance, bool add_model_instance_prefix,
    bool always_add_suffix) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::vector<std::string> names(num_velocities(model_instance));
  std::vector<JointIndex> joint_indices = GetJointIndices(model_instance);
  // The offset into the velocity array is the velocity_start of the first
  // mobilizer in the tree; here we just take the minimum.
  int velocity_offset = num_velocities();
  for (const auto& joint_index : joint_indices) {
    velocity_offset =
        std::min(velocity_offset, get_joint(joint_index).velocity_start());
  }

  for (const auto& joint_index : joint_indices) {
    const Joint<T>& joint = get_joint(joint_index);
    if (joint.num_positions() == 0) continue;  // Skip welds.

    // Sanity check: joint velocities are in range.
    DRAKE_DEMAND(joint.velocity_start() >= velocity_offset);
    DRAKE_DEMAND(joint.velocity_start() + joint.num_velocities() -
                     velocity_offset <=
                 ssize(names));

    const std::string prefix =
        add_model_instance_prefix
            ? fmt::format("{}_", GetModelInstanceName(model_instance))
            : "";
    for (int i = 0; i < joint.num_velocities(); ++i) {
      const std::string suffix =
          always_add_suffix || joint.num_velocities() > 1
              ? fmt::format("_{}", joint.velocity_suffix(i))
              : "";
      names[joint.velocity_start() + i - velocity_offset] =
          fmt::format("{}{}{}", prefix, joint.name(), suffix);
    }
  }
  return names;
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetStateNames(
    bool add_model_instance_prefix) const {
  std::vector<std::string> names =
      GetPositionNames(add_model_instance_prefix, true /* always_add_suffix */);
  std::vector<std::string> velocity_names =
      GetVelocityNames(add_model_instance_prefix, true /* always_add_suffix */);
  names.insert(names.end(), std::make_move_iterator(velocity_names.begin()),
               std::make_move_iterator(velocity_names.end()));
  return names;
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetStateNames(
    ModelInstanceIndex model_instance, bool add_model_instance_prefix) const {
  std::vector<std::string> names = GetPositionNames(
      model_instance, add_model_instance_prefix, true /* always_add_suffix */);
  std::vector<std::string> velocity_names = GetVelocityNames(
      model_instance, add_model_instance_prefix, true /* always_add_suffix */);
  names.insert(names.end(), std::make_move_iterator(velocity_names.begin()),
               std::make_move_iterator(velocity_names.end()));
  return names;
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetActuatorNames(
    bool add_model_instance_prefix) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::vector<std::string> names(num_actuators());

  for (JointActuatorIndex actuator_index : GetJointActuatorIndices()) {
    const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
    const std::string prefix =
        add_model_instance_prefix
            ? fmt::format("{}_",
                          GetModelInstanceName(actuator.model_instance()))
            : "";
    // TODO(russt): Need to add actuator name suffix to JointActuator and loop
    // over actuator.num_inputs() if we ever actually support actuators with
    // multiple inputs.
    DRAKE_DEMAND(actuator.num_inputs() == 1);
    names[actuator.input_start()] =
        fmt::format("{}{}", prefix, actuator.name());
  }
  return names;
}

template <typename T>
std::vector<std::string> MultibodyPlant<T>::GetActuatorNames(
    ModelInstanceIndex model_instance, bool add_model_instance_prefix) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::vector<std::string> names(num_actuators(model_instance));
  std::vector<JointActuatorIndex> actuator_indices =
      GetJointActuatorIndices(model_instance);
  // The offset into the actuation array is the start of the first
  // mobilizer in the tree; here we just take the minimum.
  int offset = num_actuators();
  for (const auto& actuator_index : actuator_indices) {
    offset = std::min(offset, get_joint_actuator(actuator_index).input_start());
  }

  for (const auto& actuator_index : actuator_indices) {
    const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
    // Sanity check: indices are in range.
    DRAKE_DEMAND(actuator.input_start() >= offset);
    DRAKE_DEMAND(actuator.input_start() - offset < ssize(names));

    const std::string prefix =
        add_model_instance_prefix
            ? fmt::format("{}_", GetModelInstanceName(model_instance))
            : "";
    // TODO(russt): Need to add actuator name suffix to JointActuator and loop
    // over actuator.num_inputs() if we ever actually support actuators with
    // multiple inputs.
    DRAKE_DEMAND(actuator.num_inputs() == 1);
    names[actuator.input_start() - offset] =
        fmt::format("{}{}", prefix, actuator.name());
  }
  return names;
}

template <typename T>
void MultibodyPlant<T>::EstimatePointContactParameters(
    double penetration_allowance) {
  // Default to Earth's gravity for this estimation.
  const UniformGravityFieldElement<T>& gravity = gravity_field();
  const double g = (!gravity.gravity_vector().isZero())
                       ? gravity.gravity_vector().norm()
                       : UniformGravityFieldElement<double>::kDefaultStrength;

  // TODO(amcastro-tri): Improve this heuristics in future PR's for when there
  // are several flying objects and fixed base robots (E.g.: manipulation
  // cases.)

  // The heuristic now is very simple. We should update it to:
  //  - Only scan free bodies for weight.
  //  - Consider an estimate of maximum velocities (context dependent).
  // Right now we are being very conservative and use the maximum mass in the
  // system.
  double mass = 0.0;
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    mass = std::max(mass, body.default_mass());
  }

  // For now, we use the model of a critically damped spring mass oscillator
  // to estimate these parameters: mẍ+cẋ+kx=mg
  // Notice however that normal forces are computed according to: fₙ=kx(1+dẋ)
  // which translate to a second order oscillator of the form:
  // mẍ+(kdx)ẋ+kx=mg
  // Therefore, for this more complex, non-linear, oscillator, we estimate the
  // damping constant d using a time scale related to the free oscillation
  // (omega below) and the requested penetration allowance as a length scale.

  // We first estimate the combined stiffness based on static equilibrium.
  const double combined_stiffness = mass * g / penetration_allowance;
  // Frequency associated with the combined_stiffness above.
  const double omega = sqrt(combined_stiffness / mass);

  // Estimated contact time scale. The relative velocity of objects coming into
  // contact goes to zero in this time scale.
  const double time_scale = 1.0 / omega;

  // Damping ratio for a critically damped model. We could allow users to set
  // this. Right now, critically damp the normal direction.
  // This corresponds to a non-penetraion constraint in the limit for
  // contact_penetration_allowance_ going to zero (no bounce off).
  const double damping_ratio = 1.0;
  // We form the dissipation (with units of 1/velocity) using dimensional
  // analysis. Thus we use 1/omega for the time scale and penetration_allowance
  // for the length scale. We then scale it by the damping ratio.
  const double dissipation = damping_ratio * time_scale / penetration_allowance;

  // Final parameters used in the penalty method:
  //
  // Before #13630 this method estimated an effective "combined" stiffness.
  // That is, penalty_method_contact_parameters_.geometry_stiffness (previously
  // called penalty_method_contact_parameters_.stiffness) was the desired
  // stiffness of the contact pair. Post #13630, the semantics of this variable
  // changes to "stiffness per contact geometry". Therefore, in order to
  // maintain backwards compatibility for sims run pre #13630, we include now a
  // factor of 2 so that when two geometries have the same stiffness, the
  // combined stiffness reduces to combined_stiffness.
  //
  // Stiffness in the penalty method is calculated as a combination of
  // individual stiffness parameters per geometry. The variable
  // `combined_stiffness` as calculated here is a combined stiffness, but
  // `penalty_method_contact_parameters_.geometry_stiffness` stores the
  // parameter for an individual geometry. Combined stiffness, for geometries
  // with individual stiffnesses k1 and k2 respectively, is defined as:
  //   Kc = (k1*k2) / (k1 + k2)
  // If we have a desired combined stiffness Kd (for two geometries with
  // default heuristically computed parameters), setting k1 = k2 = 2 * Kd
  // results in the correct combined stiffness:
  //   Kc = (2*Kd*2*Kd) / (2*Kd + 2*Kd) = Kd
  // Therefore we set the `geometry_stiffness` to 2*`combined_stiffness`.
  penalty_method_contact_parameters_.geometry_stiffness =
      2 * combined_stiffness;
  penalty_method_contact_parameters_.dissipation = dissipation;
  // The time scale can be requested to hint the integrator's time step.
  penalty_method_contact_parameters_.time_scale = time_scale;
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcContactResultsOutput(
    const systems::Context<T>& context, ContactResults<T>* output) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(output != nullptr);

  // Guard against failure to acquire the geometry input deep in the call graph.
  if constexpr (!sampled) {
    ValidateGeometryInput(context, get_contact_results_output_port());
  }

  // Use is_discrete() and use_sampled_output_ports() to govern the approach.
  if (this->is_discrete()) {
    if constexpr (sampled) {
      DRAKE_DEMAND(use_sampled_output_ports_);
      const DiscreteStepMemory::Data<T>* const memory =
          get_discrete_step_memory(context);
      if (memory != nullptr) {
        discrete_update_manager_->CalcContactResults(*memory, output);
      } else {
        // The plant has not been stepped yet.
        *output = ContactResults<T>{};
        output->set_plant(this);
      }
    } else {
      discrete_update_manager_->CalcContactResults(context, output);
    }
  } else {
    DRAKE_DEMAND(!sampled);
    CalcContactResultsContinuous(context, output);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsContinuous(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(contact_results != nullptr);
  DRAKE_DEMAND(!is_discrete());

  if (num_collision_geometries() == 0) {
    *contact_results = ContactResults<T>{};
    contact_results->set_plant(this);
    return;
  }

  bool use_point = false;
  bool use_hydroelastic = false;
  switch (contact_model_) {
    case ContactModel::kPoint:
      use_point = true;
      break;
    case ContactModel::kHydroelastic:
      use_hydroelastic = true;
      break;
    case ContactModel::kHydroelasticWithFallback:
      use_point = true;
      use_hydroelastic = true;
      break;
  }

  std::vector<PointPairContactInfo<T>> contact_results_point_pair;
  std::vector<HydroelasticContactInfo<T>> contact_results_hydroelastic;
  std::shared_ptr<const void> backing_store;
  if (use_point) {
    contact_results_point_pair = EvalContactResultsPointPairContinuous(context);
  }
  if (use_hydroelastic) {
    if constexpr (!std::is_same_v<T, symbolic::Expression>) {
      CalcContactResultsHydroelasticContinuous(context,
                                               &contact_results_hydroelastic);
      backing_store = EvalGeometryContactData(context).Share();
    } else {
      throw std::logic_error("This method doesn't support T = Expression");
    }
  }

  *contact_results = ContactResults<T>(std::move(contact_results_point_pair),
                                       std::move(contact_results_hydroelastic),
                                       {},  // Empty contact_results_deformable.
                                       std::move(backing_store));
  contact_results->set_plant(this);
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsHydroelasticContinuous(
    const systems::Context<T>& context,
    std::vector<HydroelasticContactInfo<T>>* contact_results_hydroelastic) const
  requires scalar_predicate<T>::is_bool
{  // NOLINT(whitespace/braces)
  this->ValidateContext(context);
  DRAKE_DEMAND(contact_results_hydroelastic != nullptr);
  DRAKE_DEMAND(!is_discrete());

  const std::vector<ContactSurface<T>>& all_surfaces =
      EvalGeometryContactData(context).get().surfaces;
  const std::vector<SpatialForce<T>>& F_Ac_W_array =
      EvalHydroelasticContactForcesContinuous(context).F_Ac_W_array;
  DRAKE_DEMAND(all_surfaces.size() == F_Ac_W_array.size());
  contact_results_hydroelastic->clear();
  contact_results_hydroelastic->reserve(all_surfaces.size());
  for (int i = 0; i < ssize(all_surfaces); ++i) {
    // Here we use emplacement to add a HydroelasticContactInfo with a raw
    // pointer back into the GeometryContactData. This is safe only because
    // our caller adds the GeometryContactData to the "backing store" as a
    // keep-alive.
    contact_results_hydroelastic->emplace_back(&all_surfaces[i],
                                               F_Ac_W_array[i]);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsPointPairContinuous(
    const systems::Context<T>& context,
    std::vector<PointPairContactInfo<T>>* contact_results_point_pair_continuous)
    const {
  this->ValidateContext(context);
  DRAKE_DEMAND(contact_results_point_pair_continuous != nullptr);
  DRAKE_DEMAND(!is_discrete());
  contact_results_point_pair_continuous->clear();

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      EvalGeometryContactData(context).get().point_pairs;

  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);
  const internal::VelocityKinematicsCache<T>& vc =
      EvalVelocityKinematics(context);

  const SceneGraphInspector<T>& inspector = EvalSceneGraphInspector(context);

  for (size_t icontact = 0; icontact < point_pairs.size(); ++icontact) {
    const auto& pair = point_pairs[icontact];
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index = FindBodyByGeometryId(geometryA_id);
    const BodyIndex bodyB_index = FindBodyByGeometryId(geometryB_id);

    internal::MobodIndex bodyA_mobod_index =
        get_body(bodyA_index).mobod_index();
    internal::MobodIndex bodyB_mobod_index =
        get_body(bodyB_index).mobod_index();

    // Penetration depth, > 0 during pair.
    const T& x = pair.depth;
    DRAKE_ASSERT(x >= 0);
    const Vector3<T>& nhat_BA_W = pair.nhat_BA_W;
    const Vector3<T>& p_WCa = pair.p_WCa;
    const Vector3<T>& p_WCb = pair.p_WCb;

    // Contact point C.
    const Vector3<T> p_WC = 0.5 * (p_WCa + p_WCb);

    // Contact point position on body A.
    const Vector3<T>& p_WAo = pc.get_X_WB(bodyA_mobod_index).translation();
    const Vector3<T>& p_CoAo_W = p_WAo - p_WC;

    // Contact point position on body B.
    const Vector3<T>& p_WBo = pc.get_X_WB(bodyB_mobod_index).translation();
    const Vector3<T>& p_CoBo_W = p_WBo - p_WC;

    // Separation velocity, > 0  if objects separate.
    const Vector3<T> v_WAc =
        vc.get_V_WB(bodyA_mobod_index).Shift(-p_CoAo_W).translational();
    const Vector3<T> v_WBc =
        vc.get_V_WB(bodyB_mobod_index).Shift(-p_CoBo_W).translational();
    const Vector3<T> v_AcBc_W = v_WBc - v_WAc;

    // if xdot = vn > 0 ==> they are getting closer.
    const T vn = v_AcBc_W.dot(nhat_BA_W);

    // Magnitude of the normal force on body A at contact point C.
    const auto [kA, dA] = GetPointContactParameters(geometryA_id, inspector);
    const auto [kB, dB] = GetPointContactParameters(geometryB_id, inspector);
    const auto [k, d] = internal::CombinePointContactParameters(kA, kB, dA, dB);
    const T fn_AC = k * x * (1.0 + d * vn);

    // Acquire friction coefficients and combine them.
    const CoulombFriction<double>& geometryA_friction =
        GetCoulombFriction(geometryA_id, inspector);
    const CoulombFriction<double>& geometryB_friction =
        GetCoulombFriction(geometryB_id, inspector);
    const CoulombFriction<double> combined_friction =
        CalcContactFrictionFromSurfaceProperties(geometryA_friction,
                                                 geometryB_friction);

    if (fn_AC > 0) {
      // Normal force on body A, at C, expressed in W.
      const Vector3<T> fn_AC_W = fn_AC * nhat_BA_W;

      // Compute tangential velocity, that is, v_AcBc projected onto the tangent
      // plane with normal nhat_BA:
      const Vector3<T> vt_AcBc_W = v_AcBc_W - vn * nhat_BA_W;
      // Tangential speed (squared):
      const T vt_squared = vt_AcBc_W.squaredNorm();

      // Consider a value indistinguishable from zero if it is smaller
      // then 1e-14 and test against that value squared.
      const T kNonZeroSqd = 1e-14 * 1e-14;
      // Tangential friction force on A at C, expressed in W.
      Vector3<T> ft_AC_W = Vector3<T>::Zero();
      T slip_velocity = 0;
      if (vt_squared > kNonZeroSqd) {
        slip_velocity = sqrt(vt_squared);
        // Stribeck friction coefficient.
        const T mu_stribeck = friction_model_.ComputeFrictionCoefficient(
            slip_velocity, combined_friction);
        // Tangential direction.
        const Vector3<T> that_W = vt_AcBc_W / slip_velocity;

        // Magnitude of the friction force on A at C.
        const T ft_AC = mu_stribeck * fn_AC;
        ft_AC_W = ft_AC * that_W;
      }

      // Spatial force on body A at C, expressed in the world frame W.
      const SpatialForce<T> F_AC_W(Vector3<T>::Zero(), fn_AC_W + ft_AC_W);

      const Vector3<T> f_Bc_W = -F_AC_W.translational();
      contact_results_point_pair_continuous->emplace_back(
          bodyA_index, bodyB_index, f_Bc_W, p_WC, vn, slip_velocity, pair);
    }
  }
}

template <typename T>
const std::vector<PointPairContactInfo<T>>&
MultibodyPlant<T>::EvalContactResultsPointPairContinuous(
    const systems::Context<T>& context) const {
  return this
      ->get_cache_entry(cache_indices_.contact_results_point_pair_continuous)
      .template Eval<std::vector<PointPairContactInfo<T>>>(context);
}

template <typename T>
void MultibodyPlant<T>::CalcAndAddPointContactForcesContinuous(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(F_BBo_W_array != nullptr);
  DRAKE_DEMAND(ssize(*F_BBo_W_array) == num_bodies());
  if (num_collision_geometries() == 0) return;

  const std::vector<PointPairContactInfo<T>>& contact_results_point_pair =
      EvalContactResultsPointPairContinuous(context);

  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);

  for (int pair_index = 0; pair_index < ssize(contact_results_point_pair);
       ++pair_index) {
    const PointPairContactInfo<T>& contact_info =
        contact_results_point_pair[pair_index];
    const PenetrationAsPointPair<T>& pair = contact_info.point_pair();

    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index = FindBodyByGeometryId(geometryA_id);
    const BodyIndex bodyB_index = FindBodyByGeometryId(geometryB_id);

    const internal::MobodIndex bodyA_mobod_index =
        get_body(bodyA_index).mobod_index();
    const internal::MobodIndex bodyB_mobod_index =
        get_body(bodyB_index).mobod_index();

    // Contact point C.
    const Vector3<T> p_WC = contact_info.contact_point();

    // Contact point position on body A.
    const Vector3<T>& p_WAo = pc.get_X_WB(bodyA_mobod_index).translation();
    const Vector3<T>& p_CoAo_W = p_WAo - p_WC;

    // Contact point position on body B.
    const Vector3<T>& p_WBo = pc.get_X_WB(bodyB_mobod_index).translation();
    const Vector3<T>& p_CoBo_W = p_WBo - p_WC;

    const Vector3<T> f_Bc_W = contact_info.contact_force();
    const SpatialForce<T> F_AC_W(Vector3<T>::Zero(), -f_Bc_W);

    if (bodyA_index != world_index()) {
      // Spatial force on body A at Ao, expressed in W.
      const SpatialForce<T> F_AAo_W = F_AC_W.Shift(p_CoAo_W);
      F_BBo_W_array->at(bodyA_mobod_index) += F_AAo_W;
    }

    if (bodyB_index != world_index()) {
      // Spatial force on body B at Bo, expressed in W.
      const SpatialForce<T> F_BBo_W = -F_AC_W.Shift(p_CoBo_W);
      F_BBo_W_array->at(bodyB_mobod_index) += F_BBo_W;
    }
  }
}

template <typename T>
void MultibodyPlant<T>::CalcHydroelasticContactForcesContinuous(
    const Context<T>& context,
    internal::HydroelasticContactForcesContinuousCacheData<T>* output) const
  requires scalar_predicate<T>::is_bool
{  // NOLINT(whitespace/braces)
  this->ValidateContext(context);
  DRAKE_DEMAND(output != nullptr);
  DRAKE_DEMAND(!is_discrete());

  std::vector<SpatialForce<T>>& F_BBo_W_array = output->F_BBo_W_array;
  std::vector<SpatialForce<T>>& F_Ac_W_array = output->F_Ac_W_array;
  DRAKE_DEMAND(ssize(F_BBo_W_array) == num_bodies());
  F_BBo_W_array.assign(num_bodies(), SpatialForce<T>::Zero());
  F_Ac_W_array.clear();

  if (num_collision_geometries() == 0) {
    return;
  }

  const std::vector<ContactSurface<T>>& all_surfaces =
      EvalGeometryContactData(context).get().surfaces;

  internal::HydroelasticTractionCalculator<T> traction_calculator(
      friction_model_.stiction_tolerance());

  const SceneGraphInspector<T>& inspector = EvalSceneGraphInspector(context);

  F_Ac_W_array.reserve(all_surfaces.size());
  for (const ContactSurface<T>& surface : all_surfaces) {
    const GeometryId geometryM_id = surface.id_M();
    const GeometryId geometryN_id = surface.id_N();

    const ProximityProperties* propM =
        inspector.GetProximityProperties(geometryM_id);
    const ProximityProperties* propN =
        inspector.GetProximityProperties(geometryN_id);
    DRAKE_DEMAND(propM != nullptr);
    DRAKE_DEMAND(propN != nullptr);
    DRAKE_THROW_UNLESS(propM->HasProperty(geometry::internal::kMaterialGroup,
                                          geometry::internal::kFriction));
    DRAKE_THROW_UNLESS(propN->HasProperty(geometry::internal::kMaterialGroup,
                                          geometry::internal::kFriction));

    const CoulombFriction<double>& geometryM_friction =
        propM->GetProperty<CoulombFriction<double>>(
            geometry::internal::kMaterialGroup, geometry::internal::kFriction);
    const CoulombFriction<double>& geometryN_friction =
        propN->GetProperty<CoulombFriction<double>>(
            geometry::internal::kMaterialGroup, geometry::internal::kFriction);

    // Compute combined friction coefficient.
    const CoulombFriction<double> combined_friction =
        CalcContactFrictionFromSurfaceProperties(geometryM_friction,
                                                 geometryN_friction);
    const double dynamic_friction = combined_friction.dynamic_friction();

    // Get the bodies that the two geometries are affixed to. We'll call these
    // A and B.
    const BodyIndex bodyA_index = FindBodyByGeometryId(geometryM_id);
    const BodyIndex bodyB_index = FindBodyByGeometryId(geometryN_id);
    const RigidBody<T>& bodyA = get_body(bodyA_index);
    const RigidBody<T>& bodyB = get_body(bodyB_index);

    // The poses and spatial velocities of bodies A and B.
    const RigidTransform<T>& X_WA = bodyA.EvalPoseInWorld(context);
    const RigidTransform<T>& X_WB = bodyB.EvalPoseInWorld(context);
    const SpatialVelocity<T>& V_WA = bodyA.EvalSpatialVelocityInWorld(context);
    const SpatialVelocity<T>& V_WB = bodyB.EvalSpatialVelocityInWorld(context);

    // Pack everything calculator needs.
    typename internal::HydroelasticTractionCalculator<T>::Data data(
        X_WA, X_WB, V_WA, V_WB, &surface);

    // Combined Hunt & Crossley dissipation.
    const hydroelastics::internal::HydroelasticEngine<T> hydroelastics_engine;
    const double dissipation = hydroelastics_engine.CalcCombinedDissipation(
        geometryM_id, geometryN_id, inspector);

    // Integrate the hydroelastic traction field over the contact surface.
    SpatialForce<T> F_Ac_W;
    traction_calculator.ComputeSpatialForcesAtCentroidFromHydroelasticModel(
        data, dissipation, dynamic_friction, &F_Ac_W);

    // Shift the traction at the centroid to tractions at the body origins.
    SpatialForce<T> F_Ao_W, F_Bo_W;
    traction_calculator.ShiftSpatialForcesAtCentroidToBodyOrigins(
        data, F_Ac_W, &F_Ao_W, &F_Bo_W);

    if (bodyA_index != world_index()) {
      F_BBo_W_array.at(bodyA.mobod_index()) += F_Ao_W;
    }

    if (bodyB_index != world_index()) {
      F_BBo_W_array.at(bodyB.mobod_index()) += F_Bo_W;
    }

    // Add the information for contact reporting.
    F_Ac_W_array.push_back(F_Ac_W);
  }
}

template <typename T>
const internal::HydroelasticContactForcesContinuousCacheData<T>&
MultibodyPlant<T>::EvalHydroelasticContactForcesContinuous(
    const systems::Context<T>& context) const
  requires scalar_predicate<T>::is_bool
{  // NOLINT(whitespace/braces)
  this->ValidateContext(context);
  DRAKE_DEMAND(!is_discrete());
  return this
      ->get_cache_entry(cache_indices_.hydroelastic_contact_forces_continuous)
      .template Eval<internal::HydroelasticContactForcesContinuousCacheData<T>>(
          context);
}

template <typename T>
void MultibodyPlant<T>::AddInForcesFromInputPorts(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  AddAppliedExternalGeneralizedForces(context, forces);
  AddAppliedExternalSpatialForces(context, forces);
  AddJointActuationForces(context, &forces->mutable_generalized_forces());
}

template <typename T>
void MultibodyPlant<T>::AddAppliedExternalGeneralizedForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  // If there are applied generalized forces, add them in.
  const InputPort<T>& applied_generalized_force_input =
      this->get_input_port(input_port_indices_.applied_generalized_force);
  if (applied_generalized_force_input.HasValue(context)) {
    const VectorX<T>& applied_generalized_force =
        applied_generalized_force_input.Eval(context);
    if (HasNaN(applied_generalized_force)) {
      throw std::runtime_error(
          "Detected NaN in applied generalized force input port.");
    }
    forces->mutable_generalized_forces() += applied_generalized_force;
  }
}

template <typename T>
void MultibodyPlant<T>::CalcGeneralizedForces(
    const systems::Context<T>& context, const MultibodyForces<T>& forces,
    VectorX<T>* generalized_forces) const {
  this->ValidateContext(context);
  DRAKE_THROW_UNLESS(forces.CheckHasRightSizeForModel(*this));
  DRAKE_THROW_UNLESS(generalized_forces != nullptr);
  generalized_forces->resize(num_velocities());
  // Heap allocate the necessary workspace.
  // TODO(amcastro-tri): Get rid of these heap allocations.
  std::vector<SpatialAcceleration<T>> A_scratch(num_bodies());
  std::vector<SpatialForce<T>> F_scratch(num_bodies());
  const VectorX<T> zero_vdot = VectorX<T>::Zero(num_velocities());
  // TODO(amcastro-tri): For performance, update this implementation to exclude
  // terms involving accelerations.
  const bool zero_velocities = true;
  internal_tree().CalcInverseDynamics(
      context, zero_vdot, forces.body_forces(), forces.generalized_forces(),
      zero_velocities, &A_scratch, &F_scratch, generalized_forces);
  *generalized_forces = -*generalized_forces;
}

template <typename T>
void MultibodyPlant<T>::AddAppliedExternalSpatialForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  // Get the mutable applied external spatial forces vector
  // (a.k.a., body force vector).
  this->ValidateContext(context);
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces->mutable_body_forces();

  // Evaluate the input port; if it's not connected, return now.
  const auto* applied_input = this->template EvalInputValue<
      std::vector<ExternallyAppliedSpatialForce<T>>>(
      context, input_port_indices_.applied_spatial_force);
  if (!applied_input) return;

  // Helper to throw a useful message if the input contains NaN.
  auto throw_if_contains_nan = [this](const ExternallyAppliedSpatialForce<T>&
                                          external_spatial_force) {
    const SpatialForce<T>& spatial_force = external_spatial_force.F_Bq_W;
    if (HasNaN(external_spatial_force.p_BoBq_B) ||
        HasNaN(spatial_force.rotational()) ||
        HasNaN(spatial_force.translational())) {
      throw std::runtime_error(fmt::format(
          "Spatial force applied on body {} contains NaN.",
          internal_tree().get_body(external_spatial_force.body_index).name()));
    }
  };
  // Loop over all forces.
  for (const auto& force_structure : *applied_input) {
    throw_if_contains_nan(force_structure);
    const BodyIndex body_index = force_structure.body_index;
    const RigidBody<T>& body = get_body(body_index);
    const auto body_mobod_index = body.mobod_index();

    // Get the pose for this body in the world frame.
    const RigidTransform<T>& X_WB = EvalBodyPoseInWorld(context, body);

    // Get the position vector from the body origin (Bo) to the point of
    // force application (Bq), expressed in the world frame (W).
    const Vector3<T> p_BoBq_W = X_WB.rotation() * force_structure.p_BoBq_B;

    // Shift the spatial force from Bq to Bo.
    F_BBo_W_array[body_mobod_index] += force_structure.F_Bq_W.Shift(-p_BoBq_W);
  }
}

template <typename T>
void MultibodyPlant<T>::AddJointActuationForces(
    const systems::Context<T>& context, VectorX<T>* forces) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->size() == num_velocities());
  if (num_actuators() > 0) {
    const VectorX<T>& u =
        EvalActuationInput(context, /* effort_limit = */ true);
    for (JointActuatorIndex actuator_index : GetJointActuatorIndices()) {
      const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
      const Joint<T>& joint = actuator.joint();
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(joint.num_velocities() == 1);
      (*forces)[joint.velocity_start()] += u[actuator.input_start()];
    }
  }
}

template <typename T>
void MultibodyPlant<T>::AddJointLimitsPenaltyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  DRAKE_THROW_UNLESS(is_discrete());
  DRAKE_DEMAND(forces != nullptr);

  auto CalcPenaltyForce = [](double lower_limit, double upper_limit,
                             double stiffness, double damping, const T& q,
                             const T& v) {
    DRAKE_DEMAND(lower_limit <= upper_limit);
    DRAKE_DEMAND(stiffness >= 0);
    DRAKE_DEMAND(damping >= 0);

    if (q > upper_limit) {
      const T delta_q = q - upper_limit;
      const T limit_force = -stiffness * delta_q - damping * v;
      using std::min;  // Needed for ADL.
      return min(limit_force, 0.);
    } else if (q < lower_limit) {
      const T delta_q = q - lower_limit;
      const T limit_force = -stiffness * delta_q - damping * v;
      using std::max;  // Needed for ADL.
      return max(limit_force, 0.);
    }
    return T(0.0);
  };

  for (size_t index = 0;
       index < joint_limits_parameters_.joints_with_limits.size(); ++index) {
    const JointIndex joint_index =
        joint_limits_parameters_.joints_with_limits[index];
    const double lower_limit = joint_limits_parameters_.lower_limit[index];
    const double upper_limit = joint_limits_parameters_.upper_limit[index];
    const double stiffness = joint_limits_parameters_.stiffness[index];
    const double damping = joint_limits_parameters_.damping[index];
    const Joint<T>& joint = get_joint(joint_index);

    const T& q = joint.GetOnePosition(context);
    const T& v = joint.GetOneVelocity(context);

    const T penalty_force =
        CalcPenaltyForce(lower_limit, upper_limit, stiffness, damping, q, v);

    joint.AddInOneForce(context, 0, penalty_force, forces);
  }
}

template <typename T>
const VectorX<T>& MultibodyPlant<T>::EvalActuationInput(
    const systems::Context<T>& context, bool effort_limit) const {
  this->ValidateContext(context);
  return this
      ->get_cache_entry(
          effort_limit ? cache_indices_.actuation_input_with_effort_limit
                       : cache_indices_.actuation_input_without_effort_limit)
      .template Eval<VectorX<T>>(context);
}

template <typename T>
void MultibodyPlant<T>::CalcActuationInputWithoutEffortLimit(
    const systems::Context<T>& context, VectorX<T>* actuation_input) const {
  this->ValidateContext(context);

  // Assemble the vector from the model instance input ports.
  // We initialize to zero. Actuation inputs are assumed to have zero values if
  // not connected.
  actuation_input->setZero();

  // Contribution from the per model-instance input ports.
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    // Ignore the port if the model instance has no actuated DoFs.
    const int instance_num_dofs = num_actuated_dofs(model_instance_index);
    if (instance_num_dofs == 0) continue;

    const auto& input_port = this->get_input_port(
        input_port_indices_.instance[model_instance_index].actuation);

    if (input_port.HasValue(context)) {
      const auto& u_instance = input_port.Eval(context);
      if (HasNaN(u_instance)) {
        throw std::runtime_error(fmt::format(
            "Actuation input port for model instance {} contains NaN.",
            GetModelInstanceName(model_instance_index)));
      }
      SetActuationInArray(model_instance_index, u_instance, actuation_input);
    }
  }

  // Contribution from the port for the full MultibodyPlant model.
  // Contributions are additive.
  const auto& actuation_port =
      this->get_input_port(input_port_indices_.actuation);
  if (actuation_port.HasValue(context)) {
    const auto& u = actuation_port.Eval(context);
    if (HasNaN(u)) {
      throw std::runtime_error(
          "Detected NaN in the actuation input port for all instances.");
    }
    // Contribution is added to the per model-instance contribution.
    *actuation_input += u;
  }
}

template <typename T>
void MultibodyPlant<T>::CalcActuationInputWithEffortLimit(
    const systems::Context<T>& context, VectorX<T>* actuation_input) const {
  // Start with the unlimited actuation.
  *actuation_input = EvalActuationInput(context, /* effort_limit = */ false);

  // Apply the limits.
  for (JointActuatorIndex actuator_index : GetJointActuatorIndices()) {
    const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
    const double e = actuator.effort_limit();
    if (std::isfinite(e)) {
      using std::max;
      using std::min;
      T& u = (*actuation_input)[actuator.input_start()];
      u = max(-e, min(e, u));
    }
  }
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcNetActuationOutput(const Context<T>& context,
                                               BasicVector<T>* output) const {
  DRAKE_DEMAND(output != nullptr);
  DRAKE_DEMAND(output->size() == num_actuated_dofs());
  // Use is_discrete() and use_sampled_output_ports() to govern the approach.
  if (is_discrete()) {
    if constexpr (sampled) {
      DRAKE_DEMAND(use_sampled_output_ports_);
      const DiscreteStepMemory::Data<T>* const memory =
          get_discrete_step_memory(context);
      if (memory != nullptr) {
        output->SetFromVector(memory->net_actuation);
      } else {
        // The plant has not been stepped yet.
        output->SetZero();
      }
    } else {
      output->SetFromVector(discrete_update_manager_->EvalActuation(context));
    }
  } else {
    DRAKE_DEMAND(!sampled);
    output->SetFromVector(
        EvalActuationInput(context, /* effort_limit = */ true));
  }
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcInstanceNetActuationOutput(
    ModelInstanceIndex model_instance, const systems::Context<T>& context,
    systems::BasicVector<T>* output) const {
  // The per-instance calc delegates to the full-model calc and then slices it.
  const VectorX<T>& net_actuation =
      get_net_actuation_output_port().Eval(context);
  output->SetFromVector(
      this->GetActuationFromArray(model_instance, net_actuation));
}

template <typename T>
const internal::DesiredStateInput<T>& MultibodyPlant<T>::EvalDesiredStateInput(
    const systems::Context<T>& context) const {
  this->ValidateContext(context);
  return this->get_cache_entry(cache_indices_.desired_state_input)
      .template Eval<internal::DesiredStateInput<T>>(context);
}

template <typename T>
void MultibodyPlant<T>::CalcDesiredStateInput(
    const systems::Context<T>& context,
    internal::DesiredStateInput<T>* desired_state_input) const {
  this->ValidateContext(context);

  // Checks if desired state x for model_instance has NaNs. Only entries
  // corresponding to PD-controlled actuators on non-locked joints are checked
  // and otherwise ignored.
  auto has_nans_unless_ignored = [&](ModelInstanceIndex model_instance,
                                     const VectorX<T>& x) -> bool {
    using std::isnan;
    const int nu = num_actuators(model_instance);
    DRAKE_DEMAND(x.size() == 2 * nu);
    const auto q = x.head(nu);
    const auto v = x.tail(nu);
    int a = 0;  // Actuator index local to its model-instance.
    for (JointActuatorIndex actuator_index :
         GetJointActuatorIndices(model_instance)) {
      const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
      const bool is_locked = actuator.joint().is_locked(context);
      if (actuator.has_controller() && !is_locked) {
        if (isnan(q[a]) || isnan(v[a])) return true;
      }
      ++a;
    }
    return false;
  };

  // Assemble the vector from the model instance input ports.
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    // Ignore the port if the model instance has no actuated DoFs.
    const int instance_num_u = num_actuated_dofs(model_instance_index);
    if (instance_num_u == 0) continue;

    const auto& xd_input_port =
        this->get_desired_state_input_port(model_instance_index);
    if (xd_input_port.HasValue(context)) {
      const auto& xd_instance = xd_input_port.Eval(context);
      if (has_nans_unless_ignored(model_instance_index, xd_instance)) {
        throw std::runtime_error(
            fmt::format("Desired state input port for model "
                        "instance {} contains NaN.",
                        GetModelInstanceName(model_instance_index)));
      }
      const auto qd = xd_instance.head(instance_num_u);
      const auto vd = xd_instance.tail(instance_num_u);
      desired_state_input->SetModelInstanceDesiredStates(model_instance_index,
                                                         qd, vd);
    } else {
      desired_state_input->ClearModelInstanceDesiredStates(
          model_instance_index);
    }
  }
}

template <typename T>
void MultibodyPlant<T>::CalcGeometryContactData(
    const drake::systems::Context<T>& context,
    GeometryContactData<T>* result) const {
  this->ValidateContext(context);

  // Bail out early when there is not any proximity geometry.
  if (num_collision_geometries() == 0) {
    result->Clear();
    return;
  }

  // Replace the result with newly-allocated (empty) storage, keeping around a
  // mutable reference for us to fill in. After we return, the geometry contact
  // data is forevermore immutable.
  NestedGeometryContactData<T>& storage = result->Allocate();

  // Add all of the contacts to `result`.
  const auto& query_object = EvalGeometryQueryInput(context, __func__);
  switch (contact_model_) {
    case ContactModel::kPoint: {
      storage.point_pairs = query_object.ComputePointPairPenetration();
      break;
    }
    case ContactModel::kHydroelastic: {
      if constexpr (scalar_predicate<T>::is_bool) {
        storage.surfaces = query_object.ComputeContactSurfaces(
            get_contact_surface_representation());
        break;
      } else {
        // TODO(SeanCurtis-TRI): Special case the QueryObject scalar support
        //  such that it works as long as there are no collisions.
        throw std::logic_error(
            "MultibodyPlant::CalcGeometryContactData(): This method doesn't "
            "support T=Expression once collision geometries have been added.");
      }
    }
    case ContactModel::kHydroelasticWithFallback: {
      if constexpr (scalar_predicate<T>::is_bool) {
        query_object.ComputeContactSurfacesWithFallback(
            get_contact_surface_representation(), &storage.surfaces,
            &storage.point_pairs);
        break;
      } else {
        // TODO(SeanCurtis-TRI): Special case the QueryObject scalar support
        //  such that it works as long as there are no collisions.
        throw std::logic_error(
            "MultibodyPlant::CalcGeometryContactData(): This method doesn't "
            "support T=Expression once collision geometries have been added.");
      }
    }
  }
  if constexpr (std::is_same_v<T, double>) {
    if (is_discrete()) {
      const auto* deformable_driver =
          discrete_update_manager_->deformable_driver();
      if (deformable_driver != nullptr) {
        deformable_driver->CalcDeformableContact(query_object,
                                                 &storage.deformable);
      }
    }
  }

  // Filter out irrelevant contacts due to joint locking, i.e., between bodies
  // that belong to trees with 0 degrees of freedom. For a contact to remain in
  // consideration, at least one of the trees involved has to be valid and have
  // a non-zero number of DOFs. (Ideally, we would tell SceneGraph to omit
  // these in the first place and then we can get rid of the following code.)
  const internal::JointLockingCacheData<T>& joint_locking =
      EvalJointLocking(context);
  if (joint_locking.locked_velocity_indices.empty()) {
    return;
  }
  const auto& geometry_id_to_body_index = geometry_id_to_body_index_;
  const internal::SpanningForest& forest = internal_tree().forest();
  const std::vector<std::vector<int>>& per_tree_unlocked_indices =
      joint_locking.unlocked_velocity_indices_per_tree;
  const auto is_irrelevant_geometry =
      [&geometry_id_to_body_index, &forest,
       &per_tree_unlocked_indices](GeometryId geometry_id) {
        // Checks whether `geometry_id` belongs to a zero-dof tree.
        const BodyIndex body_index = geometry_id_to_body_index.at(geometry_id);
        const internal::TreeIndex tree_index =
            forest.link_to_tree_index(body_index);
        const internal::SpanningForest::Tree& tree = forest.trees(tree_index);
        return !tree.has_dofs() ||
               per_tree_unlocked_indices[tree_index].size() == 0;
      };
  const auto is_irrelevant_point_pair =
      [&is_irrelevant_geometry](const PenetrationAsPointPair<T>& point_pair) {
        // Checks whether `point_pair` refers to only zero-dof trees.
        return is_irrelevant_geometry(point_pair.id_A) &&
               is_irrelevant_geometry(point_pair.id_B);
      };
  std::erase_if(storage.point_pairs, is_irrelevant_point_pair);
  if constexpr (scalar_predicate<T>::is_bool) {
    const auto is_irrelevant_surface =
        [&is_irrelevant_geometry](const ContactSurface<T>& surface) {
          // Checks whether `surface` refers to only zero-dof trees.
          return is_irrelevant_geometry(surface.id_M()) &&
                 is_irrelevant_geometry(surface.id_N());
        };
    std::erase_if(storage.surfaces, is_irrelevant_surface);
  }
  // TODO(jwnimmer-tri) Filter out irrelevant deformable contact, too.
}

template <typename T>
const GeometryContactData<T>& MultibodyPlant<T>::EvalGeometryContactData(
    const Context<T>& context) const {
  return this->get_cache_entry(cache_indices_.geometry_contact_data)
      .template Eval<GeometryContactData<T>>(context);
}

template <typename T>
void MultibodyPlant<T>::CalcJointLocking(
    const systems::Context<T>& context,
    internal::JointLockingCacheData<T>* data) const {
  DRAKE_DEMAND(data != nullptr);

  const internal::SpanningForest& forest = internal_tree().forest();

  auto& unlocked = data->unlocked_velocity_indices;
  auto& locked = data->locked_velocity_indices;
  auto& unlocked_per_tree = data->unlocked_velocity_indices_per_tree;
  auto& locked_per_tree = data->locked_velocity_indices_per_tree;

  unlocked_per_tree.clear();
  locked_per_tree.clear();
  unlocked.resize(num_velocities());
  locked.resize(num_velocities());
  unlocked_per_tree.resize(forest.num_trees());
  locked_per_tree.resize(forest.num_trees());

  int unlocked_cursor = 0;
  int locked_cursor = 0;
  for (JointIndex joint_index : GetJointIndices()) {
    const Joint<T>& joint = get_joint(joint_index);
    if (joint.is_locked(context)) {
      for (int k = 0; k < joint.num_velocities(); ++k) {
        locked[locked_cursor++] = joint.velocity_start() + k;
      }
    } else {
      for (int k = 0; k < joint.num_velocities(); ++k) {
        unlocked[unlocked_cursor++] = joint.velocity_start() + k;
      }
    }
  }

  DRAKE_ASSERT(unlocked_cursor <= num_velocities());
  DRAKE_ASSERT(locked_cursor <= num_velocities());
  DRAKE_ASSERT(unlocked_cursor + locked_cursor == num_velocities());

  // Use size to indicate exactly how many velocities are locked/unlocked.
  unlocked.resize(unlocked_cursor);
  locked.resize(locked_cursor);

  // Sort the locked/unlocked indices to keep the original DOF ordering
  // established by the plant stable.
  std::sort(unlocked.begin(), unlocked.end());
  internal::DemandIndicesValid(unlocked, num_velocities());
  std::sort(locked.begin(), locked.end());
  internal::DemandIndicesValid(locked, num_velocities());

  for (int dof : unlocked) {
    const internal::TreeIndex tree_index = forest.v_to_tree_index(dof);
    const internal::SpanningForest::Tree& tree = forest.trees(tree_index);
    const int tree_dof = dof - tree.v_start();
    unlocked_per_tree[tree_index].push_back(tree_dof);
  }

  for (int dof : locked) {
    const internal::TreeIndex tree_index = forest.v_to_tree_index(dof);
    const internal::SpanningForest::Tree& tree = forest.trees(tree_index);
    const int tree_dof = dof - tree.v_start();
    locked_per_tree[tree_index].push_back(tree_dof);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcGeneralizedContactForcesContinuous(
    const Context<T>& context, VectorX<T>* tau_contact) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(tau_contact != nullptr);
  DRAKE_DEMAND(tau_contact->size() == num_velocities());
  DRAKE_DEMAND(!is_discrete());

  // Early exit if there are no contact forces.
  tau_contact->setZero();
  if (num_collision_geometries() == 0) return;

  // We will alias this zero vector to serve both as zero-valued generalized
  // accelerations and zero-valued externally applied generalized forces.
  const int nv = this->num_velocities();
  const VectorX<T> zero = VectorX<T>::Zero(nv);
  const VectorX<T>& zero_vdot = zero;
  const VectorX<T>& tau_array = zero;

  // Get the spatial forces.
  const std::vector<SpatialForce<T>>& Fcontact_BBo_W_array =
      EvalSpatialContactForcesContinuous(context);

  // Bodies' accelerations and inboard mobilizer reaction forces, respectively,
  // ordered by MobodIndex and required as output arguments for
  // CalcInverseDynamics() below but otherwise not used by this method.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_bodies());

  // With vdot = 0, this computes:
  //   tau_contact = - ∑ J_WBᵀ(q) Fcontact_Bo_W.
  internal_tree().CalcInverseDynamics(
      context, zero_vdot, Fcontact_BBo_W_array, tau_array,
      true /* Do not compute velocity-dependent terms */, &A_WB_array,
      &F_BMo_W_array, tau_contact);

  // Per above, tau_contact must be negated to get ∑ J_WBᵀ(q) Fcontact_Bo_W.
  (*tau_contact) = -(*tau_contact);
}

template <typename T>
const VectorX<T>& MultibodyPlant<T>::EvalGeneralizedContactForcesContinuous(
    const systems::Context<T>& context) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(!is_discrete());
  return this
      ->get_cache_entry(cache_indices_.generalized_contact_forces_continuous)
      .template Eval<VectorX<T>>(context);
}

template <typename T>
void MultibodyPlant<T>::CalcSpatialContactForcesContinuous(
    const drake::systems::Context<T>& context,
    std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(F_BBo_W_array != nullptr);
  DRAKE_DEMAND(ssize(*F_BBo_W_array) == num_bodies());
  DRAKE_DEMAND(!is_discrete());

  // Forces can accumulate into F_BBo_W_array; initialize it to zero first.
  std::fill(F_BBo_W_array->begin(), F_BBo_W_array->end(),
            SpatialForce<T>::Zero());

  CalcAndAddSpatialContactForcesContinuous(context, F_BBo_W_array);
}

template <typename T>
const std::vector<SpatialForce<T>>&
MultibodyPlant<T>::EvalSpatialContactForcesContinuous(
    const systems::Context<T>& context) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(!is_discrete());
  return this->get_cache_entry(cache_indices_.spatial_contact_forces_continuous)
      .template Eval<std::vector<SpatialForce<T>>>(context);
}

template <typename T>
void MultibodyPlant<T>::CalcAndAddSpatialContactForcesContinuous(
    const drake::systems::Context<T>& context,
    std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(F_BBo_W_array != nullptr);
  DRAKE_DEMAND(ssize(*F_BBo_W_array) == num_bodies());
  DRAKE_DEMAND(!is_discrete());

  // Early exit if there are no contact forces.
  if (num_collision_geometries() == 0) return;

  if constexpr (std::is_same_v<T, symbolic::Expression>) {
    throw std::logic_error("This method doesn't support T = Expression");
  } else {
    // Note: we don't need to know the applied forces here because we use a
    // regularized friction model whose forces depend only on the current state;
    // a constraint based friction model would require accounting for the
    // applied forces.

    // Compute the spatial forces on each body from contact.
    switch (contact_model_) {
      case ContactModel::kPoint:
        // Note: consider caching the results from the following method (in
        // which case we would also want to introduce the Eval... naming
        // convention for the method).
        CalcAndAddPointContactForcesContinuous(context, &(*F_BBo_W_array));
        break;

      case ContactModel::kHydroelastic:
        *F_BBo_W_array =
            EvalHydroelasticContactForcesContinuous(context).F_BBo_W_array;
        break;

      case ContactModel::kHydroelasticWithFallback:
        // Combine the point-penalty forces with the contact surface forces.
        CalcAndAddPointContactForcesContinuous(context, &(*F_BBo_W_array));
        const std::vector<SpatialForce<T>>& Fhydro_BBo_W_all =
            EvalHydroelasticContactForcesContinuous(context).F_BBo_W_array;
        DRAKE_DEMAND(F_BBo_W_array->size() == Fhydro_BBo_W_all.size());
        for (int i = 0; i < ssize(Fhydro_BBo_W_all); ++i) {
          // Both sets of forces are applied to the body's origins and expressed
          // in frame W. They should simply sum.
          (*F_BBo_W_array)[i] += Fhydro_BBo_W_all[i];
        }
        break;
    }
  }
}

template <typename T>
void MultibodyPlant<T>::ThrowIfUnsupportedContinuousTimeDynamics(
    const systems::Context<T>&) const {
  // Reject constraints.
  // TODO(#23759,#23760,#23762,#23763,#23992): revisit this check and error
  // message as constraints are implemented for CENIC.
  if (num_constraints() > 0) {
    throw std::logic_error(
        "Currently this MultibodyPlant is set to use continuous time. "
        "Continuous time does not support constraints. Use a discrete time "
        "model and set_discrete_contact_approximation() to set a model "
        "approximation that uses the SAP solver instead (kSap, kSimilar, or "
        "kLagged).");
  }

  // TODO(#24061): consider rejecting models with joint limits here, once CENIC
  // has correct behavior for joint limits, and errors/warnings are rearranged.
}

template <typename T>
void MultibodyPlant<T>::CalcNonContactForcesContinuous(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(*this));
  DRAKE_DEMAND(!is_discrete());

  // Compute forces applied through force elements. Note that this resets
  // forces to empty so must come first.
  CalcForceElementsContribution(context, forces);
  AddInForcesFromInputPorts(context, forces);
  // Only discrete models support joint limits. We log a warning if joint
  // limits are set.
  auto& warning = joint_limits_parameters_.pending_warning_message;
  if (!warning.empty()) {
    drake::log()->warn(warning);
    warning.clear();
  }
}

template <typename T>
void MultibodyPlant<T>::AddInForcesContinuous(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(!is_discrete());
  ThrowIfUnsupportedContinuousTimeDynamics(context);

  // Guard against failure to acquire the geometry input deep in the call graph.
  ValidateGeometryInput(
      context, "You've tried evaluating time derivatives or their residuals.");

  // Forces from MultibodyTree elements are handled in MultibodyTreeSystem;
  // we need only handle MultibodyPlant-specific forces here.
  AddInForcesFromInputPorts(context, forces);

  // Add the contribution of contact forces.
  std::vector<SpatialForce<T>>& Fapp_BBo_W_array =
      forces->mutable_body_forces();
  const std::vector<SpatialForce<T>>& Fcontact_BBo_W_array =
      EvalSpatialContactForcesContinuous(context);
  for (int i = 0; i < ssize(Fapp_BBo_W_array); ++i)
    Fapp_BBo_W_array[i] += Fcontact_BBo_W_array[i];
}

template <typename T>
void MultibodyPlant<T>::DoCalcForwardDynamicsDiscrete(
    const drake::systems::Context<T>& context0,
    AccelerationKinematicsCache<T>* ac) const {
  this->ValidateContext(context0);
  DRAKE_DEMAND(ac != nullptr);
  DRAKE_DEMAND(is_discrete());

  // Guard against failure to acquire the geometry input deep in the call graph.
  ValidateGeometryInput(context0,
                        "You've tried evaluating discrete forward dynamics.");

  DRAKE_DEMAND(discrete_update_manager_ != nullptr);
  discrete_update_manager_->CalcAccelerationKinematicsCache(context0, ac);
}

template <typename T>
systems::EventStatus MultibodyPlant<T>::CalcStepDiscrete(
    const systems::Context<T>& context0,
    systems::DiscreteValues<T>* next_discrete_state) const {
  this->ValidateContext(context0);
  discrete_update_manager_->CalcDiscreteValues(context0, next_discrete_state);
  return systems::EventStatus::Succeeded();
}

template <typename T>
systems::EventStatus MultibodyPlant<T>::CalcStepUnrestricted(
    const systems::Context<T>& context0, systems::State<T>* next_state) const {
  this->ValidateContext(context0);
  systems::DiscreteValues<T>& next_discrete_state =
      next_state->get_mutable_discrete_state();
  DiscreteStepMemory::Data<T>& next_memory =
      next_state->template get_mutable_abstract_state<DiscreteStepMemory>(0)
          .template Allocate<T>(internal_tree().forest());
  discrete_update_manager_->CalcDiscreteValues(context0, &next_discrete_state,
                                               &next_memory);
  next_memory.reaction_forces.resize(num_joints());
  CalcReactionForces(context0, &next_memory.reaction_forces);
  return systems::EventStatus::Succeeded();
}

template <typename T>
template <bool sampled>
const AccelerationKinematicsCache<T>&
MultibodyPlant<T>::EvalAccelerationKinematicsCacheForOutputPortCalc(
    const systems::Context<T>& context) const {
  if constexpr (sampled) {
    DRAKE_DEMAND(is_discrete());
    DRAKE_DEMAND(use_sampled_output_ports_);
    const DiscreteStepMemory::Data<T>* const memory =
        get_discrete_step_memory(context);
    if (memory == nullptr) {
      DRAKE_DEMAND(zero_acceleration_kinematics_placeholder_ != nullptr);
      return *zero_acceleration_kinematics_placeholder_;
    }
    return memory->acceleration_kinematics_cache;
  } else {
    return this->EvalForwardDynamics(context);
  }
}

template <typename T>
template <typename ModelValue, typename CalcFunction,
          typename MaybeModelInstanceIndex>
OutputPortIndex MultibodyPlant<T>::DeclareSampledOutputPort(
    const std::string& name, const ModelValue& model_value,
    const CalcFunction& calc_sampled, const CalcFunction& calc_unsampled,
    const std::set<DependencyTicket>& prerequisites_of_unsampled,
    MaybeModelInstanceIndex model_instance) {
  constexpr bool is_vector_port = std::is_same_v<ModelValue, int>;
  OutputPortIndex result;
  if constexpr (std::is_same_v<MaybeModelInstanceIndex, ModelInstanceIndex>) {
    // We need to pass a model instance into the callback, so we'll use a
    // callable helper struct to do that. (We avoid using a lambda, because
    // it doesn't play nice with the ternary-conditional operator, below when
    // initializing selected_calc.)
    using Output =
        std::conditional_t<is_vector_port, BasicVector<T>, ModelValue>;
    struct InstanceCall {
      void operator()(const Context<T>& context, Output* output) const {
        (plant->*calc)(model_instance, context, output);
      }
      const MultibodyPlant<T>* plant{};
      CalcFunction calc{};
      ModelInstanceIndex model_instance;
    };
    // Use a self-call without the model_instance, passing InstanceCall functors
    // instead of member functions.
    result = DeclareSampledOutputPort(
        name, model_value, InstanceCall{this, calc_sampled, model_instance},
        InstanceCall{this, calc_unsampled, model_instance},
        prerequisites_of_unsampled);
  } else {
    // The use_sampled_output_ports_ governs whether the port is sampled or not.
    const auto& selected_calc =
        use_sampled_output_ports_ ? calc_sampled : calc_unsampled;
    const auto& selected_prerequisites =
        use_sampled_output_ports_
            ? std::set<DependencyTicket>(
                  {this->abstract_state_ticket(systems::AbstractStateIndex{0})})
            : prerequisites_of_unsampled;
    if constexpr (is_vector_port) {
      const int size = model_value;
      result = this->DeclareVectorOutputPort(name, size, selected_calc,
                                             selected_prerequisites)
                   .get_index();
    } else {
      result = this->DeclareAbstractOutputPort(name, model_value, selected_calc,
                                               selected_prerequisites)
                   .get_index();
    }
  }
  return result;
}

template <typename T>
void MultibodyPlant<T>::DeclareInputPorts() {
  // Input "actuation".
  input_port_indices_.actuation =
      this->DeclareVectorInputPort("actuation", num_actuated_dofs())
          .get_index();

  // Input "applied_generalized_force".
  input_port_indices_.applied_generalized_force =
      this->DeclareVectorInputPort("applied_generalized_force",
                                   num_velocities())
          .get_index();

  // Input "applied_spatial_force".
  input_port_indices_.applied_spatial_force =
      this->DeclareAbstractInputPort(
              "applied_spatial_force",
              Value<std::vector<ExternallyAppliedSpatialForce<T>>>())
          .get_index();

  // Loop over model instances.
  input_port_indices_.instance.resize(num_model_instances());
  for (ModelInstanceIndex i(0); i < num_model_instances(); ++i) {
    const std::string& model_instance_name = GetModelInstanceName(i);

    // Input "{model_instance_name}_actuation".
    input_port_indices_.instance[i].actuation =
        this->DeclareVectorInputPort(
                fmt::format("{}_actuation", model_instance_name),
                num_actuated_dofs(i))
            .get_index();

    // Input "{model_instance_name}_actuation".
    // Actuators can only be defined on single-dof joints. Therefore the number
    // of desired states per instance is twice the number of actuators.
    const int instance_num_xd = 2 * num_actuators(i);
    input_port_indices_.instance[i].desired_state =
        this->DeclareVectorInputPort(
                fmt::format("{}_desired_state", model_instance_name),
                instance_num_xd)
            .get_index();
  }
}

template <typename T>
void MultibodyPlant<T>::DeclareStateUpdate() {
  // The model must be finalized.
  DRAKE_DEMAND(this->is_finalized());
  if (is_discrete()) {
    // Declare our periodic update step, and also permit triggering a step via
    // a Forced update. For output port sampling, we also need additional State.
    if (use_sampled_output_ports_) {
      this->DeclareAbstractState(Value<DiscreteStepMemory>{});
      this->DeclarePeriodicUnrestrictedUpdateEvent(
          time_step_, 0.0, &MultibodyPlant<T>::CalcStepUnrestricted);
      this->DeclareForcedUnrestrictedUpdateEvent(
          &MultibodyPlant<T>::CalcStepUnrestricted);
    } else {
      this->DeclarePeriodicDiscreteUpdateEvent(
          time_step_, 0.0, &MultibodyPlant<T>::CalcStepDiscrete);
      this->DeclareForcedDiscreteUpdateEvent(
          &MultibodyPlant<T>::CalcStepDiscrete);
    }
  }
}

template <typename T>
void MultibodyPlant<T>::DeclareOutputPorts() {
  // Note that the "state" ticket here is just the kinematics.
  // It does not include the abstract DiscreteStepMemory state (if any).
  const DependencyTicket state_ticket =
      is_discrete() ? this->xd_ticket() : this->kinematics_ticket();
  const DependencyTicket position_ticket =
      is_discrete() ? this->xd_ticket() : this->q_ticket();

  // Output "state".
  output_port_indices_.state =
      this->DeclareVectorOutputPort("state", num_multibody_states(),
                                    &MultibodyPlant::CalcStateOutput,
                                    {state_ticket})
          .get_index();

  // Output "body_poses".
  output_port_indices_.body_poses =
      this->DeclareAbstractOutputPort(
              "body_poses", std::vector<math::RigidTransform<T>>(num_bodies()),
              &MultibodyPlant<T>::CalcBodyPosesOutput,
              {position_ticket, this->all_parameters_ticket()})
          .get_index();

  // Output "body_spatial_velocities".
  output_port_indices_.body_spatial_velocities =
      this->DeclareAbstractOutputPort(
              "body_spatial_velocities",
              std::vector<SpatialVelocity<T>>(num_bodies()),
              &MultibodyPlant<T>::CalcBodySpatialVelocitiesOutput,
              {state_ticket, this->all_parameters_ticket()})
          .get_index();

  // Output "body_spatial_accelerations".
  output_port_indices_.body_spatial_accelerations = DeclareSampledOutputPort(
      "body_spatial_accelerations",
      std::vector<SpatialAcceleration<T>>(num_bodies()),
      &MultibodyPlant<T>::CalcBodySpatialAccelerationsOutput<true>,
      &MultibodyPlant<T>::CalcBodySpatialAccelerationsOutput<false>,
      {this->acceleration_kinematics_cache_entry().ticket()});

  // Output "generalized_acceleration".
  output_port_indices_.generalized_acceleration = DeclareSampledOutputPort(
      "generalized_acceleration", num_velocities(),
      &MultibodyPlant<T>::CalcGeneralizedAccelerationOutput<true>,
      &MultibodyPlant<T>::CalcGeneralizedAccelerationOutput<false>,
      {this->acceleration_kinematics_cache_entry().ticket()});

  // Output "net_actuation".
  // N.B. We intentionally declare a dependency on kinematics in the continuous
  // mode in anticipation for adding PD support to it.
  output_port_indices_.net_actuation = DeclareSampledOutputPort(
      "net_actuation", num_actuated_dofs(),
      &MultibodyPlant<T>::CalcNetActuationOutput<true>,
      &MultibodyPlant<T>::CalcNetActuationOutput<false>,
      {state_ticket, this->all_input_ports_ticket(),
       this->all_parameters_ticket()});

  // Output "reaction_forces".
  output_port_indices_.reaction_forces = DeclareSampledOutputPort(
      "reaction_forces", std::vector<SpatialForce<T>>(num_joints()),
      &MultibodyPlant<T>::CalcReactionForcesOutput<true>,
      &MultibodyPlant<T>::CalcReactionForcesOutput<false>,
      {this->acceleration_kinematics_cache_entry().ticket()});

  // Output port "contact_results".
  output_port_indices_.contact_results = DeclareSampledOutputPort(
      "contact_results", ContactResults<T>(),
      &MultibodyPlant<T>::CalcContactResultsOutput<true>,
      &MultibodyPlant<T>::CalcContactResultsOutput<false>,
      {this->acceleration_kinematics_cache_entry().ticket()});

  // Loop over model instances.
  output_port_indices_.instance.resize(num_model_instances());
  for (ModelInstanceIndex i(0); i < num_model_instances(); ++i) {
    const std::string& model_instance_name = GetModelInstanceName(i);

    // Output "{model_instance_name}_state".
    output_port_indices_.instance[i].state =
        this->DeclareVectorOutputPort(
                fmt::format("{}_state", model_instance_name),
                num_multibody_states(i),
                [this, i](const Context<T>& context, BasicVector<T>* output) {
                  this->CalcInstanceStateOutput(i, context, output);
                },
                {state_ticket})
            .get_index();

    // Output "{model_instance_name}_generalized_acceleration".
    output_port_indices_.instance[i]
        .generalized_acceleration = this->DeclareSampledOutputPort(
        fmt::format("{}_generalized_acceleration", model_instance_name),
        num_velocities(i),
        &MultibodyPlant<T>::CalcInstanceGeneralizedAccelerationOutput<true>,
        &MultibodyPlant<T>::CalcInstanceGeneralizedAccelerationOutput<false>,
        {this->acceleration_kinematics_cache_entry().ticket()}, i);

    // Output "{model_instance_name}_generalized_contact_forces".
    output_port_indices_.instance[i]
        .generalized_contact_forces = this->DeclareSampledOutputPort(
        fmt::format("{}_generalized_contact_forces", model_instance_name),
        num_velocities(i),
        &MultibodyPlant<T>::CalcInstanceGeneralizedContactForcesOutput<true>,
        &MultibodyPlant<T>::CalcInstanceGeneralizedContactForcesOutput<false>,
        is_discrete()
            ? std::set<DependencyTicket>(
                  {this->acceleration_kinematics_cache_entry().ticket()})
            : std::set<DependencyTicket>(
                  {this->get_cache_entry(
                           cache_indices_.generalized_contact_forces_continuous)
                       .ticket()}),
        i);

    // Output "{model_instance_name}_net_actuation".
    // N.B. We intentionally declare a dependency on kinematics in the
    // continuous mode in anticipation for adding PD support to it.
    output_port_indices_.instance[i].net_actuation = DeclareSampledOutputPort(
        fmt::format("{}_net_actuation", model_instance_name),
        num_actuated_dofs(i),
        &MultibodyPlant<T>::CalcInstanceNetActuationOutput<true>,
        &MultibodyPlant<T>::CalcInstanceNetActuationOutput<false>,
        {state_ticket, this->all_input_ports_ticket(),
         this->all_parameters_ticket()},
        i);
  }
}

template <typename T>
void MultibodyPlant<T>::DeclareCacheEntries() {
  DRAKE_DEMAND(this->is_finalized());

  // TODO(joemasterjohn): Create more granular parameter tickets for finer
  // control over cache dependencies on parameters. For example,
  // all_rigid_body_parameters, etc.
  const DependencyTicket state_ticket =
      is_discrete() ? this->xd_ticket() : this->kinematics_ticket();
  const DependencyTicket position_ticket =
      is_discrete() ? this->xd_ticket() : this->q_ticket();

  // TODO(SeanCurtis-TRI): When SG caches the results of these queries itself,
  //  (https://github.com/RobotLocomotion/drake/issues/12767), remove this
  //  cache entry.
  auto& geometry_contact_data_cache_entry = this->DeclareCacheEntry(
      std::string("GeometryContactData"),
      // We can't just depend on configuration ticket here because the results
      // from query objects also depend on other things in GeometryState such
      // as collision filters.
      &MultibodyPlant::CalcGeometryContactData,
      {position_ticket, this->all_parameters_ticket(),
       get_geometry_query_input_port().ticket()});
  cache_indices_.geometry_contact_data =
      geometry_contact_data_cache_entry.cache_index();

  // Cache entry for HydroelasticContactForcesContinuous.
  const bool use_hydroelastic =
      contact_model_ == ContactModel::kHydroelastic ||
      contact_model_ == ContactModel::kHydroelasticWithFallback;
  if constexpr (!std::is_same_v<T, symbolic::Expression>) {
    if (!is_discrete() && use_hydroelastic) {
      auto& hydroelastic_contact_forces_continuous_cache_entry =
          this->DeclareCacheEntry(
              std::string("HydroelasticContactForcesContinuous"),
              internal::HydroelasticContactForcesContinuousCacheData<T>(
                  this->num_bodies()),
              &MultibodyPlant<T>::CalcHydroelasticContactForcesContinuous,
              // Compliant contact forces due to hydroelastics with Hunt &
              // Crossley are function of the kinematic variables q & v only.
              {state_ticket, this->all_parameters_ticket(),
               get_geometry_query_input_port().ticket()});
      cache_indices_.hydroelastic_contact_forces_continuous =
          hydroelastic_contact_forces_continuous_cache_entry.cache_index();
    }
  }

  // Cache entry for ContactResultsPointPairContinuous.
  if (!is_discrete()) {
    auto& contact_results_point_pair_continuous_cache_entry =
        this->DeclareCacheEntry(
            std::string("ContactResultsPointPairContinuous"),
            &MultibodyPlant<T>::CalcContactResultsPointPairContinuous,
            {state_ticket, this->all_parameters_ticket(),
             get_geometry_query_input_port().ticket()});
    cache_indices_.contact_results_point_pair_continuous =
        contact_results_point_pair_continuous_cache_entry.cache_index();
  }

  // Cache entry for SpatialContactForcesContinuous.
  if (!is_discrete()) {
    auto& spatial_contact_forces_continuous_cache_entry =
        this->DeclareCacheEntry(
            "SpatialContactForcesContinuous",
            std::vector<SpatialForce<T>>(num_bodies()),
            &MultibodyPlant::CalcSpatialContactForcesContinuous,
            {state_ticket, this->all_parameters_ticket(),
             get_geometry_query_input_port().ticket()});
    cache_indices_.spatial_contact_forces_continuous =
        spatial_contact_forces_continuous_cache_entry.cache_index();
  }

  // Cache entry for GeneralizedContactForcesContinuous.
  if (!is_discrete()) {
    auto& generalized_contact_forces_continuous_cache_entry =
        this->DeclareCacheEntry(
            "GeneralizedContactForcesContinuous", VectorX<T>(num_velocities()),
            &MultibodyPlant::CalcGeneralizedContactForcesContinuous,
            {this->cache_entry_ticket(
                 cache_indices_.spatial_contact_forces_continuous),
             this->all_parameters_ticket()});
    cache_indices_.generalized_contact_forces_continuous =
        generalized_contact_forces_continuous_cache_entry.cache_index();
  }

  // Cache joint locking data. A joint's locked/unlocked state is stored as an
  // abstract parameter, so this is the only dependency needed.
  const auto& joint_locking_cache_entry = this->DeclareCacheEntry(
      "JointLocking", internal::JointLockingCacheData<T>{},
      &MultibodyPlant::CalcJointLocking, {this->all_parameters_ticket()});
  cache_indices_.joint_locking = joint_locking_cache_entry.cache_index();

  // Cache actuation input data (without effort limit).
  {
    std::set<DependencyTicket> prerequisites;
    prerequisites.insert(get_actuation_input_port().ticket());
    for (ModelInstanceIndex i(0); i < num_model_instances(); ++i) {
      prerequisites.insert(get_actuation_input_port(i).ticket());
    }
    const auto& actuation_input_without_effort_limit_cache_entry =
        this->DeclareCacheEntry(
            "ActuationInputWithoutEffortLimit", VectorX<T>(num_actuators()),
            &MultibodyPlant::CalcActuationInputWithoutEffortLimit,
            std::move(prerequisites));
    cache_indices_.actuation_input_without_effort_limit =
        actuation_input_without_effort_limit_cache_entry.cache_index();
  }

  // Cache actuation input data (with effort limit).
  {
    const auto& actuation_input_with_effort_limit_cache_entry =
        this->DeclareCacheEntry(
            "ActuationInputWithEffortLimit", VectorX<T>(num_actuators()),
            &MultibodyPlant::CalcActuationInputWithEffortLimit,
            {this->cache_entry_ticket(
                cache_indices_.actuation_input_without_effort_limit)});
    cache_indices_.actuation_input_with_effort_limit =
        actuation_input_with_effort_limit_cache_entry.cache_index();
  }

  // Cache desired state input data.
  {
    std::set<DependencyTicket> prerequisites;
    for (ModelInstanceIndex i(0); i < num_model_instances(); ++i) {
      prerequisites.insert(get_desired_state_input_port(i).ticket());
    }
    const auto& desired_state_input_cache_entry = this->DeclareCacheEntry(
        "DesiredStateInput",
        internal::DesiredStateInput<T>(num_model_instances()),
        &MultibodyPlant::CalcDesiredStateInput, std::move(prerequisites));
    cache_indices_.desired_state_input =
        desired_state_input_cache_entry.cache_index();
  }
}

template <typename T>
void MultibodyPlant<T>::DeclareParameters() {
  DRAKE_DEMAND(this->is_finalized());

  // Collect ids from all constraints known at finalize time and set their
  // active status to true by default.
  std::map<MultibodyConstraintId, bool> constraint_active_status_map;

  for (const auto& [id, spec] : coupler_constraints_specs_) {
    constraint_active_status_map[id] = true;
  }
  for (const auto& [id, params] : distance_constraints_params_) {
    constraint_active_status_map[id] = true;
  }
  for (const auto& [id, spec] : ball_constraints_specs_) {
    constraint_active_status_map[id] = true;
  }
  for (const auto& [id, spec] : weld_constraints_specs_) {
    constraint_active_status_map[id] = true;
  }
  for (const auto& [id, spec] : tendon_constraints_specs_) {
    constraint_active_status_map[id] = true;
  }

  // Active status parameters.
  internal::ConstraintActiveStatusMap map_wrapper{constraint_active_status_map};
  parameter_indices_.constraint_active_status = systems::AbstractParameterIndex{
      this->DeclareAbstractParameter(drake::Value(map_wrapper))};

  // Constraint parameters.
  parameter_indices_.distance_constraints =
      systems::AbstractParameterIndex{this->DeclareAbstractParameter(
          drake::Value(internal::DistanceConstraintParamsMap{
              distance_constraints_params_}))};
}

template <typename T>
void MultibodyPlant<T>::CalcStateOutput(const Context<T>& context,
                                        BasicVector<T>* output) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  output->SetFromVector(GetPositionsAndVelocities(context));
}

template <typename T>
void MultibodyPlant<T>::CalcInstanceStateOutput(
    ModelInstanceIndex model_instance, const Context<T>& context,
    BasicVector<T>* output) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  output->SetFromVector(GetPositionsAndVelocities(context, model_instance));
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcInstanceGeneralizedContactForcesOutput(
    ModelInstanceIndex model_instance, const Context<T>& context,
    BasicVector<T>* output) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  if (!sampled) {
    ValidateGeometryInput(
        context, get_generalized_contact_forces_output_port(model_instance));
  }

  // Vector of generalized contact forces for the entire multibody system.
  // We will find a pointer to it based on the discrete / sampled mode.
  const VectorX<T>* tau_contact{};
  if (is_discrete()) {
    if constexpr (sampled) {
      DRAKE_DEMAND(use_sampled_output_ports_);
      const DiscreteStepMemory::Data<T>* const memory =
          get_discrete_step_memory(context);
      if (memory == nullptr) {
        // The plant has not been stepped yet.
        DRAKE_DEMAND(sampled == true);
        output->SetZero();
        return;
      }
      tau_contact = &memory->contact_solver_results.tau_contact;
    } else {
      tau_contact = &discrete_update_manager_->EvalContactSolverResults(context)
                         .tau_contact;
    }
  } else {
    DRAKE_DEMAND(sampled == false);
    tau_contact = &EvalGeneralizedContactForcesContinuous(context);
  }
  DRAKE_DEMAND(tau_contact != nullptr);

  // Generalized velocities and generalized forces are ordered in the same way.
  // Thus we can call GetVelocitiesFromArray(). Make sure we only take the
  // rigid body dofs.
  output->SetFromVector(GetVelocitiesFromArray(
      model_instance, (*tau_contact).head(num_velocities())));
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_applied_generalized_force_input_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_input_port(input_port_indices_.applied_generalized_force);
}

template <typename T>
const systems::InputPort<T>& MultibodyPlant<T>::get_actuation_input_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_input_port(
      input_port_indices_.instance.at(model_instance).actuation);
}

template <typename T>
const systems::InputPort<T>& MultibodyPlant<T>::get_actuation_input_port()
    const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_input_port(input_port_indices_.actuation);
}

template <typename T>
const systems::OutputPort<T>& MultibodyPlant<T>::get_net_actuation_output_port()
    const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.net_actuation);
}

template <typename T>
const systems::OutputPort<T>& MultibodyPlant<T>::get_net_actuation_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_output_port(
      output_port_indices_.instance.at(model_instance).net_actuation);
}

template <typename T>
const systems::InputPort<T>& MultibodyPlant<T>::get_desired_state_input_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_input_port(
      input_port_indices_.instance.at(model_instance).desired_state);
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_applied_spatial_force_input_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_input_port(input_port_indices_.applied_spatial_force);
}

template <typename T>
const systems::OutputPort<T>& MultibodyPlant<T>::get_state_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.state);
}

template <typename T>
const systems::OutputPort<T>& MultibodyPlant<T>::get_state_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_output_port(
      output_port_indices_.instance.at(model_instance).state);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_generalized_acceleration_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.generalized_acceleration);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_generalized_acceleration_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_output_port(output_port_indices_.instance.at(model_instance)
                                   .generalized_acceleration);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_generalized_contact_forces_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_output_port(output_port_indices_.instance.at(model_instance)
                                   .generalized_contact_forces);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_contact_results_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.contact_results);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_reaction_forces_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.reaction_forces);
}

template <typename T>
void MultibodyPlant<T>::DeclareSceneGraphPorts() {
  input_port_indices_.geometry_query =
      this->DeclareAbstractInputPort("geometry_query",
                                     Value<geometry::QueryObject<T>>{})
          .get_index();
  output_port_indices_.geometry_pose =
      this->DeclareAbstractOutputPort(
              "geometry_pose", &MultibodyPlant<T>::CalcGeometryPoseOutput,
              {this->configuration_ticket()})
          .get_index();
  physical_models_->DeclareSceneGraphPorts();
}

template <typename T>
void MultibodyPlant<T>::CalcBodyPosesOutput(
    const Context<T>& context,
    std::vector<math::RigidTransform<T>>* outupt) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  outupt->resize(num_bodies());
  for (BodyIndex body_index(0); body_index < this->num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    outupt->at(body_index) = EvalBodyPoseInWorld(context, body);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcBodySpatialVelocitiesOutput(
    const Context<T>& context, std::vector<SpatialVelocity<T>>* output) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  output->resize(num_bodies());
  for (BodyIndex body_index(0); body_index < this->num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    output->at(body_index) = EvalBodySpatialVelocityInWorld(context, body);
  }
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcBodySpatialAccelerationsOutput(
    const Context<T>& context,
    std::vector<SpatialAcceleration<T>>* output) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  output->resize(num_bodies());
  const AccelerationKinematicsCache<T>& ac =
      EvalAccelerationKinematicsCacheForOutputPortCalc<sampled>(context);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const auto mobod_index = get_body(body_index).mobod_index();
    output->at(body_index) = ac.get_A_WB(mobod_index);
  }
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcGeneralizedAccelerationOutput(
    const Context<T>& context, BasicVector<T>* output) const {
  const AccelerationKinematicsCache<T>& ac =
      EvalAccelerationKinematicsCacheForOutputPortCalc<sampled>(context);
  output->SetFromVector(ac.get_vdot());
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcInstanceGeneralizedAccelerationOutput(
    ModelInstanceIndex model_instance, const Context<T>& context,
    BasicVector<T>* output) const {
  // Our calc function is the same for sampled vs unsampled but our dependency
  // tickets differ, so it's convenient to use the DeclareSampledOutputPort
  // (which requires the template argument) anyway.
  unused(sampled);
  // The per-instance calc delegates to the full-model calc and then slices it.
  const VectorX<T>& generalized_acceleration =
      get_generalized_acceleration_output_port().Eval(context);
  output->SetFromVector(
      this->GetVelocitiesFromArray(model_instance, generalized_acceleration));
}

template <typename T>
const SpatialAcceleration<T>&
MultibodyPlant<T>::EvalBodySpatialAccelerationInWorld(
    const Context<T>& context, const RigidBody<T>& body_B) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  DRAKE_DEMAND(this == &body_B.GetParentPlant());
  this->ValidateContext(context);
  const AccelerationKinematicsCache<T>& ac = this->EvalForwardDynamics(context);
  return ac.get_A_WB(body_B.mobod_index());
}

template <typename T>
void MultibodyPlant<T>::CalcGeometryPoseOutput(
    const Context<T>& context, FramePoseVector<T>* output) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);

  // NOTE: The body index to frame id map *always* includes the world body but
  // the world body does *not* get reported in the frame poses; only dynamic
  // frames do.
  // TODO(amcastro-tri): Make use of RigidBody::EvalPoseInWorld(context) once
  // caching lands.
  output->clear();
  for (const auto& it : body_index_to_frame_id_) {
    const BodyIndex body_index = it.first;
    if (body_index == world_index()) continue;
    const RigidBody<T>& body = get_body(body_index);

    // NOTE: The GeometryFrames for each body were registered in the world
    // frame, so we report poses in the world frame.
    output->set_value(body_index_to_frame_id_.at(body_index),
                      pc.get_X_WB(body.mobod_index()));
  }
}

template <typename T>
template <bool sampled>
void MultibodyPlant<T>::CalcReactionForcesOutput(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* output) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(output != nullptr);
  DRAKE_DEMAND(ssize(*output) == num_joints());

  // Sampled mode is a simple copy.
  if constexpr (sampled) {
    const DiscreteStepMemory::Data<T>* memory =
        get_discrete_step_memory(context);
    if (memory == nullptr) {
      for (SpatialForce<T>& item : *output) {
        item.SetZero();
      }
    } else {
      *output = memory->reaction_forces;
    }
    return;
  }

  // Unsampled mode can simply delegate to the helper.
  CalcReactionForces(context, output);
}

template <typename T>
void MultibodyPlant<T>::CalcReactionForces(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* output) const {
  // Guard against failure to acquire the geometry input deep in the call graph.
  ValidateGeometryInput(context, get_reaction_forces_output_port());

  // Compute the multibody forces applied during the computation of forward
  // dynamics, consistent with the value of vdot obtained below.
  MultibodyForces<T> applied_forces(*this);
  auto& Fapplied_Bo_W_array = applied_forces.mutable_body_forces();
  auto& tau_applied = applied_forces.mutable_generalized_forces();
  if (is_discrete()) {
    applied_forces =
        discrete_update_manager_->EvalDiscreteUpdateMultibodyForces(context);
  } else {
    CalcNonContactForcesContinuous(context, &applied_forces);
    CalcAndAddSpatialContactForcesContinuous(context, &Fapplied_Bo_W_array);
  }

  // Compute reaction forces at each mobilizer.
  // N.B. For discrete systems, this approach right now evaluates Coriolis terms
  // at the state stored in the context (previous time step). This is correct
  // for the discrete solvers we have today, though it might change for future
  // solvers that prefer an implicit evaluation of these terms.
  // TODO(amcastro-tri): Consider having a
  //  DiscreteUpdateManager::EvalReactionForces() to ensure the manager performs
  //  this computation consistently with its discrete update.
  const VectorX<T>& vdot = this->EvalForwardDynamics(context).get_vdot();
  std::vector<SpatialAcceleration<T>> A_WB_vector(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_vector(num_bodies());
  VectorX<T> tau_id(num_velocities());
  internal_tree().CalcInverseDynamics(context, vdot, Fapplied_Bo_W_array,
                                      tau_applied, &A_WB_vector,
                                      &F_BMo_W_vector, &tau_id);

  // Since vdot is the result of Fapplied and tau_applied we expect the result
  // from inverse dynamics to be zero.
  // TODO(amcastro-tri): find a better estimation for this bound. For instance,
  //  we can make an estimation based on the trace of the mass matrix (Jain
  //  2011, Eq. 4.21). For now we only ASSERT though with a better estimation we
  //  could promote this to a DEMAND.
  // TODO(amcastro-tri) Uncomment this line once issue #12473 is resolved.
  // DRAKE_ASSERT(tau_id.norm() <
  //              100 * num_velocities() *
  //              std::numeric_limits<double>::epsilon());

  // Map mobilizer reaction forces to joint reaction forces and perform the
  // necessary frame conversions.
  for (JointIndex joint_index : GetJointIndices()) {
    const Joint<T>& joint = get_joint(joint_index);
    const internal::MobodIndex mobilizer_index =
        internal_tree().get_joint_mobilizer(joint_index);
    const internal::Mobilizer<T>& mobilizer =
        internal_tree().get_mobilizer(mobilizer_index);

    // Reversed means the joint's parent(child) body is the outboard(inboard)
    // body for the mobilizer.
    const bool is_reversed = mobilizer.mobod().is_reversed();

    // F_BMo_W is the mobilizer reaction force on mobilized body B at the origin
    // Mo of the mobilizer's outboard frame M, expressed in the world frame W.
    const SpatialForce<T>& F_BMo_W = F_BMo_W_vector[mobilizer_index];

    // But the quantity of interest, F_CJc_Jc, is the joint's reaction force on
    // the joint's child body C at the joint's child frame Jc, expressed in Jc.
    SpatialForce<T>& F_CJc_Jc = output->at(joint.ordinal());

    // Frames of interest:
    const Frame<T>& frame_Jc = joint.frame_on_child();
    const Frame<T>& frame_M = mobilizer.outboard_frame();

    // We'll need this in both cases below since we're required to report
    // the reaction force expressed in the joint's child frame Jc.
    const RotationMatrix<T> R_JcW =
        frame_Jc.CalcRotationMatrixInWorld(context).inverse();

    if (&frame_M == &frame_Jc) {
      // This is the easy case. Just need to re-express.
      F_CJc_Jc = R_JcW * F_BMo_W;
      continue;
    }

    // If the mobilizer is reversed, Newton's 3ʳᵈ law (action/reaction) (and
    // knowing Drake's joints are massless) says the force on the child at M
    // is equal and opposite to the force on the parent at M.
    const SpatialForce<T> F_CMo_W = is_reversed ? -F_BMo_W : F_BMo_W;
    const SpatialForce<T> F_CMo_Jc = R_JcW * F_CMo_W;  // Reexpress in Jc.

    // However, the reaction force we want to report on the child is at Jc,
    // not M. We need to shift the application point from Mo to Jco.

    // Find the shift vector p_MoJco_Jc (= -p_JcoMo_Jc).
    const RigidTransform<T> X_JcM = frame_M.CalcPose(context, frame_Jc);
    const Vector3<T> p_MoJco_Jc = -X_JcM.translation();

    // Perform  the M->Jc shift.
    F_CJc_Jc = F_CMo_Jc.Shift(p_MoJco_Jc);
  }
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_body_poses_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.body_poses);
}

template <typename T>
const OutputPort<T>&
MultibodyPlant<T>::get_body_spatial_velocities_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.body_spatial_velocities);
}

template <typename T>
const OutputPort<T>&
MultibodyPlant<T>::get_body_spatial_accelerations_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(output_port_indices_.body_spatial_accelerations);
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_pose_output_port() const {
  return systems::System<T>::get_output_port(
      output_port_indices_.geometry_pose);
}

template <typename T>
const systems::InputPort<T>& MultibodyPlant<T>::get_geometry_query_input_port()
    const {
  return this->get_input_port(input_port_indices_.geometry_query);
}

template <typename T>
const OutputPort<T>&
MultibodyPlant<T>::get_deformable_body_configuration_output_port() const {
  const DeformableModel<T>* deformable_model =
      physical_models_->deformable_model();
  DRAKE_DEMAND(deformable_model != nullptr);
  return systems::System<T>::get_output_port(
      deformable_model->configuration_output_port_index());
}

template <typename T>
void MultibodyPlant<T>::ThrowIfFinalized(const char* source_method) const {
  if (is_finalized()) {
    throw std::logic_error(
        "Post-finalize calls to '" + std::string(source_method) +
        "()' are "
        "not allowed; calls to this method must happen before Finalize().");
  }
}

template <typename T>
void MultibodyPlant<T>::ThrowIfNotFinalized(const char* source_method) const {
  if (!is_finalized()) {
    throw std::logic_error("Pre-finalize calls to '" +
                           std::string(source_method) +
                           "()' are "
                           "not allowed; you must call Finalize() first.");
  }
}

template <typename T>
void MultibodyPlant<T>::RemoveUnsupportedScalars(
    const internal::ScalarConvertibleComponent<T>& component) {
  systems::SystemScalarConverter& scalar_converter =
      this->get_mutable_system_scalar_converter();
  if (!component.is_cloneable_to_double()) {
    scalar_converter.Remove<double, T>();
  }
  if (!component.is_cloneable_to_autodiff()) {
    scalar_converter.Remove<AutoDiffXd, T>();
  }
  if (!component.is_cloneable_to_symbolic()) {
    scalar_converter.Remove<symbolic::Expression, T>();
  }
}

template <typename T>
std::vector<std::set<BodyIndex>>
MultibodyPlant<T>::FindSubgraphsOfWeldedBodies() const {
  return internal_tree().graph().GetSubgraphsOfWeldedLinks();
}

template <typename T>
bool MultibodyPlant<T>::is_gravity_enabled(
    ModelInstanceIndex model_instance) const {
  if (model_instance >= num_model_instances()) {
    throw std::logic_error("Model instance index is invalid.");
  }
  return gravity_field().is_enabled(model_instance);
}

template <typename T>
void MultibodyPlant<T>::set_gravity_enabled(ModelInstanceIndex model_instance,
                                            bool is_enabled) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  if (model_instance >= num_model_instances()) {
    throw std::logic_error("Model instance index is invalid.");
  }
  mutable_gravity_field().set_enabled(model_instance, is_enabled);
}

template <typename T>
DeformableModel<T>& MultibodyPlant<T>::AddDeformableModel() {
  // TODO(xuchenhan-tri): We should verify that the plant is not finalized yet.
  // Right now, we can do exactly just that yet because is_finalized() returns
  // true iff the MultibodyTree is finalized. There's no easy way to confirm
  // that a plant is not yet finalized when the tree is already finalized.
  DRAKE_DEMAND(physical_models_->deformable_model() == nullptr);
  return physical_models_->AddDeformableModel(
      std::make_unique<DeformableModel<T>>(this));
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::ComputeFrictionCoefficient(
    const T& speed_BcAc, const CoulombFriction<double>& friction) const {
  DRAKE_ASSERT(speed_BcAc >= 0);
  const double mu_d = friction.dynamic_friction();
  const double mu_s = friction.static_friction();
  const T v = speed_BcAc * inv_v_stiction_tolerance_;
  if (v >= 3) {
    return mu_d;
  } else if (v >= 1) {
    return mu_s - (mu_s - mu_d) * step5((v - 1) / 2);
  } else {
    return mu_s * step5(v);
  }
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::step5(const T& x) {
  DRAKE_ASSERT(0 <= x && x <= 1);
  const T x3 = x * x * x;
  return x3 * (10 + x * (6 * x - 15));  // 10x³ - 15x⁴ + 6x⁵
}

template <typename T>
AddMultibodyPlantSceneGraphResult<T> AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,
    std::unique_ptr<MultibodyPlant<T>> plant,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph) {
  DRAKE_DEMAND(builder != nullptr);
  DRAKE_THROW_UNLESS(plant != nullptr);
  return internal::AddMultibodyPlantSceneGraphFromShared<T>(
      builder, std::move(plant), std::move(scene_graph));
}

template <typename T>
AddMultibodyPlantSceneGraphResult<T> AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,  // BR
    double time_step,                     // BR
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph) {
  DRAKE_DEMAND(builder != nullptr);
  auto plant = std::make_unique<MultibodyPlant<T>>(time_step);
  return internal::AddMultibodyPlantSceneGraphFromShared<T>(
      builder, std::move(plant), std::move(scene_graph));
}

namespace internal {
template <typename T>
AddMultibodyPlantSceneGraphResult<T> AddMultibodyPlantSceneGraphFromShared(
    systems::DiagramBuilder<T>* builder,
    std::shared_ptr<MultibodyPlant<T>> plant,
    std::shared_ptr<geometry::SceneGraph<T>> scene_graph) {
  DRAKE_DEMAND(builder != nullptr);
  DRAKE_THROW_UNLESS(plant != nullptr);
  plant->set_name("plant");
  if (!scene_graph) {
    scene_graph = std::make_unique<geometry::SceneGraph<T>>();
    scene_graph->set_name("scene_graph");
  }
  auto* plant_ptr = builder->AddSystem(std::move(plant));
  auto* scene_graph_ptr = builder->AddSystem(std::move(scene_graph));
  plant_ptr->RegisterAsSourceForSceneGraph(scene_graph_ptr);
  builder->Connect(plant_ptr->get_geometry_pose_output_port(),
                   scene_graph_ptr->get_source_pose_port(
                       plant_ptr->get_source_id().value()));
  builder->Connect(scene_graph_ptr->get_query_output_port(),
                   plant_ptr->get_geometry_query_input_port());
  builder->Connect(plant_ptr->get_deformable_body_configuration_output_port(),
                   scene_graph_ptr->get_source_configuration_port(
                       plant_ptr->get_source_id().value()));
  return {plant_ptr, scene_graph_ptr};
}
}  // namespace internal

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    /* Use static_cast to disambiguate the two different overloads. */
    static_cast<AddMultibodyPlantSceneGraphResult<T> (*)(
        systems::DiagramBuilder<T>*, double,
        std::unique_ptr<geometry::SceneGraph<T>>)>(
        &AddMultibodyPlantSceneGraph),
    /* Use static_cast to disambiguate the two different overloads. */
    static_cast<AddMultibodyPlantSceneGraphResult<T> (*)(
        systems::DiagramBuilder<T>*, std::unique_ptr<MultibodyPlant<T>>,
        std::unique_ptr<geometry::SceneGraph<T>>)>(
        &AddMultibodyPlantSceneGraph),
    &internal::AddMultibodyPlantSceneGraphFromShared<T>));

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::MultibodyPlant);
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct drake::multibody::AddMultibodyPlantSceneGraphResult);
