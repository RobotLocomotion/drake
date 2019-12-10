#include "drake/multibody/plant/multibody_plant.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/render/render_label.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

using geometry::ContactSurface;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::render::RenderLabel;
using geometry::SceneGraph;
using geometry::SourceId;
using systems::InputPort;
using systems::OutputPort;
using systems::State;

using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::internal::AccelerationKinematicsCache;
using drake::multibody::internal::ArticulatedBodyForceBiasCache;
using drake::multibody::internal::ArticulatedBodyInertiaCache;
using drake::multibody::internal::PositionKinematicsCache;
using drake::multibody::internal::VelocityKinematicsCache;
using drake::multibody::MultibodyForces;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using systems::BasicVector;
using systems::Context;
using systems::InputPort;
using systems::InputPortIndex;
using systems::OutputPortIndex;

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
  CalcCriticallyDampedHarmonicOscillatorParameters(
      double period, double inertia) {
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
    const double parent_mass = joint.parent_body().index() == world_index() ?
                               std::numeric_limits<double>::infinity() :
                               joint.parent_body().get_default_mass();
    const auto parent_params = CalcCriticallyDampedHarmonicOscillatorParameters(
        numerical_time_scale, parent_mass);
    // Penalty parameters for the child body (parent fixed).
    const double child_mass = joint.child_body().index() == world_index() ?
                               std::numeric_limits<double>::infinity() :
                               joint.child_body().get_default_mass();
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
          if (std::isnan(body->get_default_mass())) {
            return std::numeric_limits<double>::infinity();
          }

          const SpatialInertia<T>& M_PPo_P =
              body->default_spatial_inertia().template cast<T>();
          const RigidTransform<T> X_PJ = frame.GetFixedPoseInBodyFrame();
          const Vector3<T>& p_PJ = X_PJ.translation();
          const math::RotationMatrix<T>& R_PJ = X_PJ.rotation();
          const SpatialInertia<T> M_PJo_J =
              M_PPo_P.Shift(p_PJ).ReExpress(R_PJ);
          const RotationalInertia<T> I_PJo_J =
              M_PJo_J.CalcRotationalInertia();
          // Rotational inertia about the joint axis.
          const Vector3<T>& axis = joint.revolute_axis();
          const T I_a = axis.transpose() * (I_PJo_J * axis);
          return ExtractDoubleOrThrow(I_a);
        };

    // Rotational inertia about the joint's axis for the parent body.
    const double I_Pa =
        joint.parent_body().index() == world_index() ?
        std::numeric_limits<double>::infinity() :
        CalcRotationalInertiaAboutAxis(joint.frame_on_parent());
    auto parent_params = CalcCriticallyDampedHarmonicOscillatorParameters(
        numerical_time_scale, I_Pa);

    // Rotational inertia about the joint's axis for the child body.
    const double I_Ca =
        joint.child_body().index() == world_index() ?
        std::numeric_limits<double>::infinity() :
        CalcRotationalInertiaAboutAxis(joint.frame_on_child());
    auto child_params = CalcCriticallyDampedHarmonicOscillatorParameters(
        numerical_time_scale, I_Ca);

    // Return the combined penalty parameters of the two bodies.
    return PickLessStiffPenaltyParameters(parent_params, child_params);
  }
};
}  // namespace internal

namespace {

// Hack to fully qualify frame names, pending resolution of #9128. Used by
// geometry registration routines.
template <typename T>
std::string GetScopedName(
    const MultibodyPlant<T>& plant,
    ModelInstanceIndex model_instance, const std::string& name) {
  if (model_instance != world_model_instance() &&
      model_instance != default_model_instance()) {
    return plant.GetModelInstanceName(model_instance) + "::" + name;
  } else {
    return name;
  }
}

}  // namespace

template <typename T>
MultibodyPlant<T>::MultibodyPlant(double time_step)
    : MultibodyPlant(nullptr, time_step) {}

template <typename T>
MultibodyPlant<T>::MultibodyPlant(
    std::unique_ptr<internal::MultibodyTree<T>> tree_in, double time_step)
    : internal::MultibodyTreeSystem<T>(
          systems::SystemTypeTag<MultibodyPlant>{},
          std::move(tree_in), time_step > 0),
      time_step_(time_step) {
  DRAKE_THROW_UNLESS(time_step >= 0);
  // TODO(eric.cousineau): Combine all of these elements into one struct, make
  // it less brittle.
  visual_geometries_.emplace_back();  // Entries for the "world" body.
  collision_geometries_.emplace_back();
  X_WB_default_list_.emplace_back();
  // Add the world body to the graph.
  multibody_graph_.AddBody(world_body().name(), world_body().model_instance());
}

template <typename T>
void MultibodyPlant<T>::set_contact_model(ContactModel model) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  contact_model_ = model;
}

template <typename T>
ContactModel MultibodyPlant<T>::get_contact_model() const {
  return contact_model_;
}

template <typename T>
void MultibodyPlant<T>::SetFreeBodyRandomRotationDistributionToUniform(
    const Body<T>& body) {
  RandomGenerator generator;
  auto q_FM =
      math::UniformlyRandomQuaternion<symbolic::Expression>(&generator);
  SetFreeBodyRandomRotationDistribution(body, q_FM);
}

template <typename T>
const WeldJoint<T>& MultibodyPlant<T>::WeldFrames(
    const Frame<T>& A, const Frame<T>& B,
    const math::RigidTransform<double>& X_AB) {
  const std::string joint_name = A.name() + "_welds_to_" + B.name();
  return AddJoint(std::make_unique<WeldJoint<T>>(joint_name, A, B, X_AB));
}

template <typename T>
geometry::SourceId MultibodyPlant<T>::RegisterAsSourceForSceneGraph(
    SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(!geometry_source_is_registered());
  // Save the GS pointer so that on later geometry registrations can use this
  // instance. This will be nullified at Finalize().
  scene_graph_ = scene_graph;
  source_id_ = member_scene_graph().RegisterSource();
  const geometry::FrameId world_frame_id =
      member_scene_graph().world_frame_id();
  body_index_to_frame_id_[world_index()] = world_frame_id;
  frame_id_to_body_index_[world_frame_id] = world_index();
  DeclareSceneGraphPorts();
  return source_id_.value();
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name) {
  return RegisterVisualGeometry(
      body, X_BG, shape, name, geometry::IllustrationProperties());
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const Vector4<double>& diffuse_color) {
  return RegisterVisualGeometry(
      body, X_BG, shape, name,
      geometry::MakePhongIllustrationProperties(diffuse_color));
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const geometry::IllustrationProperties& properties) {
  // TODO(SeanCurtis-TRI): Consider simply adding an interface that takes a
  // unique pointer to an already instantiated GeometryInstance. This will
  // require shuffling around a fair amount of code and should ultimately be
  // supplanted by providing a cleaner interface between parsing MBP and SG
  // elements.
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());

  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register geometry that has a fixed path to world to the world body (i.e.,
  // as anchored geometry).
  GeometryId id =
      RegisterGeometry(body, X_BG, shape,
                       GetScopedName(*this, body.model_instance(), name));
  member_scene_graph().AssignRole(*source_id_, id, properties);

  // TODO(SeanCurtis-TRI): Eliminate the automatic assignment of perception
  //  and illustration in favor of a protocol that allows definition.
  geometry::PerceptionProperties perception_props;
  perception_props.AddProperty("label", "id", RenderLabel(body.index()));
  perception_props.AddProperty(
      "phong", "diffuse",
      properties.GetPropertyOrDefault(
          "phong", "diffuse", Vector4<double>(0.9, 0.9, 0.9, 1.0)));
  member_scene_graph().AssignRole(*source_id_, id, perception_props);

  const int visual_index = geometry_id_to_visual_index_.size();
  geometry_id_to_visual_index_[id] = visual_index;
  DRAKE_ASSERT(num_bodies() == static_cast<int>(visual_geometries_.size()));
  visual_geometries_[body.index()].push_back(id);
  return id;
}

template <typename T>
const std::vector<geometry::GeometryId>&
MultibodyPlant<T>::GetVisualGeometriesForBody(const Body<T>& body) const {
  return visual_geometries_[body.index()];
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterCollisionGeometry(
    const Body<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const CoulombFriction<double>& coulomb_friction) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());

  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register geometry that has a fixed path to world to the world body (i.e.,
  // as anchored geometry).
  GeometryId id =
      RegisterGeometry(body, X_BG, shape,
                       GetScopedName(*this, body.model_instance(), name));

  // TODO(SeanCurtis-TRI): Push the contact parameters into the
  // ProximityProperties.
  member_scene_graph().AssignRole(
      *source_id_, id, geometry::ProximityProperties());
  const int collision_index = geometry_id_to_collision_index_.size();
  geometry_id_to_collision_index_[id] = collision_index;
  DRAKE_ASSERT(
      static_cast<int>(default_coulomb_friction_.size()) == collision_index);
  default_coulomb_friction_.push_back(coulomb_friction);
  DRAKE_ASSERT(num_bodies() == static_cast<int>(collision_geometries_.size()));
  collision_geometries_[body.index()].push_back(id);
  return id;
}

template <typename T>
const std::vector<geometry::GeometryId>&
MultibodyPlant<T>::GetCollisionGeometriesForBody(const Body<T>& body) const {
  DRAKE_ASSERT(body.index() < num_bodies());
  return collision_geometries_[body.index()];
}

template <typename T>
geometry::GeometrySet MultibodyPlant<T>::CollectRegisteredGeometries(
    const std::vector<const Body<T>*>& bodies) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());

  geometry::GeometrySet geometry_set;
  for (const Body<T>* body : bodies) {
    std::optional<FrameId> frame_id = GetBodyFrameIdIfExists(body->index());
    if (frame_id) {
      geometry_set.Add(frame_id.value());
    }
  }
  return geometry_set;
}

template <typename T>
std::vector<const Body<T>*> MultibodyPlant<T>::GetBodiesWeldedTo(
    const Body<T>& body) const {
  const std::set<BodyIndex> island =
      multibody_graph_.FindBodiesWeldedTo(body.index());
  // Map body indices to pointers.
  std::vector<const Body<T>*> sub_graph_bodies;
  for (BodyIndex body_index : island) {
    sub_graph_bodies.push_back(&get_body(body_index));
  }
  return sub_graph_bodies;
}

template <typename T>
std::unordered_set<BodyIndex> MultibodyPlant<T>::GetFloatingBaseBodies() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  std::unordered_set<BodyIndex> floating_bodies;
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    if (body.is_floating()) floating_bodies.insert(body.index());
  }
  return floating_bodies;
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterGeometry(
    const Body<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape,
    const std::string& name) {
  DRAKE_ASSERT(!is_finalized());
  DRAKE_ASSERT(geometry_source_is_registered());
  // If not already done, register a frame for this body.
  if (!body_has_registered_frame(body)) {
    FrameId frame_id = member_scene_graph().RegisterFrame(
        source_id_.value(),
        GeometryFrame(
            GetScopedName(*this, body.model_instance(), body.name()),
            /* TODO(@SeanCurtis-TRI): Add test coverage for this
             * model-instance support as requested in #9390. */
            body.model_instance()));
    body_index_to_frame_id_[body.index()] = frame_id;
    frame_id_to_body_index_[frame_id] = body.index();
  }

  // Register geometry in the body frame.
  std::unique_ptr<geometry::GeometryInstance> geometry_instance =
      std::make_unique<GeometryInstance>(X_BG, shape.Clone(), name);
  GeometryId geometry_id = member_scene_graph().RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.index()],
      std::move(geometry_instance));
  geometry_id_to_body_index_[geometry_id] = body.index();
  return geometry_id;
}

template<typename T>
void MultibodyPlant<T>::SetFreeBodyPoseInWorldFrame(
    systems::Context<T>* context,
    const Body<T>& body, const math::RigidTransform<T>& X_WB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context);
}

template<typename T>
void MultibodyPlant<T>::SetFreeBodyPoseInAnchoredFrame(
    systems::Context<T>* context,
    const Frame<T>& frame_F, const Body<T>& body,
    const math::RigidTransform<T>& X_FB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();

  if (!internal_tree().get_topology().IsBodyAnchored(frame_F.body().index())) {
    throw std::logic_error(
        "Frame '" + frame_F.name() + "' must be anchored to the world frame.");
  }

  // Pose of frame F in its parent body frame P.
  const RigidTransform<T> X_PF = frame_F.GetFixedPoseInBodyFrame();
  // Pose of frame F's parent body P in the world.
  const RigidTransform<T>& X_WP = EvalBodyPoseInWorld(*context, frame_F.body());
  // Pose of "body" B in the world frame.
  const RigidTransform<T> X_WB = X_WP * X_PF * X_FB;
  SetFreeBodyPoseInWorldFrame(context, body, X_WB);
}

template<typename T>
void MultibodyPlant<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context,
    const VectorX<T>& known_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  DRAKE_THROW_UNLESS(A_WB_array != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(A_WB_array->size()) == num_bodies());
  internal_tree().CalcSpatialAccelerationsFromVdot(
      context, internal_tree().EvalPositionKinematics(context),
      internal_tree().EvalVelocityKinematics(context), known_vdot, A_WB_array);
  // Permute BodyNodeIndex -> BodyIndex.
  // TODO(eric.cousineau): Remove dynamic allocations. Making this in-place
  // still required dynamic allocation for recording permutation indices.
  // Can change implementation once MultibodyTree becomes fully internal.
  std::vector<SpatialAcceleration<T>> A_WB_array_node = *A_WB_array;
  const internal::MultibodyTreeTopology& topology =
      internal_tree().get_topology();
  for (internal::BodyNodeIndex node_index(1);
       node_index < topology.get_num_body_nodes(); ++node_index) {
    const BodyIndex body_index = topology.get_body_node(node_index).body;
    (*A_WB_array)[body_index] = A_WB_array_node[node_index];
  }
}

template<typename T>
void MultibodyPlant<T>::CalcForceElementsContribution(
      const systems::Context<T>& context,
      MultibodyForces<T>* forces) const {
  DRAKE_THROW_UNLESS(forces != nullptr);
  DRAKE_THROW_UNLESS(forces->CheckHasRightSizeForModel(internal_tree()));
  internal_tree().CalcForceElementsContribution(
      context, EvalPositionKinematics(context),
      EvalVelocityKinematics(context),
      forces);
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  // After finalizing the base class, tree is read-only.
  internal::MultibodyTreeSystem<T>::Finalize();
  if (geometry_source_is_registered()) {
    FilterAdjacentBodies();
    ExcludeCollisionsWithVisualGeometry();
  }
  FinalizePlantOnly();
}

template<typename T>
void MultibodyPlant<T>::SetUpJointLimitsParameters() {
  for (JointIndex joint_index(0); joint_index < num_joints();
       ++joint_index) {
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
            internal::JointLimitsPenaltyParametersEstimator<T>::
            CalcRevoluteJointPenaltyParameters(
                *revolute_joint, penalty_time_scale);
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
            internal::JointLimitsPenaltyParametersEstimator<T>::
            CalcPrismaticJointPenaltyParameters(
                *prismatic_joint, penalty_time_scale);
        joint_limits_parameters_.stiffness.push_back(penalty_parameters.first);
        joint_limits_parameters_.damping.push_back(penalty_parameters.second);
      }
    }

    // Since currently MBP only handles joint limits for discrete models, verify
    // there are no joint limits when the model is continuous.
    // Therefore we throw an exception with an appropriate message when a user
    // specifies joint limits for a continuous model.
    if (!is_discrete()) {
      for (size_t i = 0; i < joint_limits_parameters_.stiffness.size(); ++i) {
        const double stiffness = joint_limits_parameters_.stiffness[i];
        if (!std::isinf(stiffness)) {
          const JointIndex index =
              joint_limits_parameters_.joints_with_limits[i];
          throw std::logic_error(
              "Currently MultibodyPlant does not handle joint limits for "
              "continuous models. However a limit was specified for joint `"
              "`" + get_joint(index).name() + "`.");
        }
      }
    }
  }
}

template<typename T>
void MultibodyPlant<T>::FinalizePlantOnly() {
  DeclareStateCacheAndPorts();
  if (num_collision_geometries() > 0 &&
      penalty_method_contact_parameters_.time_scale < 0)
    set_penetration_allowance();
  if (num_collision_geometries() > 0 &&
      friction_model_.stiction_tolerance() < 0)
    set_stiction_tolerance();
  // Make a contact solver when the plant is modeled as a discrete system.
  if (is_discrete()) {
    tamsi_solver_ =
        std::make_unique<TamsiSolver<T>>(num_velocities());
    // Set the stiction tolerance according to the values set by users with
    // set_stiction_tolerance().
    TamsiSolverParameters solver_parameters;
    solver_parameters.stiction_tolerance =
        friction_model_.stiction_tolerance();
    tamsi_solver_->set_solver_parameters(solver_parameters);
  } else {
    // We only build hydroelastics if the user requested it AND if geometry was
    // registered with a SceneGraph. Since by default bodies are rigid, we use
    // point contact unless the user specifies otherwise.
    if (contact_model_ != ContactModel::kPointContactOnly && get_source_id())
      MakeHydroelasticModels();
  }
  SetUpJointLimitsParameters();
  scene_graph_ = nullptr;  // must not be used after Finalize().
}

template <typename T>
MatrixX<T> MultibodyPlant<T>::MakeActuationMatrix() const {
  MatrixX<T> B = MatrixX<T>::Zero(num_velocities(), num_actuated_dofs());
  for (JointActuatorIndex actuator_index(0);
       actuator_index < num_actuators(); ++actuator_index) {
    const JointActuator<T>& actuator = get_joint_actuator(actuator_index);
    // This method assumes actuators on single dof joints. Assert this
    // condition.
    DRAKE_DEMAND(actuator.joint().num_velocities() == 1);
    B(actuator.joint().velocity_start(), actuator.index()) = 1;
  }
  return B;
}

template <typename T>
struct MultibodyPlant<T>::SceneGraphStub {
  struct StubSceneGraphInspector {
    const geometry::ProximityProperties* GetProximityProperties(
        GeometryId) const {
      return nullptr;
    }
    const geometry::IllustrationProperties* GetIllustrationProperties(
        GeometryId) const {
      return nullptr;
    }
  };

  static void Throw(const char* operation_name) {
    throw std::logic_error(fmt::format(
        "Cannot {} on a SceneGraph<symbolic::Expression>", operation_name));
  }

  static FrameId world_frame_id() {
    return SceneGraph<double>::world_frame_id();
  }

#define DRAKE_STUB(Ret, Name)                   \
  template <typename... Args>                   \
  Ret Name(Args...) const { Throw(#Name); return Ret(); }

  DRAKE_STUB(void, AssignRole)
  DRAKE_STUB(void, ExcludeCollisionsBetween)
  DRAKE_STUB(void, ExcludeCollisionsWithin)
  DRAKE_STUB(FrameId, RegisterFrame)
  DRAKE_STUB(GeometryId, RegisterGeometry)
  DRAKE_STUB(SourceId, RegisterSource)
  const StubSceneGraphInspector model_inspector() const {
    Throw("model_inspector");
    return StubSceneGraphInspector();
  }

#undef DRAKE_STUB
};

template <typename T>
typename MultibodyPlant<T>::MemberSceneGraph&
MultibodyPlant<T>::member_scene_graph() {
  DRAKE_THROW_UNLESS(scene_graph_ != nullptr);
  return *scene_graph_;
}

// Specialize this function so that we can use our Stub class; we cannot call
// methods on SceneGraph<Expression> because they do not exist.
template <>
typename MultibodyPlant<symbolic::Expression>::MemberSceneGraph&
MultibodyPlant<symbolic::Expression>::member_scene_graph() {
  static never_destroyed<SceneGraphStub> stub_;
  return stub_.access();
}

template <typename T>
void MultibodyPlant<T>::CheckValidState(const systems::State<T>* state) const {
  DRAKE_THROW_UNLESS(state != nullptr);
  DRAKE_THROW_UNLESS(
      is_discrete() == (state->get_discrete_state().num_groups() > 0));
}

template <typename T>
void MultibodyPlant<T>::FilterAdjacentBodies() {
  DRAKE_DEMAND(geometry_source_is_registered());
  // Disallow collisions between adjacent bodies. Adjacency is implied by the
  // existence of a joint between bodies.
  for (JointIndex j{0}; j < num_joints(); ++j) {
    const Joint<T>& joint = get_joint(j);
    const Body<T>& child = joint.child_body();
    const Body<T>& parent = joint.parent_body();
    if (parent.index() == world_index()) continue;
    std::optional<FrameId> child_id = GetBodyFrameIdIfExists(child.index());
    std::optional<FrameId> parent_id = GetBodyFrameIdIfExists(parent.index());

    if (child_id && parent_id) {
      member_scene_graph().ExcludeCollisionsBetween(
          geometry::GeometrySet(*child_id),
          geometry::GeometrySet(*parent_id));
    }
  }
  // We must explictly exclude collisions between all geometries registered
  // against the world.
  // TODO(eric.cousineau): Do this in a better fashion (#11117).
  auto g_world = CollectRegisteredGeometries(GetBodiesWeldedTo(world_body()));
  member_scene_graph().ExcludeCollisionsWithin(g_world);
}

template <typename T>
void MultibodyPlant<T>::ExcludeCollisionsWithVisualGeometry() {
  DRAKE_DEMAND(geometry_source_is_registered());
  geometry::GeometrySet visual;
  for (const auto& body_geometries : visual_geometries_) {
    visual.Add(body_geometries);
  }
  geometry::GeometrySet collision;
  for (const auto& body_geometries : collision_geometries_) {
    collision.Add(body_geometries);
  }
  member_scene_graph().ExcludeCollisionsWithin(visual);
  member_scene_graph().ExcludeCollisionsBetween(visual, collision);
}

template<typename T>
void MultibodyPlant<T>::CalcNormalAndTangentContactJacobians(
    const systems::Context<T>& context,
    const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs_set,
    MatrixX<T>* Jn_ptr, MatrixX<T>* Jt_ptr,
    std::vector<RotationMatrix<T>>* R_WC_set) const {
  DRAKE_DEMAND(Jn_ptr != nullptr);
  DRAKE_DEMAND(Jt_ptr != nullptr);

  const int num_contacts = point_pairs_set.size();

  // Jn is defined such that vn = Jn * v, with vn of size nc.
  auto& Jn = *Jn_ptr;
  Jn.resize(num_contacts, num_velocities());

  // Jt is defined such that vt = Jt * v, with vt of size 2nc.
  auto& Jt = *Jt_ptr;
  Jt.resize(2 * num_contacts, num_velocities());

  if (R_WC_set != nullptr) R_WC_set->clear();

  // Quick no-op exit. Notice we did resize Jn, Jt and R_WC_set to be zero
  // sized.
  if (num_contacts == 0) return;

  const Frame<T>& frame_W = world_frame();
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = point_pairs_set[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const Body<T>& bodyA = get_body(bodyA_index);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);
    const Body<T>& bodyB = get_body(bodyB_index);

    // Penetration depth, > 0 if bodies interpenetrate.
    const Vector3<T>& nhat_BA_W = point_pair.nhat_BA_W;
    const Vector3<T>& p_WCa = point_pair.p_WCa;
    const Vector3<T>& p_WCb = point_pair.p_WCb;

    // TODO(amcastro-tri): Consider using the midpoint between Ac and Bc for
    // stability reasons. Besides that, there is no other reason to use the
    // midpoint (or any other point between Ac and Bc for that matter) since,
    // in the limit to rigid contact, Ac = Bc.

    // For contact point Ac of (fixed to) body A, calculate `Jv_v_WAc` (Ac's
    // translational velocity Jacobian in the world frame W with respect to
    // generalized velocities v).  Note: Ac's translational velocity in W can
    // be written in terms of this Jacobian as `v_WAc = Jv_v_WAc * v`.
    Matrix3X<T> Jv_WAc(3, this->num_velocities());
    internal_tree().CalcJacobianTranslationalVelocity(context,
                                                      JacobianWrtVariable::kV,
                                                      bodyA.body_frame(),
                                                      frame_W,
                                                      p_WCa,
                                                      frame_W,
                                                      frame_W,
                                                      &Jv_WAc);

    // Similarly, for contact point Bc of body B, calculate `Jv_v_WBc
    // (Bc's translational velocity Jacobian in W with respect to v)
    Matrix3X<T> Jv_WBc(3, this->num_velocities());
    internal_tree().CalcJacobianTranslationalVelocity(context,
                                                      JacobianWrtVariable::kV,
                                                      bodyB.body_frame(),
                                                      frame_W,
                                                      p_WCb,
                                                      frame_W,
                                                      frame_W,
                                                      &Jv_WBc);

    // Computation of the normal separation velocities Jacobian Jn:
    //
    // The velocity of Bc relative to Ac is
    //   v_AcBc_W = v_WBc - v_WAc.
    // From where the separation velocity is computed as
    //   vn = -v_AcBc_W.dot(nhat_BA_W) = -nhat_BA_Wᵀ⋅v_AcBc_W
    // where the negative sign stems from the sign convention for vn and xdot.
    // This can be written in terms of the Jacobians as
    //   vn = -nhat_BA_Wᵀ⋅(Jv_WBc - Jv_WAc)⋅v
    Jn.row(icontact) = nhat_BA_W.transpose() * (Jv_WAc - Jv_WBc);

    // Computation of the tangential velocities Jacobian Jt:
    //
    // Compute the orientation of a contact frame C at the contact point such
    // that the z-axis Cz equals to nhat_BA_W. The tangent vectors are
    // arbitrary, with the only requirement being that they form a valid right
    // handed basis with nhat_BA.
    const RotationMatrix<T> R_WC(math::ComputeBasisFromAxis(2, nhat_BA_W));
    if (R_WC_set != nullptr) {
      R_WC_set->push_back(R_WC);
    }

    const Vector3<T> that1_W = R_WC.matrix().col(0);  // that1 = Cx.
    const Vector3<T> that2_W = R_WC.matrix().col(1);  // that2 = Cy.

    // The velocity of Bc relative to Ac is
    //   v_AcBc_W = v_WBc - v_WAc.
    // The first two components of this velocity in C corresponds to the
    // tangential velocities in a plane normal to nhat_BA.
    //   vx_AcBc_C = that1⋅v_AcBc = that1ᵀ⋅(Jv_WBc - Jv_WAc)⋅v
    //   vy_AcBc_C = that2⋅v_AcBc = that2ᵀ⋅(Jv_WBc - Jv_WAc)⋅v
    Jt.row(2 * icontact)     = that1_W.transpose() * (Jv_WBc - Jv_WAc);
    Jt.row(2 * icontact + 1) = that2_W.transpose() * (Jv_WBc - Jv_WAc);
  }
}

template<typename T>
void MultibodyPlant<T>::set_penetration_allowance(
    double penetration_allowance) {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
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
    const Body<T>& body = get_body(body_index);
    mass = std::max(mass, body.get_default_mass());
  }

  // For now, we use the model of a critically damped spring mass oscillator
  // to estimate these parameters: mẍ+cẋ+kx=mg
  // Notice however that normal forces are computed according to: fₙ=kx(1+dẋ)
  // which translate to a second order oscillator of the form:
  // mẍ+(kdx)ẋ+kx=mg
  // Therefore, for this more complex, non-linear, oscillator, we estimate the
  // damping constant d using a time scale related to the free oscillation
  // (omega below) and the requested penetration allowance as a length scale.

  // We first estimate the stiffness based on static equilibrium.
  const double stiffness = mass * g / penetration_allowance;
  // Frequency associated with the stiffness above.
  const double omega = sqrt(stiffness / mass);

  // Estimated contact time scale. The relative velocity of objects coming into
  // contact goes to zero in this time scale.
  const double time_scale = 1.0 / omega;

  // Damping ratio for a critically damped model. We could allow users to set
  // this. Right now, critically damp the normal direction.
  // This corresponds to a non-penetraion constraint in the limit for
  // contact_penetration_allowance_ goint to zero (no bounce off).
  const double damping_ratio = 1.0;
  // We form the damping (with units of 1/velocity) using dimensional analysis.
  // Thus we use 1/omega for the time scale and penetration_allowance for the
  // length scale. We then scale it by the damping ratio.
  const double damping = damping_ratio * time_scale / penetration_allowance;

  // Final parameters used in the penalty method:
  penalty_method_contact_parameters_.stiffness = stiffness;
  penalty_method_contact_parameters_.damping = damping;
  // The time scale can be requested to hint the integrator's time step.
  penalty_method_contact_parameters_.time_scale = time_scale;
}

// Specialize this function so that double is fully supported.
template <>
std::vector<PenetrationAsPointPair<double>>
MultibodyPlant<double>::CalcPointPairPenetrations(
    const systems::Context<double>& context) const {
  if (num_collision_geometries() > 0) {
    if (!geometry_query_port_.is_valid()) {
      throw std::logic_error(
          "This MultibodyPlant registered geometry for contact handling. "
          "However its query input port (get_geometry_query_input_port()) "
          "is not connected.");
    }
    const auto& query_object = get_geometry_query_input_port().
        Eval<geometry::QueryObject<double>>(context);
    return query_object.ComputePointPairPenetration();
  }
  return std::vector<PenetrationAsPointPair<double>>();
}

// Specialize this function so that AutoDiffXd is (partially) supported. This
// AutoDiffXd specialization will throw if there are any collisions.
// TODO(SeanCurtis-TRI): Move this logic into SceneGraph per #11454.
template <>
std::vector<PenetrationAsPointPair<AutoDiffXd>>
MultibodyPlant<AutoDiffXd>::CalcPointPairPenetrations(
    const systems::Context<AutoDiffXd>& context) const {
  if (num_collision_geometries() > 0) {
    const auto &query_object = get_geometry_query_input_port().
        Eval<geometry::QueryObject<AutoDiffXd>>(context);
    auto results = query_object.ComputePointPairPenetration();
    if (results.size() > 0) {
      throw std::logic_error(
          "CalcPointPairPenetration(): Some of the bodies in the model are in "
          "contact for the state stored in the given context. Currently a "
          "MultibodyPlant model cannot be auto-differentiated if contacts "
          "are detected. Notice however that auto-differentiation is allowed "
          "if there are no contacts for the given state. That is, you can "
          "invoke penetration queries on a MultibodyPlant<AutoDiffXd> as long "
          "as there are no unfiltered geometries in contact. "
          "Refer to Github issues #11454 and #11455 for details.");
    }
  }
  return {};
}

template<typename T>
std::vector<PenetrationAsPointPair<T>>
MultibodyPlant<T>::CalcPointPairPenetrations(const systems::Context<T>&) const {
  throw std::domain_error(fmt::format("This method doesn't support T = {}.",
                                      NiceTypeName::Get<T>()));
}

template<typename T>
std::vector<CoulombFriction<double>>
MultibodyPlant<T>::CalcCombinedFrictionCoefficients(
    const std::vector<PenetrationAsPointPair<T>>& point_pairs) const {
  std::vector<CoulombFriction<double>> combined_frictions;
  combined_frictions.reserve(point_pairs.size());
  for (const auto& pair : point_pairs) {
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const int collision_indexA =
        geometry_id_to_collision_index_.at(geometryA_id);
    const int collision_indexB =
        geometry_id_to_collision_index_.at(geometryB_id);
    const CoulombFriction<double>& geometryA_friction =
        default_coulomb_friction_[collision_indexA];
    const CoulombFriction<double>& geometryB_friction =
        default_coulomb_friction_[collision_indexB];

    combined_frictions.push_back(CalcContactFrictionFromSurfaceProperties(
        geometryA_friction, geometryB_friction));
  }
  return combined_frictions;
}

template<typename T>
void MultibodyPlant<T>::CopyContactResultsOutput(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);
  *contact_results = EvalContactResults(context);
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsContinuous(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);
  contact_results->Clear();
  if (num_collision_geometries() == 0) return;

  switch (contact_model_) {
    case ContactModel::kPointContactOnly:
      CalcContactResultsContinuousPointPair(context, contact_results);
      break;

    case ContactModel::kHydroelasticsOnly:
      CalcContactResultsContinuousHydroelastic(context, contact_results);
      break;
  }
}

template <>
void MultibodyPlant<symbolic::Expression>::
    CalcContactResultsContinuousHydroelastic(
        const Context<symbolic::Expression>&,
        ContactResults<symbolic::Expression>*) const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsContinuousHydroelastic(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  const HydroelasticContactInfoAndBodySpatialForces&
      contact_info_and_spatial_body_forces =
          EvalHydroelasticContactForces(context);
  for (const HydroelasticContactInfo<T>& contact_info :
       contact_info_and_spatial_body_forces.contact_info) {
    // Note: caching dependencies guarantee that the lifetime of contact_info is
    // valid for the lifetime of the contact results.
    contact_results->AddContactInfo(&contact_info);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsContinuousPointPair(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      EvalPointPairPenetrations(context);

  const std::vector<CoulombFriction<double>> combined_friction_pairs =
      CalcCombinedFrictionCoefficients(point_pairs);

  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);
  const internal::VelocityKinematicsCache<T>& vc =
      EvalVelocityKinematics(context);

  contact_results->Clear();
  for (size_t icontact = 0; icontact < point_pairs.size(); ++icontact) {
    const auto& pair = point_pairs[icontact];
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    internal::BodyNodeIndex bodyA_node_index =
        get_body(bodyA_index).node_index();
    internal::BodyNodeIndex bodyB_node_index =
        get_body(bodyB_index).node_index();

    // Penetration depth, > 0 during pair.
    const T& x = pair.depth;
    DRAKE_ASSERT(x >= 0);
    const Vector3<T>& nhat_BA_W = pair.nhat_BA_W;
    const Vector3<T>& p_WCa = pair.p_WCa;
    const Vector3<T>& p_WCb = pair.p_WCb;

    // Contact point C.
    const Vector3<T> p_WC = 0.5 * (p_WCa + p_WCb);

    // Contact point position on body A.
    const Vector3<T>& p_WAo = pc.get_X_WB(bodyA_node_index).translation();
    const Vector3<T>& p_CoAo_W = p_WAo - p_WC;

    // Contact point position on body B.
    const Vector3<T>& p_WBo = pc.get_X_WB(bodyB_node_index).translation();
    const Vector3<T>& p_CoBo_W = p_WBo - p_WC;

    // Separation velocity, > 0  if objects separate.
    const Vector3<T> v_WAc =
        vc.get_V_WB(bodyA_node_index).Shift(-p_CoAo_W).translational();
    const Vector3<T> v_WBc =
        vc.get_V_WB(bodyB_node_index).Shift(-p_CoBo_W).translational();
    const Vector3<T> v_AcBc_W = v_WBc - v_WAc;

    // if xdot = vn > 0 ==> they are getting closer.
    const T vn = v_AcBc_W.dot(nhat_BA_W);

    // Magnitude of the normal force on body A at contact point C.
    const T k = penalty_method_contact_parameters_.stiffness;
    const T d = penalty_method_contact_parameters_.damping;
    const T fn_AC = k * x * (1.0 + d * vn);

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
            slip_velocity, combined_friction_pairs[icontact]);
        // Tangential direction.
        const Vector3<T> that_W = vt_AcBc_W / slip_velocity;

        // Magnitude of the friction force on A at C.
        const T ft_AC = mu_stribeck * fn_AC;
        ft_AC_W = ft_AC * that_W;
      }

      // Spatial force on body A at C, expressed in the world frame W.
      const SpatialForce<T> F_AC_W(Vector3<T>::Zero(), fn_AC_W + ft_AC_W);

      const Vector3<T> f_Bc_W = -F_AC_W.translational();
      contact_results->AddContactInfo(
          {bodyA_index, bodyB_index, f_Bc_W, p_WC, vn, slip_velocity, pair});
    }
  }
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsDiscrete(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);
  if (num_collision_geometries() == 0) return;

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      EvalPointPairPenetrations(context);
  const std::vector<RotationMatrix<T>>& R_WC_set =
      EvalContactJacobians(context).R_WC_list;
  const internal::TamsiSolverResults<T>& solver_results =
      EvalTamsiResults(context);

  const VectorX<T>& fn = solver_results.fn;
  const VectorX<T>& ft = solver_results.ft;
  const VectorX<T>& vt = solver_results.vt;
  const VectorX<T>& vn = solver_results.vn;

  const int num_contacts = point_pairs.size();
  DRAKE_DEMAND(fn.size() == num_contacts);
  DRAKE_DEMAND(ft.size() == 2 * num_contacts);
  DRAKE_DEMAND(vn.size() == num_contacts);
  DRAKE_DEMAND(vt.size() == 2 * num_contacts);

  contact_results->Clear();
  for (size_t icontact = 0; icontact < point_pairs.size(); ++icontact) {
    const auto& pair = point_pairs[icontact];
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    const Vector3<T> p_WC = 0.5 * (pair.p_WCa + pair.p_WCb);

    const RotationMatrix<T>& R_WC = R_WC_set[icontact];

    // Contact forces applied on B at contact point C.
    const Vector3<T> f_Bc_C(ft(2 * icontact), ft(2 * icontact + 1),
                            -fn(icontact));
    const Vector3<T> f_Bc_W = R_WC * f_Bc_C;

    // Slip velocity.
    const T slip = vt.template segment<2>(2 * icontact).norm();

    // Separation velocity in the normal direction.
    const T separation_velocity = vn(icontact);

    // Add pair info to the contact results.
    contact_results->AddContactInfo({bodyA_index, bodyB_index, f_Bc_W, p_WC,
                                     separation_velocity, slip, pair});
  }
}

template <typename T>
void MultibodyPlant<T>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  DRAKE_DEMAND(F_BBo_W_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(F_BBo_W_array->size()) == num_bodies());
  if (num_collision_geometries() == 0) return;

  const ContactResults<T>& contact_results = EvalContactResults(context);

  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);

  for (int pair_index = 0;
       pair_index < contact_results.num_point_pair_contacts(); ++pair_index) {
    const PointPairContactInfo<T>& contact_info =
        contact_results.point_pair_contact_info(pair_index);
    const PenetrationAsPointPair<T>& pair = contact_info.point_pair();

    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    internal::BodyNodeIndex bodyA_node_index =
        get_body(bodyA_index).node_index();
    internal::BodyNodeIndex bodyB_node_index =
        get_body(bodyB_index).node_index();

    // Contact point C.
    const Vector3<T> p_WC = contact_info.contact_point();

    // Contact point position on body A.
    const Vector3<T>& p_WAo = pc.get_X_WB(bodyA_node_index).translation();
    const Vector3<T>& p_CoAo_W = p_WAo - p_WC;

    // Contact point position on body B.
    const Vector3<T>& p_WBo = pc.get_X_WB(bodyB_node_index).translation();
    const Vector3<T>& p_CoBo_W = p_WBo - p_WC;

    const Vector3<T> f_Bc_W = contact_info.contact_force();
    const SpatialForce<T> F_AC_W(Vector3<T>::Zero(), -f_Bc_W);

    if (bodyA_index != world_index()) {
      // Spatial force on body A at Ao, expressed in W.
      const SpatialForce<T> F_AAo_W = F_AC_W.Shift(p_CoAo_W);
      F_BBo_W_array->at(bodyA_node_index) += F_AAo_W;
    }

    if (bodyB_index != world_index()) {
      // Spatial force on body B at Bo, expressed in W.
      const SpatialForce<T> F_BBo_W = -F_AC_W.Shift(p_CoBo_W);
      F_BBo_W_array->at(bodyB_node_index) += F_BBo_W;
    }
  }
}

template <>
void MultibodyPlant<symbolic::Expression>::MakeHydroelasticModels() {
  // This is a no-op for symbolic::Expression to allow building the plant even
  // if the user requested the use of the hydroelastic model.
  // However an exception will be thrown at runtime if hydroelastic force
  // computations are invoked with symbolic::Expression.
}

template <typename T>
void MultibodyPlant<T>::MakeHydroelasticModels() {
  hydroelastics_engine_.MakeModels(scene_graph_->model_inspector());
}

template <>
void MultibodyPlant<symbolic::Expression>::CalcHydroelasticContactForces(
    const Context<symbolic::Expression>&,
    HydroelasticContactInfoAndBodySpatialForces*) const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template <typename T>
void MultibodyPlant<T>::CalcHydroelasticContactForces(
    const Context<T>& context,
    HydroelasticContactInfoAndBodySpatialForces* contact_info_and_body_forces)
    const {
  DRAKE_DEMAND(contact_info_and_body_forces != nullptr);

  std::vector<SpatialForce<T>>& F_BBo_W_array =
      contact_info_and_body_forces->F_BBo_W_array;
  DRAKE_DEMAND(static_cast<int>(F_BBo_W_array.size()) == num_bodies());
  std::vector<HydroelasticContactInfo<T>>& contact_info =
      contact_info_and_body_forces->contact_info;

  // Initialize the body forces to zero.
  F_BBo_W_array.assign(num_bodies(), SpatialForce<T>::Zero());
  if (num_collision_geometries() == 0) return;

  const std::vector<ContactSurface<T>>& all_surfaces =
      EvalContactSurfaces(context);

  // Reserve memory here to keep from repeatedly allocating heap storage in the
  // loop below.
  contact_info.clear();
  contact_info.reserve(all_surfaces.size());

  internal::HydroelasticTractionCalculator<T> traction_calculator(
      friction_model_.stiction_tolerance());

  for (const ContactSurface<T>& surface : all_surfaces) {
    const GeometryId geometryM_id = surface.id_M();
    const GeometryId geometryN_id = surface.id_N();
    const int collision_indexM =
        geometry_id_to_collision_index_.at(geometryM_id);
    const int collision_indexN =
        geometry_id_to_collision_index_.at(geometryN_id);
    const CoulombFriction<double>& geometryM_friction =
        default_coulomb_friction_[collision_indexM];
    const CoulombFriction<double>& geometryN_friction =
        default_coulomb_friction_[collision_indexN];

    // Compute combined friction coefficient.
    const CoulombFriction<double> combined_friction =
        CalcContactFrictionFromSurfaceProperties(geometryM_friction,
                                                 geometryN_friction);
    const double dynamic_friction = combined_friction.dynamic_friction();

    // Get the bodies that the two geometries are affixed to. We'll call these
    // A and B.
    const BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryM_id);
    const BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryN_id);
    const Body<T>& bodyA = get_body(bodyA_index);
    const Body<T>& bodyB = get_body(bodyB_index);

    // The the poses and spatial velocities of bodies A and B.
    const RigidTransform<T>& X_WA = bodyA.EvalPoseInWorld(context);
    const RigidTransform<T>& X_WB = bodyB.EvalPoseInWorld(context);
    const SpatialVelocity<T>& V_WA = bodyA.EvalSpatialVelocityInWorld(context);
    const SpatialVelocity<T>& V_WB = bodyB.EvalSpatialVelocityInWorld(context);

    // Pack everything calculator needs.
    typename internal::HydroelasticTractionCalculator<T>::Data data(
        X_WA, X_WB, V_WA, V_WB, &surface);

    // Combined Hunt & Crossley dissipation.
    const double dissipation = hydroelastics_engine_.CalcCombinedDissipation(
        geometryM_id, geometryN_id);

    // Integrate the hydroelastic traction field over the contact surface.
    std::vector<HydroelasticQuadraturePointData<T>> traction_output;
    SpatialForce<T> F_Ac_W;
    traction_calculator.ComputeSpatialForcesAtCentroidFromHydroelasticModel(
        data, dissipation, dynamic_friction, &traction_output, &F_Ac_W);

    // Shift the traction at the centroid to tractions at the body origins.
    SpatialForce<T> F_Ao_W, F_Bo_W;
    traction_calculator.ShiftSpatialForcesAtCentroidToBodyOrigins(
        data, F_Ac_W, &F_Ao_W, &F_Bo_W);

    if (bodyA_index != world_index()) {
      F_BBo_W_array.at(bodyA.node_index()) += F_Ao_W;
    }

    if (bodyB_index != world_index()) {
      F_BBo_W_array.at(bodyB.node_index()) += F_Bo_W;
    }

    // Add the information for contact reporting.
    contact_info.emplace_back(&surface, F_Ac_W, std::move(traction_output));
  }
}

template<typename T>
void MultibodyPlant<T>::AddAppliedExternalSpatialForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  // Get the mutable applied external spatial forces vector
  // (a.k.a., body force vector).
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces->mutable_body_forces();

  // Evaluate the input port; if it's not connected, return now.
  const auto* applied_input = this->template EvalInputValue<
      std::vector<ExternallyAppliedSpatialForce<T>>>(
          context, applied_spatial_force_input_port_);
  if (!applied_input)
    return;

  // Loop over all forces.
  for (const auto& force_structure : *applied_input) {
    const BodyIndex body_index = force_structure.body_index;
    const Body<T>& body = get_body(body_index);
    const auto body_node_index = body.node_index();

    // Get the pose for this body in the world frame.
    const RigidTransform<T>& X_WB = EvalBodyPoseInWorld(context, body);

    // Get the position vector from the body origin (Bo) to the point of
    // force application (Bq), expressed in the world frame (W).
    const Vector3<T> p_BoBq_W = X_WB.rotation() * force_structure.p_BoBq_B;

    // Shift the spatial force from Bq to Bo.
    F_BBo_W_array[body_node_index] += force_structure.F_Bq_W.Shift(-p_BoBq_W);
  }
}

template<typename T>
void MultibodyPlant<T>::AddJointActuationForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  if (num_actuators() > 0) {
    const VectorX<T> u = AssembleActuationInput(context);
    for (JointActuatorIndex actuator_index(0);
         actuator_index < num_actuators(); ++actuator_index) {
      const JointActuator<T>& actuator =
          get_joint_actuator(actuator_index);
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(actuator.joint().num_velocities() == 1);
      for (int joint_dof = 0;
           joint_dof < actuator.joint().num_velocities(); ++joint_dof) {
        actuator.AddInOneForce(context, joint_dof, u[actuator_index], forces);
      }
    }
  }
}

template<typename T>
void MultibodyPlant<T>::AddJointLimitsPenaltyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  DRAKE_THROW_UNLESS(is_discrete());
  DRAKE_DEMAND(forces != nullptr);

  auto CalcPenaltyForce = [](
      double lower_limit, double upper_limit, double stiffness, double damping,
      const T& q, const T& v) {
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

    const T penalty_force = CalcPenaltyForce(
        lower_limit, upper_limit, stiffness, damping, q, v);

    joint.AddInOneForce(context, 0, penalty_force, forces);
  }
}

template<typename T>
VectorX<T> MultibodyPlant<T>::AssembleActuationInput(
    const systems::Context<T>& context) const {
  // Assemble the vector from the model instance input ports.
  VectorX<T> actuation_input(num_actuated_dofs());
  int u_offset = 0;
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    // Ignore the port if the model instance has no actuated DoFs.
    const int instance_num_dofs = num_actuated_dofs(model_instance_index);
    if (instance_num_dofs == 0) continue;

    const auto& input_port = this->get_input_port(
        instance_actuation_ports_[model_instance_index]);
    if (!input_port.HasValue(context)) {
        throw std::logic_error(fmt::format("Actuation input port for model "
            "instance {} must be connected.",
            GetModelInstanceName(model_instance_index)));
    }

    const auto& u_instance = input_port.Eval(context);
    actuation_input.segment(u_offset, instance_num_dofs) = u_instance;
    u_offset += instance_num_dofs;
  }
  DRAKE_ASSERT(u_offset == num_actuated_dofs());
  return actuation_input;
}

template<typename T>
void MultibodyPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // No derivatives to compute if state is discrete.
  if (is_discrete()) return;
  // No derivatives to compute if state is empty. (Will segfault otherwise.)
  // TODO(amcastro-tri): When nv = 0 we should not declare state or cache
  // entries at all and the system framework will never call this.
  if (this->num_multibody_states() == 0) return;

  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const auto v = x.bottomRows(this->num_velocities());

  const VectorX<T>& vdot = EvalGeneralizedAccelerations(context);

  VectorX<T> xdot(this->num_multibody_states());
  VectorX<T> qdot(this->num_positions());
  MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
TamsiSolverResult MultibodyPlant<T>::SolveUsingSubStepping(
    int num_substeps,
    const MatrixX<T>& M0, const MatrixX<T>& Jn, const MatrixX<T>& Jt,
    const VectorX<T>& minus_tau,
    const VectorX<T>& stiffness, const VectorX<T>& damping,
    const VectorX<T>& mu,
    const VectorX<T>& v0, const VectorX<T>& phi0) const {

  const double dt = time_step_;  // just a shorter alias.
  const double dt_substep = dt / num_substeps;
  VectorX<T> v0_substep = v0;
  VectorX<T> phi0_substep = phi0;

  // Initialize info to an unsuccessful result.
  TamsiSolverResult info{
      TamsiSolverResult::kMaxIterationsReached};

  for (int substep = 0; substep < num_substeps; ++substep) {
    // Discrete update before applying friction forces.
    // We denote this state x* = [q*, v*], the "star" state.
    // Generalized momentum "star", before contact forces are applied.
    VectorX<T> p_star_substep = M0 * v0_substep - dt_substep * minus_tau;

    // Update the data.
    tamsi_solver_->SetTwoWayCoupledProblemData(
        &M0, &Jn, &Jt,
        &p_star_substep, &phi0_substep,
        &stiffness, &damping, &mu);

    info = tamsi_solver_->SolveWithGuess(dt_substep,
                                                     v0_substep);

    // Break the sub-stepping loop on failure and return the info result.
    if (info != TamsiSolverResult::kSuccess) break;

    // Update previous time step to new solution.
    v0_substep = tamsi_solver_->get_generalized_velocities();

    // Update penetration distance consistently with the solver update.
    const auto vn_substep =
        tamsi_solver_->get_normal_velocities();
    phi0_substep = phi0_substep - dt_substep * vn_substep;
  }

  return info;
}

template <typename T>
void MultibodyPlant<T>::CalcContactSurfaces(
    const drake::systems::Context<T>& context,
    std::vector<ContactSurface<T>>* contact_surfaces) const {
  DRAKE_DEMAND(contact_surfaces);

  const auto& query_object =
      this->get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);

  *contact_surfaces =
      hydroelastics_engine_.ComputeContactSurfaces(query_object);
}

template <typename T>
void MultibodyPlant<T>::CalcAppliedForces(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(*this));

  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);
  const internal::VelocityKinematicsCache<T>& vc =
      EvalVelocityKinematics(context);

  // Compute forces applied through force elements.
  internal_tree().CalcForceElementsContribution(context, pc, vc, forces);

  // Externally applied forces.
  AddJointActuationForces(context, forces);
  AddAppliedExternalSpatialForces(context, forces);

  // Only discrete models support joint limits.
  if (is_discrete()) AddJointLimitsPenaltyForces(context, forces);

  // If there are applied generalized forces, add them.
  const InputPort<T>& applied_generalized_force_input =
      this->get_input_port(applied_generalized_force_input_port_);
  if (applied_generalized_force_input.HasValue(context)) {
    forces->mutable_generalized_forces() +=
        applied_generalized_force_input.Eval(context);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcTamsiResults(
    const drake::systems::Context<T>& context0,
    internal::TamsiSolverResults<T>* results) const {
  // Assert this method was called on a context storing discrete state.
  DRAKE_ASSERT(context0.num_discrete_state_groups() == 1);
  DRAKE_ASSERT(context0.num_continuous_states() == 0);

  const int nq = this->num_positions();
  const int nv = this->num_velocities();

  // Quick exit if there are no moving objects.
  if (nv == 0) return;

  // Get the system state as raw Eigen vectors
  // (solution at the previous time step).
  auto x0 = context0.get_discrete_state(0).get_value();
  VectorX<T> q0 = x0.topRows(nq);
  VectorX<T> v0 = x0.bottomRows(nv);

  // Mass matrix and its factorization.
  MatrixX<T> M0(nv, nv);
  internal_tree().CalcMassMatrixViaInverseDynamics(context0, &M0);
  auto M0_ldlt = M0.ldlt();

  // Forces at the previous time step.
  MultibodyForces<T> forces0(internal_tree());

  CalcAppliedForces(context0, &forces0);

  // Workspace for inverse dynamics:
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Body forces (alias to forces0).
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces0.mutable_body_forces();

  // With vdot = 0, this computes:
  //   -tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  VectorX<T>& minus_tau = forces0.mutable_generalized_forces();
  internal_tree().CalcInverseDynamics(
      context0, vdot,
      F_BBo_W_array, minus_tau,
      &A_WB_array,
      &F_BBo_W_array, /* Note: these arrays get overwritten on output. */
      &minus_tau);

  // Compute normal and tangential velocity Jacobians at t0.
  const std::vector<PenetrationAsPointPair<T>>& point_pairs0 =
      EvalPointPairPenetrations(context0);
  const int num_contacts = point_pairs0.size();
  const internal::ContactJacobians<T>& contact_jacobians =
      EvalContactJacobians(context0);

  // Get friction coefficient into a single vector. Static friction is ignored
  // by the time stepping scheme.
  std::vector<CoulombFriction<double>> combined_friction_pairs =
      CalcCombinedFrictionCoefficients(point_pairs0);
  VectorX<T> mu(num_contacts);
  std::transform(combined_friction_pairs.begin(), combined_friction_pairs.end(),
                 mu.data(),
                 [](const CoulombFriction<double>& coulomb_friction) {
                   return coulomb_friction.dynamic_friction();
                 });

  // Place all the penetration depths within a single vector as required by
  // the solver.
  VectorX<T> phi0(num_contacts);
  std::transform(point_pairs0.begin(), point_pairs0.end(),
                 phi0.data(),
                 [](const PenetrationAsPointPair<T>& pair) {
                   return pair.depth;
                 });

  // TODO(amcastro-tri): Consider using different penalty parameters at each
  // contact point.
  // Compliance parameters used by the solver for each contact point.
  VectorX<T> stiffness = VectorX<T>::Constant(
      num_contacts, penalty_method_contact_parameters_.stiffness);
  VectorX<T> damping = VectorX<T>::Constant(
      num_contacts, penalty_method_contact_parameters_.damping);

  // Solve for v and the contact forces.
  TamsiSolverResult info{
      TamsiSolverResult::kMaxIterationsReached};

  TamsiSolverParameters params =
      tamsi_solver_->get_solver_parameters();
  // A nicely converged NR iteration should not take more than 20 iterations.
  // Otherwise we attempt a smaller time step.
  params.max_iterations = 20;
  tamsi_solver_->set_solver_parameters(params);

  // We attempt to compute the update during the time interval dt using a
  // progressively larger number of sub-steps (i.e each using a smaller time
  // step than in the previous attempt). This loop breaks on the first
  // successful attempt.
  // We only allow a maximum number of trials. If the solver is unsuccessful in
  // this number of trials, the user should probably decrease the discrete
  // update time step dt or evaluate the validity of the model.
  const int kNumMaxSubTimeSteps = 20;
  int num_substeps = 0;
  do {
    ++num_substeps;
    info = SolveUsingSubStepping(num_substeps, M0, contact_jacobians.Jn,
                                 contact_jacobians.Jt, minus_tau, stiffness,
                                 damping, mu, v0, phi0);
  } while (info != TamsiSolverResult::kSuccess &&
           num_substeps < kNumMaxSubTimeSteps);

  if (info != TamsiSolverResult::kSuccess) {
    const std::string msg = fmt::format(
        "MultibodyPlant's discrete update solver failed to converge at "
        "simulation time = {:7.3f} with discrete update period = {:7.3f}. This "
        "usually means that the plant's discrete update period is too large to "
        "resolve the system's dynamics for the given simulation conditions. "
        "This is often the case during abrupt collisions or during complex "
        "and fast changing contact configurations. Another common cause is the "
        "use of high gains in the simulation of closed loop systems. These "
        "might cause numerical instabilities given our discrete solver uses an "
        "explicit treatment of actuation inputs. Possible solutions include:\n"
        "  1. reduce the discrete update period set at construction,\n"
        "  2. decrease the high gains in your controller whenever possible,\n"
        "  3. switch to a continuous model (discrete update period is zero), "
        "     though this might affect the simulation run time.",
        context0.get_time(), this->time_step());
    throw std::runtime_error(msg);
  }

  // TODO(amcastro-tri): implement capability to dump solver statistics to a
  // file for analysis.

  // Update the results.
  results->v_next = tamsi_solver_->get_generalized_velocities();
  results->fn = tamsi_solver_->get_normal_forces();
  results->ft = tamsi_solver_->get_friction_forces();
  results->vn = tamsi_solver_->get_normal_velocities();
  results->vt = tamsi_solver_->get_tangential_velocities();
  results->tau_contact =
      tamsi_solver_->get_generalized_contact_forces();
}

template <typename T>
void MultibodyPlant<T>::CalcArticulatedBodyForceBiasCache(
    const systems::Context<T>& context,
    ArticulatedBodyForceBiasCache<T>* aba_force_bias_cache) const {
  DRAKE_DEMAND(aba_force_bias_cache != nullptr);

  // Applied forces including force elements (function of state x) and external
  // inputs u.
  MultibodyForces<T> forces(*this);
  CalcAppliedForces(context, &forces);

  // Add the contribution of contact forces.
  std::vector<SpatialForce<T>>& Fapp_BBo_W_array = forces.mutable_body_forces();
  const std::vector<SpatialForce<T>>& Fcontact_BBo_W_array =
      EvalSpatialContactForcesContinuous(context);
  for (int i = 0; i < static_cast<int>(Fapp_BBo_W_array.size()); ++i)
    Fapp_BBo_W_array[i] += Fcontact_BBo_W_array[i];

  // Perform the tip-to-base pass to compute the force bias terms needed by ABA.
  internal_tree().CalcArticulatedBodyForceBiasCache(context, forces,
                                                    aba_force_bias_cache);
}

template <typename T>
void MultibodyPlant<T>::CalcForwardDynamics(
    const systems::Context<T>& context,
    AccelerationKinematicsCache<T>* ac) const {
  DRAKE_DEMAND(ac != nullptr);

  // Evaluate the ABA cache, function of state x and inputs u.
  const ArticulatedBodyForceBiasCache<T>& aba_force_bias_cache =
      EvalArticulatedBodyForceBiasCache(context);

  // Perform the last base-to-tip pass to compute accelerations using the O(n)
  // ABA.
  internal_tree().CalcArticulatedBodyAccelerations(context,
                                                   aba_force_bias_cache, ac);
}

template <typename T>
void MultibodyPlant<T>::CalcGeneralizedAccelerations(
    const drake::systems::Context<T>& context, VectorX<T>* vdot) const {
  DRAKE_DEMAND(vdot != nullptr);
  DRAKE_DEMAND(vdot->size() == num_velocities());
  if (is_discrete())
    CalcGeneralizedAccelerationsDiscrete(context, vdot);
  else
    CalcGeneralizedAccelerationsContinuous(context, vdot);
}

template <typename T>
void MultibodyPlant<T>::CalcGeneralizedContactForcesContinuous(
    const Context<T>& context, VectorX<T>* tau_contact) const {
  DRAKE_DEMAND(tau_contact);
  DRAKE_DEMAND(tau_contact->size() == num_velocities());
  DRAKE_DEMAND(!is_discrete());
  const int nv = this->num_velocities();

  // Early exit if there are no contact forces.
  tau_contact->setZero();
  if (num_collision_geometries() == 0) return;

  // We will alias this zero vector to serve both as zero-valued generalized
  // accelerations and zero-valued externally applied generalized forces.
  const VectorX<T> zero = VectorX<T>::Zero(nv);
  const VectorX<T>& zero_vdot = zero;
  const VectorX<T>& tau_array = zero;

  // Get the spatial forces.
  const std::vector<SpatialForce<T>>& Fcontact_BBo_W_array =
      EvalSpatialContactForcesContinuous(context);

  // Bodies' accelerations and inboard mobilizer reaction forces, respectively,
  // ordered by BodyNodeIndex and required as output arguments for
  // CalcInverseDynamics() below but otherwise not used by this method.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_bodies());

  // With vdot = 0, this computes:
  //   tau_contact = - ∑ J_WBᵀ(q) Fcontact_Bo_W.
  internal_tree().CalcInverseDynamics(
      context, zero_vdot, Fcontact_BBo_W_array, tau_array,
      true /* Do not compute velocity-dependent terms */,
      &A_WB_array, &F_BMo_W_array, tau_contact);

  // Per above, tau_contact must be negated to get ∑ J_WBᵀ(q) Fcontact_Bo_W.
  (*tau_contact) = -(*tau_contact);
}

template <typename T>
void MultibodyPlant<T>::CalcSpatialContactForcesContinuous(
      const drake::systems::Context<T>& context,
      std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  DRAKE_DEMAND(F_BBo_W_array);
  DRAKE_DEMAND(static_cast<int>(F_BBo_W_array->size()) == num_bodies());
  DRAKE_DEMAND(!is_discrete());

  // Forces can accumulate into F_BBo_W_array; initialize it to zero first.
  std::fill(F_BBo_W_array->begin(), F_BBo_W_array->end(),
            SpatialForce<T>::Zero());

  // Early exit if there are no contact forces.
  if (num_collision_geometries() == 0) return;

  // Note: we don't need to know the applied forces here because we use a
  // regularized friction model whose forces depend only on the current state; a
  // constraint based friction model would require accounting for the applied
  // forces.

  // Compute the spatial forces on each body from contact.
  switch (contact_model_) {
    case ContactModel::kPointContactOnly:
      // Note: consider caching the results from the following method (in which
      // case we would also want to introduce the Eval... naming convention for
      // the method).
      CalcAndAddContactForcesByPenaltyMethod(context, &(*F_BBo_W_array));
      break;

    case ContactModel::kHydroelasticsOnly:
      *F_BBo_W_array = EvalHydroelasticContactForces(context).F_BBo_W_array;
      break;
  }
}

template <typename T>
void MultibodyPlant<T>::CalcGeneralizedAccelerationsContinuous(
    const drake::systems::Context<T>& context, VectorX<T>* vdot) const {
  DRAKE_DEMAND(vdot != nullptr);
  DRAKE_DEMAND(vdot->size() == num_velocities());
  DRAKE_DEMAND(!is_discrete());
  const int nv = this->num_velocities();

  // Allocate workspace. We might want to cache these to avoid allocations.
  // Mass matrix.
  MatrixX<T> M(nv, nv);
  // Forces.
  MultibodyForces<T> forces(internal_tree());
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  // Generalized accelerations.
  VectorX<T> zero_vdot = VectorX<T>::Zero(nv);

  CalcAppliedForces(context, &forces);

  internal_tree().CalcMassMatrixViaInverseDynamics(context, &M);

  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both the array of applied body forces and
  // the array of applied generalized forces get overwritten on output. This is
  // not important in this case since we don't need their values anymore.
  // Please see the documentation for CalcInverseDynamics() for details.

  // Compute all applied forces.
  std::vector<SpatialForce<T>>& Fapp_BBo_W_array = forces.mutable_body_forces();
  const std::vector<SpatialForce<T>>& Fcontact_BBo_W_array =
      EvalSpatialContactForcesContinuous(context);
  for (int i = 0; i < static_cast<int>(Fapp_BBo_W_array.size()); ++i)
    Fapp_BBo_W_array[i] += Fcontact_BBo_W_array[i];
  VectorX<T>& tau_app = forces.mutable_generalized_forces();

  // Aliases for memory foot-print reduction.
  VectorX<T>& tau = tau_app;
  std::vector<SpatialForce<T>>& F_BBo_W_array = Fapp_BBo_W_array;

  // Compute all applied forces as a generalized force. With vdot = 0, this
  // computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_BBo_W.

  internal_tree().CalcInverseDynamics(
      context, zero_vdot, Fapp_BBo_W_array, tau_app, &A_WB_array,
      &F_BBo_W_array, /* Notice "applied" arrays get overwritten on output. */
      &tau);

  *vdot = M.ldlt().solve(-tau);
}

template <typename T>
void MultibodyPlant<T>::CalcGeneralizedAccelerationsDiscrete(
    const drake::systems::Context<T>& context0, VectorX<T>* vdot) const {
  DRAKE_DEMAND(vdot != nullptr);
  DRAKE_DEMAND(vdot->size() == num_velocities());
  DRAKE_DEMAND(is_discrete());

  // Evaluate contact results.
  const internal::TamsiSolverResults<T>& solver_results =
      EvalTamsiResults(context0);

  // Retrieve the solution velocity for the next time step.
  const VectorX<T>& v_next = solver_results.v_next;

  auto x0 = context0.get_discrete_state(0).get_value();
  const VectorX<T> v0 = x0.bottomRows(this->num_velocities());

  *vdot = (v_next - v0) / time_step();
}

template<typename T>
void MultibodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context0,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  // Get the system state as raw Eigen vectors
  // (solution at the previous time step).
  auto x0 = context0.get_discrete_state(0).get_value();
  VectorX<T> q0 = x0.topRows(this->num_positions());
  VectorX<T> v0 = x0.bottomRows(this->num_velocities());

  // For a discrete model this evaluates vdot = (v_next - v0)/time_step() and
  // includes contact forces.
  const VectorX<T>& vdot = EvalGeneralizedAccelerations(context0);

  // TODO(amcastro-tri): Consider replacing this by:
  //   const VectorX<T>& v_next = solver_results.v_next;
  // to avoid additional vector operations.
  const VectorX<T>& v_next = v0 + time_step() * vdot;

  VectorX<T> qdot_next(this->num_positions());
  MapVelocityToQDot(context0, v_next, &qdot_next);
  VectorX<T> q_next = q0 + time_step() * qdot_next;

  VectorX<T> x_next(this->num_multibody_states());
  x_next << q_next, v_next;
  updates->get_mutable_vector(0).SetFromVector(x_next);
}

template<typename T>
void MultibodyPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = num_positions();
  const int nv = num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  VectorX<T> v(nv);
  internal_tree().MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template<typename T>
void MultibodyPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = num_positions();
  const int nv = num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void MultibodyPlant<T>::DeclareStateCacheAndPorts() {
  // The model must be finalized.
  DRAKE_DEMAND(this->is_finalized());

  if (is_discrete()) {
    this->DeclarePeriodicDiscreteUpdate(time_step_);
  }

  DeclareCacheEntries();

  // Declare per model instance actuation ports.
  int num_actuated_instances = 0;
  ModelInstanceIndex last_actuated_instance;
  instance_actuation_ports_.resize(num_model_instances());
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    const int instance_num_dofs = num_actuated_dofs(model_instance_index);
    if (instance_num_dofs > 0) {
      ++num_actuated_instances;
      last_actuated_instance = model_instance_index;
    }
    instance_actuation_ports_[model_instance_index] =
        this->DeclareVectorInputPort(
                GetModelInstanceName(model_instance_index) + "_actuation",
                systems::BasicVector<T>(instance_num_dofs))
            .get_index();
  }

  if (num_actuated_instances == 1) actuated_instance_ = last_actuated_instance;

  // Declare the generalized force input port.
  applied_generalized_force_input_port_ = this->DeclareVectorInputPort(
      "applied_generalized_force",
      systems::BasicVector<T>(num_velocities())).get_index();

  // Declare applied spatial force input force port.
  applied_spatial_force_input_port_ = this->DeclareAbstractInputPort(
        "applied_spatial_force",
        Value<std::vector<ExternallyAppliedSpatialForce<T>>>()).get_index();

  // Declare one output port for the entire state vector.
  // TODO(sherm1) Rename this port to just "state" when #12214 is resolved so
  //              we can deprecate the old port name.
  state_output_port_ =
      this->DeclareVectorOutputPort("continuous_state",
                                    BasicVector<T>(num_multibody_states()),
                                    &MultibodyPlant::CopyContinuousStateOut,
                                    {this->all_state_ticket()})
          .get_index();

  const auto& generalized_accelerations_cache_entry =
      this->get_cache_entry(cache_indexes_.generalized_accelerations);

  // Declare one output port for the entire generalized acceleration vector
  // vdot (length is nv).
  generalized_acceleration_output_port_ =
      this->DeclareVectorOutputPort(
              "generalized_acceleration", BasicVector<T>(num_velocities()),
              [this](const systems::Context<T>& context,
                     systems::BasicVector<T>* result) {
                result->SetFromVector(
                    this->EvalGeneralizedAccelerations(context));
              },
              {generalized_accelerations_cache_entry.ticket()})
          .get_index();

  // Declare per model instance state and acceleration output ports.
  instance_state_output_ports_.resize(num_model_instances());
  instance_generalized_acceleration_output_ports_.resize(num_model_instances());
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    const std::string& instance_name =
        GetModelInstanceName(model_instance_index);

    const int instance_num_states =  // Might be zero.
        num_multibody_states(model_instance_index);
    // TODO(sherm1) Rename these ports to just "_state" when #12214 is resolved
    //              so we can deprecate the old port names.
    instance_state_output_ports_[model_instance_index] =
        this->DeclareVectorOutputPort(
                instance_name + "_continuous_state",
                BasicVector<T>(instance_num_states),
                [this, model_instance_index](
                    const systems::Context<T>& context,
                    systems::BasicVector<T>* result) {
                  this->CopyContinuousStateOut(model_instance_index, context,
                                               result);
                },
                {this->all_state_ticket()})
            .get_index();

    const int instance_num_velocities =  // Might be zero.
        num_velocities(model_instance_index);
    instance_generalized_acceleration_output_ports_[model_instance_index] =
        this->DeclareVectorOutputPort(
                instance_name + "_generalized_acceleration",
                BasicVector<T>(instance_num_velocities),
                [this, model_instance_index](const systems::Context<T>& context,
                                             systems::BasicVector<T>* result) {
                  const auto& vdot =
                      this->EvalGeneralizedAccelerations(context);
                  result->SetFromVector(
                      this->GetVelocitiesFromArray(model_instance_index, vdot));
                },
                {generalized_accelerations_cache_entry.ticket()})
            .get_index();
  }

  // Declare per model instance output port of generalized contact forces.
  instance_generalized_contact_forces_output_ports_.resize(
      num_model_instances());
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    const int instance_num_velocities = num_velocities(model_instance_index);

    if (is_discrete()) {
      const auto& tamsi_solver_results_cache_entry =
          this->get_cache_entry(cache_indexes_.tamsi_solver_results);
      auto calc = [this, model_instance_index](
                      const systems::Context<T>& context,
                      systems::BasicVector<T>* result) {
        const internal::TamsiSolverResults<T>& solver_results =
            EvalTamsiResults(context);
        this->CopyGeneralizedContactForcesOut(solver_results,
                                              model_instance_index, result);
      };
      instance_generalized_contact_forces_output_ports_[model_instance_index] =
          this->DeclareVectorOutputPort(
                  GetModelInstanceName(model_instance_index) +
                      "_generalized_contact_forces",
                  BasicVector<T>(instance_num_velocities), calc,
                  {tamsi_solver_results_cache_entry.ticket()})
              .get_index();
    } else {
      const auto& generalized_contact_forces_continuous_cache_entry =
          this->get_cache_entry(
              cache_indexes_.generalized_contact_forces_continuous);
      auto calc = [this, model_instance_index](
                      const systems::Context<T>& context,
                      systems::BasicVector<T>* result) {
        result->SetFromVector(GetVelocitiesFromArray(
            model_instance_index,
            EvalGeneralizedContactForcesContinuous(context)));
      };
      instance_generalized_contact_forces_output_ports_[model_instance_index] =
          this->DeclareVectorOutputPort(
                  GetModelInstanceName(model_instance_index) +
                      "_generalized_contact_forces",
                  BasicVector<T>(instance_num_velocities), calc,
                  {generalized_contact_forces_continuous_cache_entry.ticket()})
              .get_index();
    }
  }

  // Joint reaction forces are a function of accelerations, which in turn depend
  // on both state and inputs.
  reaction_forces_port_ =
      this->DeclareAbstractOutputPort(
              "reaction_forces", std::vector<SpatialForce<T>>(num_joints()),
              &MultibodyPlant<T>::CalcReactionForces,
              {this->cache_entry_ticket(
                  cache_indexes_.generalized_accelerations)})
          .get_index();

  // Contact results output port.
  const auto& contact_results_cache_entry =
      this->get_cache_entry(cache_indexes_.contact_results);
  contact_results_port_ = this->DeclareAbstractOutputPort(
                                  "contact_results", ContactResults<T>(),
                                  &MultibodyPlant<T>::CopyContactResultsOutput,
                                  {contact_results_cache_entry.ticket()})
                              .get_index();
}

template <typename T>
void MultibodyPlant<T>::DeclareCacheEntries() {
  DRAKE_DEMAND(this->is_finalized());

  // Cache entry for point contact queries.
  auto& point_pairs_cache_entry = this->DeclareCacheEntry(
      std::string("Point pair penetrations."),
      []() {
        return AbstractValue::Make(
            std::vector<geometry::PenetrationAsPointPair<T>>());
      },
      [this](const systems::ContextBase& context_base,
             AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& point_pairs_cache = cache_value->get_mutable_value<
            std::vector<geometry::PenetrationAsPointPair<T>>>();
        point_pairs_cache = this->CalcPointPairPenetrations(context);
      },
      {this->configuration_ticket()});
  cache_indexes_.point_pairs = point_pairs_cache_entry.cache_index();

  // Cache entry for hydroelastic contact surfaces.
  auto& contact_surfaces_cache_entry = this->DeclareCacheEntry(
      std::string("Hydroelastic contact surfaces."),
      []() {
        return AbstractValue::Make(
            std::vector<ContactSurface<T>>());
      },
      [this](const systems::ContextBase& context_base,
             AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& contact_surfaces_cache = cache_value->get_mutable_value<
            std::vector<ContactSurface<T>>>();
        this->CalcContactSurfaces(context, &contact_surfaces_cache);
      },
      {this->configuration_ticket()});
  cache_indexes_.contact_surfaces = contact_surfaces_cache_entry.cache_index();

  // Cache contact Jacobians.
  auto& contact_jacobians_cache_entry = this->DeclareCacheEntry(
      std::string("Contact Jacobians Jn(q) and Jt(q)."),
      []() { return AbstractValue::Make(internal::ContactJacobians<T>()); },
      [this](const systems::ContextBase& context_base,
             AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& contact_jacobians_cache =
            cache_value->get_mutable_value<internal::ContactJacobians<T>>();
        this->CalcNormalAndTangentContactJacobians(
            context, EvalPointPairPenetrations(context),
            &contact_jacobians_cache.Jn, &contact_jacobians_cache.Jt,
            &contact_jacobians_cache.R_WC_list);
      },
      // We explicitly declare the configuration dependence even though the
      // Eval() above implicitly evaluates configuration dependent cache
      // entries.
      {this->configuration_ticket()});
  cache_indexes_.contact_jacobians =
      contact_jacobians_cache_entry.cache_index();

  // Cache TamsiSolver computations.
  auto& tamsi_solver_cache_entry = this->DeclareCacheEntry(
      std::string("Implicit Stribeck solver computations."),
      []() {
        return AbstractValue::Make(
            internal::TamsiSolverResults<T>());
      },
      [this](const systems::ContextBase& context_base,
             AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& tamsi_solver_cache = cache_value->get_mutable_value<
            internal::TamsiSolverResults<T>>();
        this->CalcTamsiResults(context,
                                          &tamsi_solver_cache);
      },
      // The Correct Solution:
      // The Implicit Stribeck solver solution S is a function of state x,
      // actuation input u (and externally applied forces) and even time if any
      // of the force elements in the model is time dependent. We can write this
      // as S = S(t, x, u).
      // Even though this variables can change continuously with time, we want
      // the solver solution to be updated periodically (with period
      // time_step()) only. That is, TamsiSolverResults should be
      // handled as an abstract state with periodic updates. In the systems::
      // framework terminology, we'd like to have an "unrestricted update" with
      // a periodic event trigger.
      // The Problem (#10149):
      // From issue #10149 we know unrestricted updates incur a very noticeably
      // performance hit that at this stage we are not willing to pay.
      // The Work Around (#10888):
      // To emulate the correct behavior until #10149 is addressed we declare
      // the Implicit Stribeck solver solution dependent only on the discrete
      // state. This is not the correct solution given these results do depend
      // on time and (even continuous) inputs. However it does emulate the
      // discrete update of these values as if zero-order held, which is what we
      // want.
      {this->xd_ticket()});
  cache_indexes_.tamsi_solver_results =
      tamsi_solver_cache_entry.cache_index();

  // Cache entry for spatial forces and contact info due to hydroelastic
  // contact.
  if (contact_model_ == ContactModel::kHydroelasticsOnly) {
    auto& contact_info_and_body_spatial_forces_cache_entry =
        this->DeclareCacheEntry(
            std::string("Hydroelastic contact info and body spatial forces."),
            [this]() {
              return AbstractValue::Make(
                  HydroelasticContactInfoAndBodySpatialForces(
                      this->num_bodies()));
            },
            [this](const systems::ContextBase& context_base,
                   AbstractValue* cache_value) {
              auto& context = dynamic_cast<const Context<T>&>(context_base);
              auto& contact_info_and_body_spatial_forces_cache =
                  cache_value->get_mutable_value<
                      HydroelasticContactInfoAndBodySpatialForces>();
              this->CalcHydroelasticContactForces(
                  context, &contact_info_and_body_spatial_forces_cache);
            },
            // Compliant contact forces due to hydroelastics with Hunt &
            // Crosseley are function of the kinematic variables q & v only.
            {this->kinematics_ticket()});
    cache_indexes_.contact_info_and_body_spatial_forces =
        contact_info_and_body_spatial_forces_cache_entry.cache_index();
  }

  // Cache contact results.
  // In discrete mode contact forces computation requires to advance the system
  // from step n to n+1. Therefore they are a function of state and input.
  // In continuous mode contact forces are simply a function of state.
  std::set<systems::DependencyTicket> dependency_ticket = [this]() {
    std::set<systems::DependencyTicket> tickets;
     if (is_discrete()) {
       tickets.insert(
           this->cache_entry_ticket(cache_indexes_.tamsi_solver_results));
     } else {
       tickets.insert(this->kinematics_ticket());
       if (contact_model_ == ContactModel::kHydroelasticsOnly) {
         tickets.insert(this->cache_entry_ticket(
             cache_indexes_.contact_info_and_body_spatial_forces));
       }
     }

     return tickets;
  }();
  auto& contact_results_cache_entry = this->DeclareCacheEntry(
      std::string("Contact results."),
      []() { return AbstractValue::Make(ContactResults<T>()); },
      [this](const systems::ContextBase& context_base,
             AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& contact_results_cache =
            cache_value->get_mutable_value<ContactResults<T>>();
        if (is_discrete()) {
          this->CalcContactResultsDiscrete(context, &contact_results_cache);
        } else {
          this->CalcContactResultsContinuous(context, &contact_results_cache);
        }
      },
      {dependency_ticket});
  cache_indexes_.contact_results = contact_results_cache_entry.cache_index();

  // Articulated Body Algorithm (ABA) force bias cache.
  auto& aba_force_bias_cache_entry = this->DeclareCacheEntry(
      std::string("ABA force bias cache."),
      ArticulatedBodyForceBiasCache<T>(internal_tree().get_topology()),
      &MultibodyPlant<T>::CalcArticulatedBodyForceBiasCache,
      // ABA computes quantities such as Zplus which are needed for the
      // computation of acceleration and thus depend on both state and inputs.
      // All sources include: time, accuracy, state, input ports, and
      // parameters.
      {this->all_sources_ticket()});
  cache_indexes_.aba_force_bias_cache =
      aba_force_bias_cache_entry.cache_index();

  // Last pass of the ABA for forward dynamics.
  auto& aba_accelerations_cache_entry = this->DeclareCacheEntry(
      std::string("ABA accelerations."),
      AccelerationKinematicsCache<T>(internal_tree().get_topology()),
      &MultibodyPlant<T>::CalcForwardDynamics,
      // Accelerations depend on both state and inputs.
      // All sources include: time, accuracy, state, input ports, and
      // parameters.
      {this->all_sources_ticket()});
  cache_indexes_.aba_accelerations =
      aba_accelerations_cache_entry.cache_index();

  // Cache generalized accelerations.
  auto& vdot_cache_entry = this->DeclareCacheEntry(
      std::string("Generalized Accelerations (vdot)."),
      [this]() { return AbstractValue::Make(VectorX<T>(num_velocities())); },
      [this](const systems::ContextBase& context_base,
             AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& vdot_in_cache = cache_value->get_mutable_value<VectorX<T>>();
        this->CalcGeneralizedAccelerations(context, &vdot_in_cache);
      },
      // Generalized accelerations depend on both state and inputs.
      // All sources include: time, accuracy, state, input ports, and
      // parameters.
      {this->all_sources_ticket()});
  cache_indexes_.generalized_accelerations = vdot_cache_entry.cache_index();

  // Cache spatial continuous contact forces.
  auto& spatial_contact_forces_continuous_cache_entry = this->DeclareCacheEntry(
      "Spatial contact forces (continuous).",
      std::vector<SpatialForce<T>>(num_bodies()),
      &MultibodyPlant::CalcSpatialContactForcesContinuous,
      {this->kinematics_ticket()});
  cache_indexes_.spatial_contact_forces_continuous =
      spatial_contact_forces_continuous_cache_entry.cache_index();

  // Cache generalized continuous contact forces.
  auto& generalized_contact_forces_continuous_cache_entry =
      this->DeclareCacheEntry(
          "Generalized contact forces (continuous).",
          VectorX<T>(num_velocities()),
          &MultibodyPlant::CalcGeneralizedContactForcesContinuous,
          {this->cache_entry_ticket(
              cache_indexes_.spatial_contact_forces_continuous)});
  cache_indexes_.generalized_contact_forces_continuous =
      generalized_contact_forces_continuous_cache_entry.cache_index();
      }

      template <typename T>
      const systems::BasicVector<T> &
      MultibodyPlant<T>::GetStateVector(const Context<T>& context) const {
    if (is_discrete()) {
      return context.get_discrete_state(0);
    } else {
      return dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector());
    }
}

template <typename T>
void MultibodyPlant<T>::CopyContinuousStateOut(
    const Context<T>& context, BasicVector<T>* state_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  state_vector->SetFrom(GetStateVector(context));
}

template <typename T>
void MultibodyPlant<T>::CopyContinuousStateOut(
    ModelInstanceIndex model_instance,
    const Context<T>& context, BasicVector<T>* state_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();

  VectorX<T> instance_state_vector =
      GetPositionsAndVelocities(context, model_instance);
  state_vector->SetFromVector(instance_state_vector);
}

template <typename T>
void MultibodyPlant<T>::CopyGeneralizedContactForcesOut(
    const internal::TamsiSolverResults<T>& solver_results,
    ModelInstanceIndex model_instance, BasicVector<T>* tau_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(is_discrete());

  // Vector of generalized contact forces for the entire plant's multibody
  // system.
  const VectorX<T>& tau_contact = solver_results.tau_contact;

  // Generalized velocities and generalized forces are ordered in the same way.
  // Thus we can call get_velocities_from_array().
  const VectorX<T> instance_tau_contact =
      GetVelocitiesFromArray(model_instance, tau_contact);

  tau_vector->set_value(instance_tau_contact);
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_actuation_input_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(num_actuators() > 0);
  DRAKE_THROW_UNLESS(actuated_instance_.is_valid());
  return get_actuation_input_port(actuated_instance_);
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_applied_generalized_force_input_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_input_port(applied_generalized_force_input_port_);
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_actuation_input_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return systems::System<T>::get_input_port(
      instance_actuation_ports_.at(model_instance));
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_applied_spatial_force_input_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return systems::System<T>::get_input_port(applied_spatial_force_input_port_);
}

template <typename T>
const systems::OutputPort<T>& MultibodyPlant<T>::get_state_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(state_output_port_);
}

template <typename T>
const systems::OutputPort<T>& MultibodyPlant<T>::get_state_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_output_port(
      instance_state_output_ports_.at(model_instance));
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_generalized_acceleration_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(generalized_acceleration_output_port_);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_generalized_acceleration_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_output_port(
      instance_generalized_acceleration_output_ports_.at(model_instance));
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_generalized_contact_forces_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  return this->get_output_port(
      instance_generalized_contact_forces_output_ports_.at(model_instance));
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_contact_results_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(contact_results_port_);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_reaction_forces_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(reaction_forces_port_);
}

namespace {
// A dummy, placeholder type.
struct SymbolicGeometryValue {};
// An alias for QueryObject<T>, except when T = Expression.
template <typename T>
using ModelQueryObject = typename std::conditional<
    std::is_same<T, symbolic::Expression>::value,
    SymbolicGeometryValue, geometry::QueryObject<T>>::type;
}  // namespace

template <typename T>
void MultibodyPlant<T>::DeclareSceneGraphPorts() {
  geometry_query_port_ = this->DeclareAbstractInputPort(
      "geometry_query", Value<ModelQueryObject<T>>{}).get_index();
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
      "geometry_pose", &MultibodyPlant<T>::CalcFramePoseOutput,
      {this->configuration_ticket()}).get_index();
}

template <typename T>
void MultibodyPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_ASSERT(source_id_ != std::nullopt);
  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);

  // NOTE: The body index to frame id map *always* includes the world body but
  // the world body does *not* get reported in the frame poses; only dynamic
  // frames do.
  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  poses->clear();
  for (const auto it : body_index_to_frame_id_) {
    const BodyIndex body_index = it.first;
    if (body_index == world_index()) continue;
    const Body<T>& body = get_body(body_index);

    // NOTE: The GeometryFrames for each body were registered in the world
    // frame, so we report poses in the world frame.
    poses->set_value(body_index_to_frame_id_.at(body_index),
                     pc.get_X_WB(body.node_index()));
  }
}

template <typename T>
void MultibodyPlant<T>::CalcReactionForces(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* F_CJc_Jc_array) const {
  DRAKE_DEMAND(F_CJc_Jc_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(F_CJc_Jc_array->size()) == num_joints());

  const VectorX<T>& vdot = EvalGeneralizedAccelerations(context);

  MultibodyForces<T> applied_forces(*this);
  CalcAppliedForces(context, &applied_forces);
  auto& Fapplied_Bo_W_array = applied_forces.mutable_body_forces();
  auto& tau_applied = applied_forces.mutable_generalized_forces();

  CalcAndAddContactForcesByPenaltyMethod(context, &Fapplied_Bo_W_array);

  // Compute reaction forces at each mobilizer.
  std::vector<SpatialAcceleration<T>> A_WB_vector(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_vector(num_bodies());
  VectorX<T> tau_id(num_velocities());
  internal_tree().CalcInverseDynamics(context, vdot, Fapplied_Bo_W_array,
                                      tau_applied, &A_WB_vector,
                                      &F_BMo_W_vector, &tau_id);
  // Since vdot is the result of Fapplied and tau_applied we expect the result
  // from inverse dynamics to be zero.
  // TODO(amcastro-tri): find a better estimation for this bound. For instance,
  // we can make an estimation based on the trace of the mass matrix (Jain 2011,
  // Eq. 4.21). For now we only ASSERT though with a better estimation we could
  // promote this to a DEMAND.
  DRAKE_ASSERT(tau_id.norm() <
               100 * num_velocities() * std::numeric_limits<double>::epsilon());

  // Map mobilizer reaction forces to joint reaction forces and perform the
  // necessary frame conversions.
  for (JointIndex joint_index(0); joint_index < num_joints(); ++joint_index) {
    const Joint<T>& joint = get_joint(joint_index);
    const internal::MobilizerIndex mobilizer_index =
        internal_tree().get_joint_mobilizer(joint_index);
    const internal::Mobilizer<T>& mobilizer =
        internal_tree().get_mobilizer(mobilizer_index);
    const internal::BodyNodeIndex body_node_index =
        mobilizer.get_topology().body_node;

    // Force on mobilized body B at mobilized frame's origin Mo, expressed in
    // world frame.
    const SpatialForce<T>& F_BMo_W = F_BMo_W_vector[body_node_index];

    // Frames:
    const Frame<T>& frame_Jp = joint.frame_on_parent();
    const Frame<T>& frame_Jc = joint.frame_on_child();
    const FrameIndex F_index = mobilizer.inboard_frame().index();
    const FrameIndex M_index = mobilizer.outboard_frame().index();
    const FrameIndex Jp_index = frame_Jp.index();
    const FrameIndex Jc_index = frame_Jc.index();

    // In Drake we have either:
    //  - Jp == F and Jc == M (typical case)
    //  - Jp == M and Jc == F (mobilizer was inverted)
    // We verify this:
    DRAKE_DEMAND((Jp_index == F_index && Jc_index == M_index) ||
                 (Jp_index == M_index && Jc_index == F_index));

    SpatialForce<T> F_CJc_W;
    if (Jc_index == M_index) {
      // Given we now Mo == Jc and B == C.
      F_CJc_W = F_BMo_W;
    } else if (joint.frame_on_child().index() ==
               mobilizer.inboard_frame().index()) {
      // Given we now Mo == Jc and B == C.
      const SpatialForce<T>& F_PJp_W = F_BMo_W;

      // Newton's third law allows to find the reaction on the child body as
      // required.
      const SpatialForce<T> F_CJp_W = -F_PJp_W;

      // Now we need to shift the application point from Jp to Jc.
      // First we need to find the position vector p_JpJc_W.
      const RigidTransform<T> X_WJp = frame_Jp.CalcPoseInWorld(context);
      const RotationMatrix<T>& R_WJp = X_WJp.rotation();
      const RigidTransform<T> X_JpJc = frame_Jc.CalcPose(context, frame_Jp);
      const Vector3<T> p_JpJc_Jp = X_JpJc.translation();
      const Vector3<T> p_JpJc_W = R_WJp * p_JpJc_Jp;

      // Finally, we shift the spatial force at Jp.
      F_CJc_W = F_CJp_W.Shift(p_JpJc_W);
    }

    // Re-express in the joint's child frame Jc.
    const RigidTransform<T> X_WJc = frame_Jc.CalcPoseInWorld(context);
    const RotationMatrix<T> R_JcW = X_WJc.rotation().transpose();
    F_CJc_Jc_array->at(joint_index) = R_JcW * F_CJc_W;
  }
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_poses_output_port()
const {
  DRAKE_DEMAND(geometry_source_is_registered());
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_geometry_query_input_port() const {
  DRAKE_DEMAND(geometry_source_is_registered());
  return systems::System<T>::get_input_port(geometry_query_port_);
}

template <typename T>
void MultibodyPlant<T>::ThrowIfFinalized(const char* source_method) const {
  if (is_finalized()) {
    throw std::logic_error(
        "Post-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; calls to this method must happen before Finalize().");
  }
}

template <typename T>
void MultibodyPlant<T>::ThrowIfNotFinalized(const char* source_method) const {
  if (!is_finalized()) {
    throw std::logic_error(
        "Pre-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; you must call Finalize() first.");
  }
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::ComputeFrictionCoefficient(
    const T& speed_BcAc,
    const CoulombFriction<double>& friction) const {
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
AddMultibodyPlantSceneGraphResult<T>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,
    std::unique_ptr<MultibodyPlant<T>> plant,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph) {
  DRAKE_DEMAND(builder != nullptr);
  if (!plant) {
    plant = std::make_unique<MultibodyPlant<T>>();
    plant->set_name("plant");
  }
  if (!scene_graph) {
    scene_graph = std::make_unique<geometry::SceneGraph<T>>();
    scene_graph->set_name("scene_graph");
  }
  auto* plant_ptr = builder->AddSystem(std::move(plant));
  auto* scene_graph_ptr = builder->AddSystem(std::move(scene_graph));
  plant_ptr->RegisterAsSourceForSceneGraph(scene_graph_ptr);
  builder->Connect(
      plant_ptr->get_geometry_poses_output_port(),
      scene_graph_ptr->get_source_pose_port(
          plant_ptr->get_source_id().value()));
  builder->Connect(
      scene_graph_ptr->get_query_output_port(),
      plant_ptr->get_geometry_query_input_port());
  return {plant_ptr, scene_graph_ptr};
}

// Add explicit instantiations for `AddMultibodyPlantSceneGraph`.
// This does *not* support symbolic::Expression.
template
AddMultibodyPlantSceneGraphResult<double>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<double>* builder,
    std::unique_ptr<MultibodyPlant<double>> plant,
    std::unique_ptr<geometry::SceneGraph<double>> scene_graph);

template
AddMultibodyPlantSceneGraphResult<AutoDiffXd>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<AutoDiffXd>* builder,
    std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant,
    std::unique_ptr<geometry::SceneGraph<AutoDiffXd>> scene_graph);

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::MultibodyPlant)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct drake::multibody::AddMultibodyPlantSceneGraphResult)
