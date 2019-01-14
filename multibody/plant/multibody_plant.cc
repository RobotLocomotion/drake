#include "drake/multibody/plant/multibody_plant.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/math/rotation_matrix.h"
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

using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::PenetrationAsPointPair;
using geometry::SceneGraph;
using geometry::SourceId;
using systems::InputPort;
using systems::OutputPort;
using systems::State;

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyTree;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::VelocityKinematicsCache;
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
          const Isometry3<T> X_PJ = frame.GetFixedPoseInBodyFrame();
          const Vector3<T>& p_PJ = X_PJ.translation();
          const math::RotationMatrix<T> R_PJ(X_PJ.linear());
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
          systems::SystemTypeTag<multibody::MultibodyPlant>{},
          std::move(tree_in), time_step > 0),
      time_step_(time_step) {
  DRAKE_THROW_UNLESS(time_step >= 0);
  visual_geometries_.emplace_back();  // Entries for the "world" body.
  collision_geometries_.emplace_back();
}

template<typename T>
const WeldJoint<T>& MultibodyPlant<T>::WeldFrames(
    const Frame<T>& A, const Frame<T>& B, const Isometry3<double>& X_AB) {
  const std::string joint_name = A.name() + "_welds_to_" + B.name();
  return this->mutable_tree().AddJoint(
      std::make_unique<WeldJoint<T>>(joint_name, A, B, X_AB));
}

template <typename T>
geometry::SourceId MultibodyPlant<T>::RegisterAsSourceForSceneGraph(
    SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(!geometry_source_is_registered());
  source_id_ = scene_graph->RegisterSource();
  // Save the GS pointer so that on later geometry registrations can use this
  // instance. This will be nullified at Finalize().
  scene_graph_ = scene_graph;
  body_index_to_frame_id_[world_index()] = scene_graph->world_frame_id();
  DeclareSceneGraphPorts();
  return source_id_.value();
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body, const Isometry3<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    geometry::SceneGraph<T>* scene_graph) {
  return RegisterVisualGeometry(
      body, X_BG, shape, name, geometry::IllustrationProperties(), scene_graph);
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body, const Isometry3<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const Vector4<double>& diffuse_color,
    SceneGraph<T>* scene_graph) {
  return RegisterVisualGeometry(
      body, X_BG, shape, name,
      geometry::MakeDrakeVisualizerProperties(diffuse_color), scene_graph);
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body, const Isometry3<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const geometry::IllustrationProperties& properties,
    SceneGraph<T>* scene_graph) {
  // TODO(SeanCurtis-TRI): Consider simply adding an interface that takes a
  // unique pointer to an already instantiated GeometryInstance. This will
  // require shuffling around a fair amount of code and should ultimately be
  // supplanted by providing a cleaner interface between parsing MBP and SG
  // elements.
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  CheckUserProvidedSceneGraph(scene_graph);

  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register geometry that has a fixed path to world to the world body (i.e.,
  // as anchored geometry).
  GeometryId id = RegisterGeometry(
      body, X_BG, shape, GetScopedName(*this, body.model_instance(), name),
      scene_graph_);
  scene_graph_->AssignRole(*source_id_, id, properties);
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
    const Body<T>& body, const Isometry3<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const CoulombFriction<double>& coulomb_friction,
    SceneGraph<T>* scene_graph) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  CheckUserProvidedSceneGraph(scene_graph);

  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register geometry that has a fixed path to world to the world body (i.e.,
  // as anchored geometry).
  GeometryId id = RegisterGeometry(
      body, X_BG, shape, GetScopedName(*this, body.model_instance(), name),
      scene_graph_);

  // TODO(SeanCurtis-TRI): Push the contact parameters into the
  // ProximityProperties.
  scene_graph_->AssignRole(*source_id_, id, geometry::ProximityProperties());
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
    optional<FrameId> frame_id = GetBodyFrameIdIfExists(body->index());
    if (frame_id) {
      geometry_set.Add(frame_id.value());
    }
  }
  return geometry_set;
}

template <typename T>
std::vector<const Body<T>*> MultibodyPlant<T>::GetBodiesWeldedTo(
    const Body<T>& body) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  // TODO(eric.cousineau): This is much slower than it should be; this could be
  // sped up by either (a) caching these results at finalization and store a
  // mapping from body to a subgraph or (b) starting the search from the query
  // body.
  auto sub_graphs = internal_tree().get_topology().CreateListOfWeldedBodies();
  // Find subgraph that contains this body.
  auto predicate = [&body](auto& sub_graph) {
    return sub_graph.count(body.index()) > 0;
  };
  auto sub_graph_iter = std::find_if(
      sub_graphs.begin(), sub_graphs.end(), predicate);
  DRAKE_THROW_UNLESS(sub_graph_iter != sub_graphs.end());
  // Map body indices to pointers.
  std::vector<const Body<T>*> sub_graph_bodies;
  for (BodyIndex sub_graph_body_index : *sub_graph_iter) {
    sub_graph_bodies.push_back(&internal_tree().get_body(sub_graph_body_index));
  }
  return sub_graph_bodies;
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterGeometry(
    const Body<T>& body, const Isometry3<double>& X_BG,
    const geometry::Shape& shape,
    const std::string& name,
    SceneGraph<T>* scene_graph) {
  DRAKE_ASSERT(!is_finalized());
  DRAKE_ASSERT(geometry_source_is_registered());
  CheckUserProvidedSceneGraph(scene_graph);
  // If not already done, register a frame for this body.
  if (!body_has_registered_frame(body)) {
    FrameId frame_id = scene_graph_->RegisterFrame(
        source_id_.value(),
        GeometryFrame(
            GetScopedName(*this, body.model_instance(), body.name()),
            /* Initial pose: Not really used by GS. Will get removed. */
            Isometry3<double>::Identity(),
            /* TODO(@SeanCurtis-TRI): Add test coverage for this
             * model-instance support as requested in #9390. */
            body.model_instance()));
    body_index_to_frame_id_[body.index()] = frame_id;
    frame_id_to_body_index_[frame_id] = body.index();
  }

  // Register geometry in the body frame.
  std::unique_ptr<geometry::GeometryInstance> geometry_instance =
      std::make_unique<GeometryInstance>(X_BG, shape.Clone(), name);
  GeometryId geometry_id = scene_graph->RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.index()],
      std::move(geometry_instance));
  geometry_id_to_body_index_[geometry_id] = body.index();
  return geometry_id;
}

template<typename T>
void MultibodyPlant<T>::SetFreeBodyPoseInWorldFrame(
    systems::Context<T>* context,
    const Body<T>& body, const Isometry3<T>& X_WB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context);
}

template<typename T>
void MultibodyPlant<T>::SetFreeBodyPoseInAnchoredFrame(
    systems::Context<T>* context,
    const Frame<T>& frame_F, const Body<T>& body,
    const Isometry3<T>& X_FB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();

  if (!internal_tree().get_topology().IsBodyAnchored(frame_F.body().index())) {
    throw std::logic_error(
        "Frame '" + frame_F.name() + "' must be anchored to the world frame.");
  }

  // Pose of frame F in its parent body frame P.
  const Isometry3<T> X_PF = frame_F.GetFixedPoseInBodyFrame();
  // Pose of frame F's parent body P in the world.
  const Isometry3<T>& X_WP = EvalBodyPoseInWorld(*context, frame_F.body());
  // Pose of "body" B in the world frame.
  const Isometry3<T> X_WB = X_WP * X_PF * X_FB;
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
void MultibodyPlant<T>::Finalize(geometry::SceneGraph<T>* scene_graph) {
  // After finalizing the base class, tree is read-only.
  MultibodyTreeSystem<T>::Finalize();
  CheckUserProvidedSceneGraph(scene_graph);
  if (geometry_source_is_registered()) {
    FilterAdjacentBodies();
    ExcludeCollisionsWithVisualGeometry();
  }
  FinalizePlantOnly();
}

template<typename T>
void MultibodyPlant<T>::SetUpJointLimitsParameters() {
  for (JointIndex joint_index(0); joint_index < internal_tree().num_joints();
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

    const Joint<T>& joint = internal_tree().get_joint(joint_index);
    auto revolute_joint = dynamic_cast<const RevoluteJoint<T>*>(&joint);
    auto prismatic_joint = dynamic_cast<const PrismaticJoint<T>*>(&joint);
    // Currently MBP only supports limits for prismatic and revolute joints.
    if (!(revolute_joint || prismatic_joint)) continue;

    const double penalty_time_scale = kAlpha * time_step();

    if (revolute_joint) {
      // We only compute parameters if joints do have upper/lower bounds.
      if (!std::isinf(revolute_joint->lower_limit()) ||
          !std::isinf(revolute_joint->upper_limit())) {
        joint_limits_parameters_.joints_with_limits.push_back(
            revolute_joint->index());

        // Store joint limits.
        joint_limits_parameters_.lower_limit.push_back(
            revolute_joint->lower_limit());
        joint_limits_parameters_.upper_limit.push_back(
            revolute_joint->upper_limit());
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
      // We only compute parameters if joints do have upper/lower bounds.
      if (!std::isinf(prismatic_joint->lower_limit()) ||
          !std::isinf(prismatic_joint->upper_limit())) {
        joint_limits_parameters_.joints_with_limits.push_back(
            prismatic_joint->index());

        // Store joint limits.
        joint_limits_parameters_.lower_limit.push_back(
            prismatic_joint->lower_limit());
        joint_limits_parameters_.upper_limit.push_back(
            prismatic_joint->upper_limit());

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
              "`" + internal_tree().get_joint(index).name() + "`.");
        }
      }
    }
  }
}

template<typename T>
void MultibodyPlant<T>::FinalizePlantOnly() {
  DeclareStateCacheAndPorts();
  scene_graph_ = nullptr;  // must not be used after Finalize().
  if (num_collision_geometries() > 0 &&
      penalty_method_contact_parameters_.time_scale < 0)
    set_penetration_allowance();
  if (num_collision_geometries() > 0 &&
      stribeck_model_.stiction_tolerance() < 0)
    set_stiction_tolerance();
  // Make a contact solver when the plant is modeled as a discrete system.
  if (is_discrete()) {
    implicit_stribeck_solver_ =
        std::make_unique<ImplicitStribeckSolver<T>>(num_velocities());
    // Set the stiction tolerance according to the values set by users with
    // set_stiction_tolerance().
    ImplicitStribeckSolverParameters solver_parameters;
    solver_parameters.stiction_tolerance =
        stribeck_model_.stiction_tolerance();
    implicit_stribeck_solver_->set_solver_parameters(solver_parameters);
  }
  SetUpJointLimitsParameters();
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
void MultibodyPlant<T>::CheckUserProvidedSceneGraph(
    const geometry::SceneGraph<T>* scene_graph) const {
  if (scene_graph != nullptr) {
    if (!geometry_source_is_registered()) {
      throw std::logic_error(
          "This MultibodyPlant instance does not have a SceneGraph registered. "
          "Geometry registration calls must be performed after "
          "RegisterAsSourceForSceneGraph() (which is implicitly called via "
          "parsing methods when passed a SceneGraph instance).");
    }
    if (scene_graph != scene_graph_) {
      throw std::logic_error(
          "Geometry registration calls must be performed on the SAME instance "
          "of SceneGraph used on the first call to "
          "RegisterAsSourceForSceneGraph() (which is implicitly called via "
          "parsing methods when passed a SceneGraph instance).");
    }
  }
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
  for (JointIndex j{0}; j < internal_tree().num_joints(); ++j) {
    const Joint<T>& joint = internal_tree().get_joint(j);
    const Body<T>& child = joint.child_body();
    const Body<T>& parent = joint.parent_body();
    // TODO(SeanCurtis-TRI): Determine the correct action for a body
    // joined to the world -- should it filter out collisions between the
    // body and all *anchored* geometry? That seems really heavy-handed. So,
    // for now, we skip the joints to the world.
    if (parent.index() == world_index()) continue;
    optional<FrameId> child_id = GetBodyFrameIdIfExists(child.index());
    optional<FrameId> parent_id = GetBodyFrameIdIfExists(parent.index());

    if (child_id && parent_id) {
      scene_graph_->ExcludeCollisionsBetween(
          geometry::GeometrySet(*child_id),
          geometry::GeometrySet(*parent_id));
    }
  }
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
  scene_graph_->ExcludeCollisionsWithin(visual);
  scene_graph_->ExcludeCollisionsBetween(visual, collision);
}

template<typename T>
void MultibodyPlant<T>::CalcNormalAndTangentContactJacobians(
    const systems::Context<T>& context,
    const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs_set,
    MatrixX<T>* Jn_ptr, MatrixX<T>* Jt_ptr,
    std::vector<Matrix3<T>>* R_WC_set) const {
  DRAKE_DEMAND(Jn_ptr != nullptr);
  DRAKE_DEMAND(Jt_ptr != nullptr);

  const int num_contacts = point_pairs_set.size();

  // Jn is defined such that vn = Jn * v, with vn of size nc.
  auto& Jn = *Jn_ptr;
  Jn.resize(num_contacts, num_velocities());

  // Jt is defined such that vt = Jt * v, with vt of size 2nc.
  auto& Jt = *Jt_ptr;
  Jt.resize(2 * num_contacts, num_velocities());

  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = point_pairs_set[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const Body<T>& bodyA = internal_tree().get_body(bodyA_index);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);
    const Body<T>& bodyB = internal_tree().get_body(bodyB_index);

    // Penetration depth, > 0 if bodies interpenetrate.
    const Vector3<T>& nhat_BA_W = point_pair.nhat_BA_W;
    const Vector3<T>& p_WCa = point_pair.p_WCa;
    const Vector3<T>& p_WCb = point_pair.p_WCb;

    // TODO(amcastro-tri): Consider using the midpoint between Ac and Bc for
    // stability reasons. Besides that, there is no other reason to use the
    // midpoint (or any other point between Ac and Bc for that matter) since,
    // in the limit to rigid contact, Ac = Bc.

    // Geometric Jacobian for the velocity of the contact point C as moving with
    // body A, s.t.: v_WAc = Jv_WAc * v
    // where v is the vector of generalized velocities.
    MatrixX<T> Jv_WAc(3, this->num_velocities());
    internal_tree().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyA.body_frame(), p_WCa, &Jv_WAc);

    // Geometric Jacobian for the velocity of the contact point C as moving with
    // body B, s.t.: v_WBc = Jv_WBc * v.
    MatrixX<T> Jv_WBc(3, this->num_velocities());
    internal_tree().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyB.body_frame(), p_WCb, &Jv_WBc);

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
    const Matrix3<T> R_WC = math::ComputeBasisFromAxis(2, nhat_BA_W);
    if (R_WC_set != nullptr) {
      R_WC_set->push_back(R_WC);
    }

    const Vector3<T> that1_W = R_WC.col(0);  // that1 = Cx.
    const Vector3<T> that2_W = R_WC.col(1);  // that2 = Cy.

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
  const double g = gravity_field_.has_value() ?
                   gravity_field_.value()->gravity_vector().norm() :
                   UniformGravityFieldElement<double>::kDefaultStrength;

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
    const Body<T>& body = internal_tree().get_body(body_index);
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

template <>
std::vector<PenetrationAsPointPair<double>>
MultibodyPlant<double>::CalcPointPairPenetrations(
    const systems::Context<double>& context) const {
  if (num_collision_geometries() > 0) {
    if (!geometry_query_port_.is_valid()) {
      throw std::logic_error(
          "This MultibodyPlant registered geometry for contact handling. "
          "However its query input port (get_geometry_query_input_port()) "
          "is not connected. ");
    }
    const geometry::QueryObject<double>& query_object =
        this->EvalAbstractInput(context, geometry_query_port_)
            ->template GetValue<geometry::QueryObject<double>>();
    return query_object.ComputePointPairPenetration();
  }
  return std::vector<PenetrationAsPointPair<double>>();
}

template<typename T>
std::vector<PenetrationAsPointPair<T>>
MultibodyPlant<T>::CalcPointPairPenetrations(
    const systems::Context<T>&) const {
  DRAKE_ABORT_MSG("This method only supports T = double.");
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
void MultibodyPlant<T>::CalcContactResultsOutput(
    const systems::Context<T>&,
    ContactResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);
  // TODO(amcastro-tri): Eval() contact results when caching lands.
  *contact_results = contact_results_;
}

template<typename T>
void MultibodyPlant<T>::CalcContactResults(
    const systems::Context<T>&,
    const std::vector<PenetrationAsPointPair<T>>& point_pairs,
    const std::vector<Matrix3<T>>& R_WC_set,
    ContactResults<T>* contact_results) const {
  if (num_collision_geometries() == 0) return;
  DRAKE_DEMAND(contact_results != nullptr);
  const int num_contacts = point_pairs.size();
  DRAKE_DEMAND(static_cast<int>(R_WC_set.size()) == num_contacts);

  // Note: auto below resolves to VectorBlock<const VectorX<T>>.
  using VectorXBlock = Eigen::VectorBlock<const VectorX<T>>;
  const VectorXBlock fn = implicit_stribeck_solver_->get_normal_forces();
  const VectorXBlock ft = implicit_stribeck_solver_->get_friction_forces();
  const VectorXBlock vt =
      implicit_stribeck_solver_->get_tangential_velocities();
  const VectorXBlock vn = implicit_stribeck_solver_->get_normal_velocities();

  DRAKE_DEMAND(fn.size() == num_contacts);
  DRAKE_DEMAND(ft.size() == 2 * num_contacts);
  DRAKE_DEMAND(vn.size() == num_contacts);
  DRAKE_DEMAND(vt.size() == 2 * num_contacts);

  contact_results->Clear();
  for (size_t icontact = 0; icontact < point_pairs.size(); ++icontact) {
    const auto& pair = point_pairs[icontact];
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    const Vector3<T> p_WC = 0.5 * (pair.p_WCa + pair.p_WCb);

    const Matrix3<T>& R_WC = R_WC_set[icontact];

    // Contact forces applied on B at contact point C.
    const Vector3<T> f_Bc_C(
        -ft(2 * icontact), -ft(2 * icontact + 1), fn(icontact));
    const Vector3<T> f_Bc_W = R_WC * f_Bc_C;

    // Slip velocity.
    const T slip = vt.template segment<2>(2 * icontact).norm();

    // Separation velocity in the normal direction.
    const T separation_velocity = vn(icontact);

    // Add pair info to the contact results.
    contact_results->AddContactInfo(
        {bodyA_index, bodyB_index, f_Bc_W, p_WC,
         separation_velocity, slip, pair});
  }
}

template<typename T>
void MultibodyPlant<T>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& pc,
    const internal::VelocityKinematicsCache<T>& vc,
    const std::vector<PenetrationAsPointPair<T>>& point_pairs,
    std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  if (num_collision_geometries() == 0) return;

  std::vector<CoulombFriction<double>> combined_friction_pairs =
      CalcCombinedFrictionCoefficients(point_pairs);

  for (size_t icontact = 0; icontact < point_pairs.size(); ++icontact) {
    const auto& pair = point_pairs[icontact];
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    internal::BodyNodeIndex bodyA_node_index =
        internal_tree().get_body(bodyA_index).node_index();
    internal::BodyNodeIndex bodyB_node_index =
        internal_tree().get_body(bodyB_index).node_index();

    // Penetration depth, > 0 during pair.
    const T& x = pair.depth;
    DRAKE_ASSERT(x >= 0);
    const Vector3<T>& nhat_BA_W = pair.nhat_BA_W;
    const Vector3<T>& p_WCa = pair.p_WCa;
    const Vector3<T>& p_WCb = pair.p_WCb;

    // Contact point C.
    const Vector3<T> p_WC = 0.5 * (p_WCa + p_WCb);

    // Contact point position on body A.
    const Vector3<T>& p_WAo =
        pc.get_X_WB(bodyA_node_index).translation();
    const Vector3<T>& p_CoAo_W = p_WAo - p_WC;

    // Contact point position on body B.
    const Vector3<T>& p_WBo =
        pc.get_X_WB(bodyB_node_index).translation();
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
      if (vt_squared > kNonZeroSqd) {
        const T vt = sqrt(vt_squared);
        // Stribeck friction coefficient.
        const T mu_stribeck = stribeck_model_.ComputeFrictionCoefficient(
            vt, combined_friction_pairs[icontact]);
        // Tangential direction.
        const Vector3<T> that_W = vt_AcBc_W / vt;

        // Magnitude of the friction force on A at C.
        const T ft_AC = mu_stribeck * fn_AC;
        ft_AC_W = ft_AC * that_W;
      }

      // Spatial force on body A at C, expressed in the world frame W.
      const SpatialForce<T> F_AC_W(Vector3<T>::Zero(),
                                   fn_AC_W + ft_AC_W);

      if (F_BBo_W_array != nullptr) {
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
          internal_tree().get_joint_actuator(actuator_index);
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
    const Joint<T>& joint = internal_tree().get_joint(joint_index);

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
    const int instance_num_dofs =
        internal_tree().num_actuated_dofs(model_instance_index);
    if (instance_num_dofs == 0) {
      continue;
    }
    Eigen::VectorBlock<const VectorX<T>> u_instance =
        this->EvalEigenVectorInput(
            context, instance_actuation_ports_[model_instance_index]);
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

  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = this->num_velocities();

  // Allocate workspace. We might want to cache these to avoid allocations.
  // Mass matrix.
  MatrixX<T> M(nv, nv);
  // Forces.
  MultibodyForces<T> forces(internal_tree());
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(internal_tree().num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);
  const internal::VelocityKinematicsCache<T>& vc =
      EvalVelocityKinematics(context);

  // Compute forces applied through force elements. This effectively resets
  // the forces to zero and adds in contributions due to force elements.
  internal_tree().CalcForceElementsContribution(context, pc, vc, &forces);

  // If there is any input actuation, add it to the multibody forces.
  AddJointActuationForces(context, &forces);

  internal_tree().CalcMassMatrixViaInverseDynamics(context, &M);

  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both the array of applied body forces and
  // the array of applied generalized forces get overwritten on output. This is
  // not important in this case since we don't need their values anymore.
  // Please see the documentation for CalcInverseDynamics() for details.

  // With vdot = 0, this computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces.mutable_body_forces();
  VectorX<T>& tau_array = forces.mutable_generalized_forces();

  // Compute contact forces on each body by penalty method.
  if (num_collision_geometries() > 0) {
    std::vector<PenetrationAsPointPair<T>> point_pairs =
        CalcPointPairPenetrations(context);
    CalcAndAddContactForcesByPenaltyMethod(
        context, pc, vc, point_pairs, &F_BBo_W_array);
  }

  internal_tree().CalcInverseDynamics(
      context, pc, vc, vdot,
      F_BBo_W_array, tau_array,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau_array);

  vdot = M.ldlt().solve(-tau_array);

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(this->num_multibody_states());
  VectorX<T> qdot(this->num_positions());
  internal_tree().MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
ImplicitStribeckSolverResult MultibodyPlant<T>::SolveUsingSubStepping(
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
  ImplicitStribeckSolverResult info{
      ImplicitStribeckSolverResult::kMaxIterationsReached};

  for (int substep = 0; substep < num_substeps; ++substep) {
    // Discrete update before applying friction forces.
    // We denote this state x* = [q*, v*], the "star" state.
    // Generalized momentum "star", before contact forces are applied.
    VectorX<T> p_star_substep = M0 * v0_substep - dt_substep * minus_tau;

    // Update the data.
    implicit_stribeck_solver_->SetTwoWayCoupledProblemData(
        &M0, &Jn, &Jt,
        &p_star_substep, &phi0_substep,
        &stiffness, &damping, &mu);

    info = implicit_stribeck_solver_->SolveWithGuess(dt_substep,
                                                     v0_substep);

    // Break the sub-stepping loop on failure and return the info result.
    if (info != ImplicitStribeckSolverResult::kSuccess) break;

    // Update previous time step to new solution.
    v0_substep = implicit_stribeck_solver_->get_generalized_velocities();

    // Update penetration distance consistently with the solver update.
    const auto vn_substep =
        implicit_stribeck_solver_->get_normal_velocities();
    phi0_substep = phi0_substep - dt_substep * vn_substep;
  }

  return info;
}

// TODO(amcastro-tri): Consider splitting this method into smaller pieces.
template<typename T>
void MultibodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context0,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  // Assert this method was called on a context storing discrete state.
  DRAKE_ASSERT(context0.get_num_discrete_state_groups() == 1);
  DRAKE_ASSERT(context0.get_continuous_state().size() == 0);

  const double dt = time_step_;  // just a shorter alias.

  const int nq = this->num_positions();
  const int nv = this->num_velocities();

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

  const internal::PositionKinematicsCache<T>& pc0 =
      EvalPositionKinematics(context0);
  const internal::VelocityKinematicsCache<T>& vc0 =
      EvalVelocityKinematics(context0);

  // Compute forces applied through force elements.
  internal_tree().CalcForceElementsContribution(context0, pc0, vc0, &forces0);

  // If there is any input actuation, add it to the multibody forces.
  AddJointActuationForces(context0, &forces0);

  AddJointLimitsPenaltyForces(context0, &forces0);

  // TODO(amcastro-tri): Eval() point_pairs0 when caching lands.
  const std::vector<PenetrationAsPointPair<T>> point_pairs0 =
      CalcPointPairPenetrations(context0);

  // Workspace for inverse dynamics:
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(internal_tree().num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Body forces (alias to forces0).
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces0.mutable_body_forces();

  // With vdot = 0, this computes:
  //   -tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  VectorX<T>& minus_tau = forces0.mutable_generalized_forces();
  internal_tree().CalcInverseDynamics(
      context0, pc0, vc0, vdot,
      F_BBo_W_array, minus_tau,
      &A_WB_array,
      &F_BBo_W_array, /* Note: these arrays get overwritten on output. */
      &minus_tau);

  // Compute normal and tangential velocity Jacobians at t0.
  const int num_contacts = point_pairs0.size();
  MatrixX<T> Jn(num_contacts, nv);
  MatrixX<T> Jt(2 * num_contacts, nv);
  // For each contact point pair, the rotation matrix R_WC giving the
  // orientation of the contact frame C in the world frame W.
  // TODO(amcastro-tri): cache R_WC_set as soon as caching lands.
  std::vector<Matrix3<T>> R_WC_set;
  if (num_contacts > 0) {
    // TODO(amcastro-tri): when it becomes a bottleneck, update the contact
    // solver to use operators instead so that we don't have to form these
    // Jacobian matrices explicitly (an O(num_contacts * nv) operation).
    CalcNormalAndTangentContactJacobians(
        context0, point_pairs0, &Jn, &Jt, &R_WC_set);
  }

  // Get friction coefficient into a single vector. Dynamic friction is ignored
  // by the time stepping scheme.
  std::vector<CoulombFriction<double>> combined_friction_pairs =
      CalcCombinedFrictionCoefficients(point_pairs0);
  VectorX<T> mu(num_contacts);
  std::transform(combined_friction_pairs.begin(), combined_friction_pairs.end(),
                 mu.data(),
                 [](const CoulombFriction<double>& coulomb_friction) {
                   return coulomb_friction.static_friction();
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
  ImplicitStribeckSolverResult info{
      ImplicitStribeckSolverResult::kMaxIterationsReached};

  ImplicitStribeckSolverParameters params =
      implicit_stribeck_solver_->get_solver_parameters();
  // A nicely converged NR iteration should not take more than 20 iterations.
  // Otherwise we attempt a smaller time step.
  params.max_iterations = 20;
  implicit_stribeck_solver_->set_solver_parameters(params);

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
    info = SolveUsingSubStepping(
        num_substeps, M0, Jn, Jt, minus_tau, stiffness, damping, mu, v0, phi0);
  } while (info != ImplicitStribeckSolverResult::kSuccess &&
           num_substeps < kNumMaxSubTimeSteps);

  DRAKE_DEMAND(info == ImplicitStribeckSolverResult::kSuccess);

  // TODO(amcastro-tri): implement capability to dump solver statistics to a
  // file for analysis.

  // Retrieve the solution velocity for the next time step.
  VectorX<T> v_next = implicit_stribeck_solver_->get_generalized_velocities();

  VectorX<T> qdot_next(this->num_positions());
  internal_tree().MapVelocityToQDot(context0, v_next, &qdot_next);
  VectorX<T> q_next = q0 + dt * qdot_next;

  VectorX<T> x_next(this->num_multibody_states());
  x_next << q_next, v_next;
  updates->get_mutable_vector(0).SetFromVector(x_next);

  // Save contact results for analysis and visualization.
  // TODO(amcastro-tri): remove next line once caching lands since point_pairs0
  // and R_WC_set will be cached.
  CalcContactResults(context0, point_pairs0, R_WC_set, &contact_results_);
}

template<typename T>
void MultibodyPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  if (is_discrete()) return;
  const int nq = internal_tree().num_positions();
  const int nv = internal_tree().num_velocities();

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
  if (is_discrete()) return;
  const int nq = internal_tree().num_positions();
  const int nv = internal_tree().num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  internal_tree().MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void MultibodyPlant<T>::DeclareStateCacheAndPorts() {
  // The model must be finalized.
  DRAKE_DEMAND(this->is_finalized());

  if (is_discrete()) {
    this->DeclarePeriodicDiscreteUpdate(time_step_);
  }

  // TODO(sherm1) Add ContactResults cache entry.

  // Declare per model instance actuation ports.
  int num_actuated_instances = 0;
  ModelInstanceIndex last_actuated_instance;
  instance_actuation_ports_.resize(num_model_instances());
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    const int instance_num_dofs =
        internal_tree().num_actuated_dofs(model_instance_index);
    if (instance_num_dofs == 0) {
      continue;
    }
    ++num_actuated_instances;
    last_actuated_instance = model_instance_index;
    instance_actuation_ports_[model_instance_index] =
        this->DeclareVectorInputPort(
                internal_tree().GetModelInstanceName(model_instance_index) +
                    "_actuation",
                systems::BasicVector<T>(instance_num_dofs))
            .get_index();
  }

  if (num_actuated_instances == 1) {
    actuated_instance_ = last_actuated_instance;
  }

  // Declare one output port for the entire state vector.
  continuous_state_output_port_ =
      this->DeclareVectorOutputPort("continuous_state",
                                    BasicVector<T>(num_multibody_states()),
                                    &MultibodyPlant::CopyContinuousStateOut)
          .get_index();

  // Declare per model instance state output ports.
  instance_continuous_state_output_ports_.resize(num_model_instances());
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    const int instance_num_states =
        internal_tree().num_states(model_instance_index);
    if (instance_num_states == 0) {
      continue;
    }

    auto calc = [this, model_instance_index](const systems::Context<T>& context,
                                             systems::BasicVector<T>* result) {
      this->CopyContinuousStateOut(model_instance_index, context, result);
    };
    instance_continuous_state_output_ports_[model_instance_index] =
        this->DeclareVectorOutputPort(
                internal_tree().GetModelInstanceName(model_instance_index) +
                    "_continuous_state",
                BasicVector<T>(instance_num_states), calc)
            .get_index();
  }

  // Declare per model instance output port of generalized contact forces.
  instance_generalized_contact_forces_output_ports_.resize(
      num_model_instances());
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    const int instance_num_velocities =
        internal_tree().num_velocities(model_instance_index);
    if (instance_num_velocities == 0) {
      continue;
    }
    auto calc = [this, model_instance_index](const systems::Context<T>& context,
                                             systems::BasicVector<T>* result) {
      this->CopyGeneralizedContactForcesOut(
          model_instance_index, context, result);
    };
    instance_generalized_contact_forces_output_ports_[model_instance_index] =
        this->DeclareVectorOutputPort(
                internal_tree().GetModelInstanceName(model_instance_index) +
                    "_generalized_contact_forces",
                BasicVector<T>(instance_num_velocities), calc)
            .get_index();
  }

  // Contact results output port.
  contact_results_port_ = this->DeclareAbstractOutputPort(
                                  "contact_results", ContactResults<T>(),
                                  &MultibodyPlant<T>::CalcContactResultsOutput)
                              .get_index();
}

template <typename T>
const systems::BasicVector<T>& MultibodyPlant<T>::GetStateVector(
    const Context<T>& context) const {
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
      internal_tree().GetPositionsAndVelocities(context, model_instance);
  state_vector->SetFromVector(instance_state_vector);
}

template <typename T>
void MultibodyPlant<T>::CopyGeneralizedContactForcesOut(
    ModelInstanceIndex model_instance, const Context<T>&,
    BasicVector<T>* tau_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(is_discrete());

  // Vector of generalized contact forces for the entire plant's multibody
  // system.
  // TODO(amcastro-tri): Contact forces should be computed into a cache entry
  // and evaluated here. Update this to use caching as soon as the capability
  // lands.
  const VectorX<T>& tau_contact =
      implicit_stribeck_solver_->get_generalized_contact_forces();

  // Generalized velocities and generalized forces are ordered in the same way.
  // Thus we can call get_velocities_from_array().
  const VectorX<T> instance_tau_contact =
      internal_tree().GetVelocitiesFromArray(model_instance, tau_contact);

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
MultibodyPlant<T>::get_actuation_input_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  DRAKE_THROW_UNLESS(num_actuated_dofs(model_instance) > 0);
  return systems::System<T>::get_input_port(
      instance_actuation_ports_.at(model_instance));
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_continuous_state_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(continuous_state_output_port_);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_continuous_state_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  DRAKE_THROW_UNLESS(internal_tree().num_states(model_instance) > 0);
  return this->get_output_port(
      instance_continuous_state_output_ports_.at(model_instance));
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_generalized_contact_forces_output_port(
    ModelInstanceIndex model_instance) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(is_discrete());
  DRAKE_THROW_UNLESS(model_instance.is_valid());
  DRAKE_THROW_UNLESS(model_instance < num_model_instances());
  DRAKE_THROW_UNLESS(internal_tree().num_states(model_instance) > 0);
  return this->get_output_port(
      instance_generalized_contact_forces_output_ports_.at(model_instance));
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_contact_results_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_THROW_UNLESS(is_discrete());
  return this->get_output_port(contact_results_port_);
}

template <typename T>
void MultibodyPlant<T>::DeclareSceneGraphPorts() {
  geometry_query_port_ = this->DeclareAbstractInputPort(
      "geometry_query", systems::Value<geometry::QueryObject<T>>{}).get_index();
  // Allocate pose port.
  // TODO(eric.cousineau): Simplify this logic.
  typename systems::LeafOutputPort<T>::AllocCallback pose_alloc = [this]() {
    // This presupposes that the source id has been assigned and _all_ frames
    // have been registered.
    std::vector<FrameId> ids;
    for (auto it : this->body_index_to_frame_id_) {
      if (it.first == world_index()) continue;
      ids.push_back(it.second);
    }
    return systems::AbstractValue::Make(
        FramePoseVector<T>(*this->source_id_, ids));
  };
  typename systems::LeafOutputPort<T>::CalcCallback pose_callback = [this](
      const Context<T>& context, systems::AbstractValue* value) {
    this->CalcFramePoseOutput(
        context, &value->GetMutableValue<FramePoseVector<T>>());
  };
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
      "geometry_pose", pose_alloc, pose_callback).get_index();
}

template <typename T>
void MultibodyPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_ASSERT(source_id_ != nullopt);
  // NOTE: The body index to frame id map *always* includes the world body but
  // the world body does *not* get reported in the frame poses; only dynamic
  // frames do.
  DRAKE_ASSERT(
      poses->size() == static_cast<int>(body_index_to_frame_id_.size() - 1));
  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);

  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  poses->clear();
  for (const auto it : body_index_to_frame_id_) {
    const BodyIndex body_index = it.first;
    if (body_index == world_index()) continue;
    const Body<T>& body = internal_tree().get_body(body_index);

    // NOTE: The GeometryFrames for each body were registered in the world
    // frame, so we report poses in the world frame.
    poses->set_value(body_index_to_frame_id_.at(body_index),
                     pc.get_X_WB(body.node_index()));
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

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::MultibodyPlant)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct drake::multibody::AddMultibodyPlantSceneGraphResult)
