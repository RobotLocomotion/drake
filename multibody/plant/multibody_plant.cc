#include "drake/multibody/plant/multibody_plant.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/render/render_label.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sparse_linear_operator.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"

namespace drake {
namespace multibody {

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

using geometry::CollisionFilterDeclaration;
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
using drake::multibody::internal::ArticulatedBodyForceCache;
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

// TODO(rpoyner-tri): the Select*() and Expand*() functions below can be
// removed and replaced with arbitrary indexing once Eigen 3.4 is our minimum
// required version.

// Valid index arrays are no larger than max_size, are sorted, and contain
// entries in the range [0, max_size).
void DemandIndicesValid(const std::vector<int>& indices, int max_size) {
  DRAKE_DEMAND(static_cast<int>(indices.size()) <= max_size);
  if (indices.empty()) { return; }

  // Only do the expensive check in debug builds.
  DRAKE_ASSERT(std::is_sorted(indices.begin(), indices.end()));
  DRAKE_DEMAND(indices[0] >= 0);
  DRAKE_DEMAND(indices[indices.size() - 1] < max_size);
}

// Make a new, possibly smaller square matrix from square matrix @p M, by
// selecting the rows and columns indexed by @p indices.
template <typename T>
MatrixX<T> SelectRowsCols(const MatrixX<T>& M,
                          const std::vector<int>& indices) {
  DRAKE_DEMAND(M.rows() == M.cols());
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, M.rows()));
  const int selected_count = indices.size();
  if (selected_count == M.rows()) {
    return M;
  }
  MatrixX<T> result(selected_count, selected_count);

  for (int i = 0; i < result.rows(); ++i) {
    for (int j = 0; j < result.cols(); ++j) {
      result(i, j) = M(indices[i], indices[j]);
    }
  }
  return result;
}

// Make a new, possibly smaller matrix from matrix M, by selecting the columns
// indexed by @p indices.
template <typename T>
MatrixX<T> SelectCols(const MatrixX<T>& M,
                      const std::vector<int>& indices) {
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, M.cols()));
  const int selected_count = indices.size();
  if (selected_count == M.cols()) {
    return M;
  }
  MatrixX<T> result(M.rows(), selected_count);

  for (int i = 0; i < result.cols(); ++i) {
    result.col(i) = M.col(indices[i]);
  }
  return result;
}

// Make a new, possibly smaller vector from vector @p v, by selecting the rows
// indexed by @p indices.
template <typename T>
VectorX<T> SelectRows(const VectorX<T>& v,
                      const std::vector<int>& indices) {
  const int selected_count = indices.size();
  if (selected_count == v.rows()) {
    return v;
  }
  VectorX<T> result(selected_count);

  for (int i = 0; i < result.rows(); ++i) {
    result(i) = v(indices[i]);
  }
  return result;
}

// Make a new, possibly larger vector of size @p rows_out, by copying rows
// of @p v to rows indexed by @p indices. New rows are filled with zeros.
template <typename T>
VectorX<T> ExpandRows(const VectorX<T>& v, int rows_out,
                       const std::vector<int>& indices) {
  DRAKE_ASSERT(static_cast<int>(indices.size()) == v.rows());
  DRAKE_ASSERT(rows_out >= v.rows());
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, rows_out));
  if (rows_out == v.rows()) {
    return v;
  }
  VectorX<T> result(rows_out);

  int index_cursor = 0;
  for (int i = 0; i < result.rows(); ++i) {
    if (index_cursor >= v.rows() || i < indices[index_cursor]) {
      result(i) = 0.;
    } else {
      result(indices[index_cursor]) = v(index_cursor);
      ++index_cursor;
    }
  }
  return result;
}

}  // namespace

template <typename T>
MultibodyPlant<T>::MultibodyPlant(double time_step)
    : MultibodyPlant(nullptr, time_step) {
  // Cross-check that the Config default matches our header file default.
  DRAKE_DEMAND(contact_model_ == ContactModel::kPoint);
  DRAKE_DEMAND(MultibodyPlantConfig{}.contact_model == "point");
}

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
  DeclareSceneGraphPorts();
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
      const Body<T>& body = get_body(body_index);
      graphviz +=
          fmt::format(" body{} [label=\"{}\"];\n", body.index(), body.name());
    }
    graphviz += "}\n";
  }
  // Add the graph edges (via the joints).
  for (JointIndex joint_index(0); joint_index < num_joints(); ++joint_index) {
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
  source_id_ = member_scene_graph().RegisterSource(this->get_name());
  const geometry::FrameId world_frame_id =
      member_scene_graph().world_frame_id();
  body_index_to_frame_id_[world_index()] = world_frame_id;
  frame_id_to_body_index_[world_frame_id] = world_index();
  // In case any bodies were added before registering scene graph, make sure the
  // bodies get their corresponding geometry frame ids.
  RegisterGeometryFramesForAllBodies();
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
  if (properties.HasProperty("phong", "diffuse_map")) {
    perception_props.AddProperty(
        "phong", "diffuse_map",
        properties.GetProperty<std::string>("phong", "diffuse_map"));
  }
  if (properties.HasProperty("renderer", "accepting")) {
    perception_props.AddProperty(
      "renderer", "accepting",
      properties.GetProperty<std::set<std::string>>("renderer", "accepting"));
  }
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
    geometry::ProximityProperties properties) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  DRAKE_THROW_UNLESS(properties.HasProperty(geometry::internal::kMaterialGroup,
                                            geometry::internal::kFriction));

  const CoulombFriction<double> coulomb_friction =
      properties.GetProperty<CoulombFriction<double>>(
          geometry::internal::kMaterialGroup, geometry::internal::kFriction);

  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register geometry that has a fixed path to world to the world body (i.e.,
  // as anchored geometry).
  GeometryId id = RegisterGeometry(
      body, X_BG, shape, GetScopedName(*this, body.model_instance(), name));

  member_scene_graph().AssignRole(*source_id_, id, std::move(properties));
  const int collision_index = geometry_id_to_collision_index_.size();
  geometry_id_to_collision_index_[id] = collision_index;
  DRAKE_ASSERT(
      static_cast<int>(default_coulomb_friction_.size()) == collision_index);
  // TODO(SeanCurtis-TRI): Stop storing coulomb friction in MBP and simply
  //  acquire it from SceneGraph.
  default_coulomb_friction_.push_back(coulomb_friction);
  DRAKE_ASSERT(num_bodies() == static_cast<int>(collision_geometries_.size()));
  collision_geometries_[body.index()].push_back(id);
  return id;
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterCollisionGeometry(
    const Body<T>& body, const math::RigidTransform<double>& X_BG,
    const geometry::Shape& shape, const std::string& name,
    const CoulombFriction<double>& coulomb_friction) {
  geometry::ProximityProperties props;
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kFriction, coulomb_friction);
  return RegisterCollisionGeometry(body, X_BG, shape, name, std::move(props));
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
  DRAKE_ASSERT(body_has_registered_frame(body));

  // Register geometry in the body frame.
  std::unique_ptr<geometry::GeometryInstance> geometry_instance =
      std::make_unique<GeometryInstance>(X_BG, shape.Clone(), name);
  GeometryId geometry_id = member_scene_graph().RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.index()],
      std::move(geometry_instance));
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
    const Body<T>& body) {
  if (geometry_source_is_registered()) {
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
  }
}

template<typename T>
void MultibodyPlant<T>::SetFreeBodyPoseInWorldFrame(
    systems::Context<T>* context,
    const Body<T>& body, const math::RigidTransform<T>& X_WB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context);
}

template<typename T>
void MultibodyPlant<T>::SetFreeBodyPoseInAnchoredFrame(
    systems::Context<T>* context,
    const Frame<T>& frame_F, const Body<T>& body,
    const math::RigidTransform<T>& X_FB) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);

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

template <typename T>
void MultibodyPlant<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context, const VectorX<T>& known_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  this->ValidateContext(context);
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
  this->ValidateContext(context);
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
  }

  // Since currently MBP only handles joint limits for discrete models, we
  // verify that there are no joint limits when the model is continuous.
  // If there are limits defined, we prepare a warning message that will be
  // logged iff the user attempts to do anything that would have needed them.
  if (!is_discrete() && !joint_limits_parameters_.joints_with_limits.empty()) {
    std::string joint_names_with_limits;
    for (auto joint_index : joint_limits_parameters_.joints_with_limits) {
      joint_names_with_limits += fmt::format(
          ", '{}'", get_joint(joint_index).name());
    }
    joint_names_with_limits = joint_names_with_limits.substr(2);  // Nix ", ".
    joint_limits_parameters_.pending_warning_message =
        "Currently MultibodyPlant does not handle joint limits for continuous "
        "models. However some joints do specify limits. Consider setting a "
        "non-zero time step in the MultibodyPlant constructor; this will put "
        "the plant in discrete-time mode, which does support joint limits. "
        "Joints that specify limits are: " + joint_names_with_limits;
  }
}

template<typename T>
void MultibodyPlant<T>::FinalizePlantOnly() {
  DeclareStateCacheAndPorts();
  if (num_collision_geometries() > 0 &&
      penalty_method_contact_parameters_.time_scale < 0)
    EstimatePointContactParameters(penetration_allowance_);
  if (num_collision_geometries() > 0 &&
      friction_model_.stiction_tolerance() < 0)
    set_stiction_tolerance();
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
    B(actuator.joint().velocity_start(), int{actuator.index()}) = 1;
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

  struct StubCollisionFilterManager {
    void Apply(const geometry::CollisionFilterDeclaration&) {
      return;
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
  DRAKE_STUB(FrameId, RegisterFrame)
  DRAKE_STUB(GeometryId, RegisterGeometry)
  DRAKE_STUB(SourceId, RegisterSource)
  const StubSceneGraphInspector model_inspector() const {
    Throw("model_inspector");
    return StubSceneGraphInspector();
  }
  StubCollisionFilterManager collision_filter_manager() {
    Throw("collision_filter_manager");
    return StubCollisionFilterManager();
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
      member_scene_graph().collision_filter_manager().Apply(
        CollisionFilterDeclaration().ExcludeBetween(
          geometry::GeometrySet(*child_id),
          geometry::GeometrySet(*parent_id)));
    }
  }
  // We must explicitly exclude collisions between all geometries registered
  // against the world.
  // TODO(eric.cousineau): Do this in a better fashion (#11117).
  auto g_world = CollectRegisteredGeometries(GetBodiesWeldedTo(world_body()));
  member_scene_graph().collision_filter_manager().Apply(
      CollisionFilterDeclaration().ExcludeWithin(g_world));
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
  // clang-format off
  member_scene_graph().collision_filter_manager().Apply(
      CollisionFilterDeclaration()
          .ExcludeWithin(visual)
          .ExcludeBetween(visual, collision));
  // clang-format on
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
    member_scene_graph().collision_filter_manager().Apply(
        CollisionFilterDeclaration().ExcludeWithin(
            collision_filter_group_a.second));
  } else {
    member_scene_graph().collision_filter_manager().Apply(
        CollisionFilterDeclaration().ExcludeBetween(
            collision_filter_group_a.second, collision_filter_group_b.second));
  }
}

template <typename T>
void MultibodyPlant<T>::CalcNormalAndTangentContactJacobians(
    const systems::Context<T>& context,
    const std::vector<internal::DiscreteContactPair<T>>& contact_pairs,
    MatrixX<T>* Jn_ptr, MatrixX<T>* Jt_ptr,
    std::vector<RotationMatrix<T>>* R_WC_set) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(Jn_ptr != nullptr);
  DRAKE_DEMAND(Jt_ptr != nullptr);

  const int num_contacts = contact_pairs.size();

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
    const auto& point_pair = contact_pairs[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const Body<T>& bodyA = get_body(bodyA_index);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);
    const Body<T>& bodyB = get_body(bodyB_index);

    // Penetration depth > 0 if bodies interpenetrate.
    const Vector3<T>& nhat_BA_W = point_pair.nhat_BA_W;
    const Vector3<T>& p_WC = point_pair.p_WC;

    // For point Ac (origin of frame A shifted to C), calculate Jv_v_WAc (Ac's
    // translational velocity Jacobian in the world frame W with respect to
    // generalized velocities v).  Note: Ac's translational velocity in W can
    // be written in terms of this Jacobian as v_WAc = Jv_v_WAc * v.
    Matrix3X<T> Jv_WAc(3, this->num_velocities());
    internal_tree().CalcJacobianTranslationalVelocity(context,
                                                      JacobianWrtVariable::kV,
                                                      bodyA.body_frame(),
                                                      frame_W,
                                                      p_WC,
                                                      frame_W,
                                                      frame_W,
                                                      &Jv_WAc);

    // Similarly, for point Bc (origin of frame B shifted to C), calculate
    // Jv_v_WBc (Bc's translational velocity Jacobian in W with respect to v).
    Matrix3X<T> Jv_WBc(3, this->num_velocities());
    internal_tree().CalcJacobianTranslationalVelocity(context,
                                                      JacobianWrtVariable::kV,
                                                      bodyB.body_frame(),
                                                      frame_W,
                                                      p_WC,
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
    const math::RotationMatrix<T> R_WC =
        math::RotationMatrix<T>::MakeFromOneVector(nhat_BA_W, 2);
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

template <typename T>
void MultibodyPlant<T>::CalcContactJacobiansCache(
    const systems::Context<T>& context,
    internal::ContactJacobians<T>* contact_jacobians) const {
  auto& Jn = contact_jacobians->Jn;
  auto& Jt = contact_jacobians->Jt;
  auto& Jc = contact_jacobians->Jc;
  auto& R_WC_list = contact_jacobians->R_WC_list;

  this->CalcNormalAndTangentContactJacobians(
      context, EvalDiscreteContactPairs(context), &Jn, &Jt, &R_WC_list);

  Jc.resize(3 * Jn.rows(), num_velocities());
  for (int i = 0; i < Jn.rows(); ++i) {
    Jc.row(3 * i) = Jt.row(2 * i);
    Jc.row(3 * i + 1) = Jt.row(2 * i + 1);
    Jc.row(3 * i + 2) = Jn.row(i);
  }
}

template <typename T>
void MultibodyPlant<T>::SetDiscreteUpdateManager(
    std::unique_ptr<internal::DiscreteUpdateManager<T>> manager) {
  // N.B. This requirement is really more important on the side of the
  // manager's constructor, since most likely it'll need MBP's topology at
  // least to build the contact problem. However, here we play safe and demand
  // finalization right here.
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(manager != nullptr);
  manager->SetOwningMultibodyPlant(this);
  discrete_update_manager_ = std::move(manager);
  RemoveUnsupportedScalars(*discrete_update_manager_);
}

template <typename T>
void MultibodyPlant<T>::AddPhysicalModel(
    std::unique_ptr<internal::PhysicalModel<T>> model) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_DEMAND(model != nullptr);
  auto& added_model = physical_models_.emplace_back(std::move(model));
  RemoveUnsupportedScalars(*added_model);
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
void MultibodyPlant<T>::CalcPointPairPenetrations(
    const systems::Context<T>& context,
    std::vector<PenetrationAsPointPair<T>>* output) const {
  this->ValidateContext(context);
  if (num_collision_geometries() > 0) {
    const auto& query_object = EvalGeometryQueryInput(context);
    *output = query_object.ComputePointPairPenetration();
  } else {
    output->clear();
  }
}

// Specialize this function so that symbolic::Expression throws an error.
template <>
void MultibodyPlant<symbolic::Expression>::CalcPointPairPenetrations(
    const systems::Context<symbolic::Expression>&,
    std::vector<PenetrationAsPointPair<symbolic::Expression>>*) const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template <>
std::vector<CoulombFriction<double>>
MultibodyPlant<symbolic::Expression>::CalcCombinedFrictionCoefficients(
    const drake::systems::Context<symbolic::Expression>&,
    const std::vector<internal::DiscreteContactPair<symbolic::Expression>>&)
    const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template<typename T>
std::vector<CoulombFriction<double>>
MultibodyPlant<T>::CalcCombinedFrictionCoefficients(
    const drake::systems::Context<T>& context,
    const std::vector<internal::DiscreteContactPair<T>>& contact_pairs) const {
  this->ValidateContext(context);
  std::vector<CoulombFriction<double>> combined_frictions;
  combined_frictions.reserve(contact_pairs.size());

  if (contact_pairs.size() == 0) {
    return combined_frictions;
  }

  const auto& query_object = EvalGeometryQueryInput(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

  for (const auto& pair : contact_pairs) {
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const CoulombFriction<double>& geometryA_friction =
        GetCoulombFriction(geometryA_id, inspector);
    const CoulombFriction<double>& geometryB_friction =
        GetCoulombFriction(geometryB_id, inspector);

    combined_frictions.push_back(CalcContactFrictionFromSurfaceProperties(
        geometryA_friction, geometryB_friction));
  }
  return combined_frictions;
}

template<typename T>
void MultibodyPlant<T>::CopyContactResultsOutput(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(contact_results != nullptr);
  *contact_results = EvalContactResults(context);
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsContinuous(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(contact_results != nullptr);
  contact_results->Clear();
  if (num_collision_geometries() == 0) return;

  switch (contact_model_) {
    case ContactModel::kPoint:
      CalcContactResultsContinuousPointPair(context, contact_results);
      break;

    case ContactModel::kHydroelastic:
      AppendContactResultsContinuousHydroelastic(context, contact_results);
      break;

    case ContactModel::kHydroelasticWithFallback:
      // Simply merge the contributions of each contact representation.
      CalcContactResultsContinuousPointPair(context, contact_results);
      AppendContactResultsContinuousHydroelastic(context, contact_results);
      break;
  }
}

template <>
void MultibodyPlant<symbolic::Expression>::
    AppendContactResultsContinuousHydroelastic(
        const Context<symbolic::Expression>&,
        ContactResults<symbolic::Expression>*) const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template <typename T>
void MultibodyPlant<T>::AppendContactResultsContinuousHydroelastic(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  this->ValidateContext(context);
  const internal::HydroelasticContactInfoAndBodySpatialForces<T>&
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
  this->ValidateContext(context);

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      EvalPointPairPenetrations(context);

  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);
  const internal::VelocityKinematicsCache<T>& vc =
      EvalVelocityKinematics(context);

  const geometry::QueryObject<T>& query_object =
      EvalGeometryQueryInput(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

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
  contact_results->Clear();
  if (num_collision_geometries() == 0) return;

  switch (contact_model_) {
    case ContactModel::kPoint:
      CalcContactResultsDiscretePointPair(context, contact_results);
      break;

    case ContactModel::kHydroelastic:
      // N.B. We are simply computing the hydro force as function of the state,
      // not the actual discrete approximation used by the contact solver.
      AppendContactResultsContinuousHydroelastic(context, contact_results);
      break;

    case ContactModel::kHydroelasticWithFallback:
      // Simply merge the contributions of each contact representation.
      CalcContactResultsDiscretePointPair(context, contact_results);

      // N.B. We are simply computing the hydro force as function of the state,
      // not the actual discrete approximation used by the contact solver.
      AppendContactResultsContinuousHydroelastic(context, contact_results);
      break;
  }
}

template <typename T>
void MultibodyPlant<T>::CalcContactResultsDiscretePointPair(
    const systems::Context<T>& context,
    ContactResults<T>* contact_results) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(contact_results != nullptr);
  if (num_collision_geometries() == 0) return;

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      EvalPointPairPenetrations(context);
  const std::vector<RotationMatrix<T>>& R_WC_set =
      EvalContactJacobians(context).R_WC_list;
  const contact_solvers::internal::ContactSolverResults<T>& solver_results =
      EvalContactSolverResults(context);

  const VectorX<T>& fn = solver_results.fn;
  const VectorX<T>& ft = solver_results.ft;
  const VectorX<T>& vt = solver_results.vt;
  const VectorX<T>& vn = solver_results.vn;

  // The strict equality is true only when point contact is used alone.
  // Otherwise there are quadrature points in addition to the point pairs.
  const int num_contacts = point_pairs.size();
  DRAKE_DEMAND(fn.size() >= num_contacts);
  DRAKE_DEMAND(ft.size() >= 2 * num_contacts);
  DRAKE_DEMAND(vn.size() >= num_contacts);
  DRAKE_DEMAND(vt.size() >= 2 * num_contacts);

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

    // TODO(SeanCurtis-TRI) It is distinctly possible to report two contacts
    //  between bodyA and bodyB such that sometimes it gets reported (A, B) and
    //  sometimes (B, A). The example below illustrates a simple scenario in
    //  which that would occur:
    //    1. Geometry A (or B) has *multiple* collision geometries.
    //    2. Assume that the collision geometries (called 1, 2, & 3) are added
    //       in order as 1 added to A, 2 added to B, and 3 added to A.
    //       Therefore collision geometries (1 and 3) would belong to A and 2
    //       to B.
    //    3. We generate contact pairs (1, 2), (2, 3) (i.e., *both* geometries
    //       of A collide with the *single* geometry of B).
    //    4. The pairs will be reported as (1, 2) and (2, 3) (and not (3, 2))
    //       because SceneGraph guarantees that the geometry ids in the
    //       point pair results will be ordered consistently.
    //    5. So, for the first contact (1, 2), we'd get body pair (A, B). But
    //       for the second contact (2, 3), we'd get body pair (B, A).
    //  This means that any processing of contact results has to recognize that
    //  interactions between bodies can be characterized as (A, B) or (B, A) and
    //  to "combine" them, the associated quantities would have to be reversed.
    //  It would be better if MBP made the guarantee that all body pairs (A, B)
    //  are always presented as (A, B) and not (B, A). (Both in this contact as
    //  well as hydro contact). This also applies to continuous point pairs.

    // Add pair info to the contact results.
    contact_results->AddContactInfo({bodyA_index, bodyB_index, f_Bc_W, p_WC,
                                     separation_velocity, slip, pair});
  }
}

template <typename T>
void MultibodyPlant<T>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  this->ValidateContext(context);
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
void MultibodyPlant<symbolic::Expression>::CalcHydroelasticContactForces(
    const Context<symbolic::Expression>&,
    internal::HydroelasticContactInfoAndBodySpatialForces<
        symbolic::Expression>*) const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template <typename T>
void MultibodyPlant<T>::CalcHydroelasticContactForces(
    const Context<T>& context,
    internal::HydroelasticContactInfoAndBodySpatialForces<T>*
        contact_info_and_body_forces) const {
  this->ValidateContext(context);
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

  const auto& query_object = EvalGeometryQueryInput(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

  for (const ContactSurface<T>& surface : all_surfaces) {
    const GeometryId geometryM_id = surface.id_M();
    const GeometryId geometryN_id = surface.id_N();

    const ProximityProperties* propM =
        inspector.GetProximityProperties(geometryM_id);
    const ProximityProperties* propN =
        inspector.GetProximityProperties(geometryM_id);
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
    const BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryM_id);
    const BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryN_id);
    const Body<T>& bodyA = get_body(bodyA_index);
    const Body<T>& bodyB = get_body(bodyB_index);

    // The poses and spatial velocities of bodies A and B.
    const RigidTransform<T>& X_WA = bodyA.EvalPoseInWorld(context);
    const RigidTransform<T>& X_WB = bodyB.EvalPoseInWorld(context);
    const SpatialVelocity<T>& V_WA = bodyA.EvalSpatialVelocityInWorld(context);
    const SpatialVelocity<T>& V_WB = bodyB.EvalSpatialVelocityInWorld(context);

    // Pack everything calculator needs.
    typename internal::HydroelasticTractionCalculator<T>::Data data(
        X_WA, X_WB, V_WA, V_WB, &surface);

    // Combined Hunt & Crossley dissipation.
    const double dissipation = hydroelastics_engine_.CalcCombinedDissipation(
        geometryM_id, geometryN_id, inspector);

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

template <typename T>
void MultibodyPlant<T>::AddInForcesFromInputPorts(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  AddAppliedExternalGeneralizedForces(context, forces);
  AddAppliedExternalSpatialForces(context, forces);
  AddJointActuationForces(context, forces);
}

template<typename T>
void MultibodyPlant<T>::AddAppliedExternalGeneralizedForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  // If there are applied generalized forces, add them in.
  const InputPort<T>& applied_generalized_force_input =
      this->get_input_port(applied_generalized_force_input_port_);
  if (applied_generalized_force_input.HasValue(context)) {
    forces->mutable_generalized_forces() +=
        applied_generalized_force_input.Eval(context);
  }
}

template<typename T>
void MultibodyPlant<T>::AddAppliedExternalSpatialForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  // Get the mutable applied external spatial forces vector
  // (a.k.a., body force vector).
  this->ValidateContext(context);
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
  this->ValidateContext(context);
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
  this->ValidateContext(context);
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
  this->ValidateContext(context);
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
TamsiSolverResult MultibodyPlant<T>::SolveUsingSubStepping(
    TamsiSolver<T>* tamsi_solver,
    int num_substeps,
    const MatrixX<T>& M0, const MatrixX<T>& Jn, const MatrixX<T>& Jt,
    const VectorX<T>& minus_tau,
    const VectorX<T>& stiffness, const VectorX<T>& damping,
    const VectorX<T>& mu,
    const VectorX<T>& v0, const VectorX<T>& fn0) const {

  const double dt = time_step_;  // just a shorter alias.
  const double dt_substep = dt / num_substeps;
  VectorX<T> v0_substep = v0;
  VectorX<T> fn0_substep = fn0;

  // Initialize info to an unsuccessful result.
  TamsiSolverResult info{
      TamsiSolverResult::kMaxIterationsReached};

  for (int substep = 0; substep < num_substeps; ++substep) {
    // Discrete update before applying friction forces.
    // We denote this state x* = [q*, v*], the "star" state.
    // Generalized momentum "star", before contact forces are applied.
    VectorX<T> p_star_substep = M0 * v0_substep - dt_substep * minus_tau;

    // Update the data.
    tamsi_solver->SetTwoWayCoupledProblemData(
        &M0, &Jn, &Jt,
        &p_star_substep, &fn0_substep,
        &stiffness, &damping, &mu);

    info = tamsi_solver->SolveWithGuess(dt_substep, v0_substep);

    // Break the sub-stepping loop on failure and return the info result.
    if (info != TamsiSolverResult::kSuccess) break;

    // Update previous time step to new solution.
    v0_substep = tamsi_solver->get_generalized_velocities();

    // TAMSI updates each normal force according to:
    //   fₙ = (1 − d vₙ)₊ (fₙ₀ − h k vₙ)₊
    // using the last computed normal velocity vₙ and we use the shorthand
    // notation  h = dt_substep in this scope.
    // The input fₙ₀ to the solver is the undamped (no dissipation) term only.
    // We must update fₙ₀ for each substep accordingly, i.e:
    //   fₙ₀(next) = (fₙ₀(previous) − h k vₙ(next))₊
    const auto vn_substep =
        tamsi_solver->get_normal_velocities();
    fn0_substep = fn0_substep.array() -
                  dt_substep * stiffness.array() * vn_substep.array();
    fn0_substep = fn0_substep.cwiseMax(T(0.0));
  }

  return info;
}

template <typename T>
void MultibodyPlant<T>::CalcContactSurfaces(
    const drake::systems::Context<T>& context,
    std::vector<ContactSurface<T>>* contact_surfaces) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(contact_surfaces != nullptr);

  const auto& query_object = EvalGeometryQueryInput(context);

  if (is_discrete()) {
    *contact_surfaces = query_object.ComputeContactSurfaces(
        geometry::HydroelasticContactRepresentation::kPolygon);
  } else {
    *contact_surfaces = query_object.ComputeContactSurfaces(
        geometry::HydroelasticContactRepresentation::kTriangle);
  }
}

template <>
void MultibodyPlant<symbolic::Expression>::CalcContactSurfaces(
    const Context<symbolic::Expression>&,
    std::vector<geometry::ContactSurface<symbolic::Expression>>*) const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template <typename T>
void MultibodyPlant<T>::CalcHydroelasticWithFallback(
    const drake::systems::Context<T>& context,
    internal::HydroelasticFallbackCacheData<T>* data) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(data != nullptr);

  if (num_collision_geometries() > 0) {
    const auto &query_object = EvalGeometryQueryInput(context);
    data->contact_surfaces.clear();
    data->point_pairs.clear();

    if (is_discrete()) {
      query_object.ComputeContactSurfacesWithFallback(
          geometry::HydroelasticContactRepresentation::kPolygon,
          &data->contact_surfaces, &data->point_pairs);
    } else {
      query_object.ComputeContactSurfacesWithFallback(
          geometry::HydroelasticContactRepresentation::kTriangle,
          &data->contact_surfaces, &data->point_pairs);
    }
  }
}

template <>
void MultibodyPlant<symbolic::Expression>::CalcHydroelasticWithFallback(
    const drake::systems::Context<symbolic::Expression>&,
    internal::HydroelasticFallbackCacheData<symbolic::Expression>*) const {
  // TODO(SeanCurtis-TRI): Special case the AutoDiff scalar such that it works
  //  as long as there are no collisions -- akin to CalcPointPairPenetrations().
  throw std::domain_error(
      fmt::format("This method doesn't support T = {}.",
                  NiceTypeName::Get<symbolic::Expression>()));
}

template <typename T>
void MultibodyPlant<T>::CalcDiscreteContactPairs(
    const systems::Context<T>& context,
    std::vector<internal::DiscreteContactPair<T>>* result) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(result != nullptr);
  std::vector<internal::DiscreteContactPair<T>>& contact_pairs = *result;
  contact_pairs.clear();

  if (num_collision_geometries() == 0) return;

  // N.B. For discrete hydro we use a first order quadrature rule. As such,
  // the per-face quadrature point is the face's centroid and the weight is 1.
  // This is compatible with a mesh that is triangle or polygon. If we attempted
  // higher order quadrature, polygons would have to be decomposed into smaller
  // n-gons which can receive an appropriate set of quadrature points.

  // Only numeric values are supported. We detect that T is a Drake numeric type
  // using scalar_predicate::is_bool. That is true for numeric types and false
  // for symbolic.
  // If the semantics of scalar_predicate changes (or Drake types change), this
  // test may have to be revisited.
  if constexpr (scalar_predicate<T>::is_bool) {
    // We first compute the number of contact pairs so that we can allocate all
    // memory at once.
    // N.B. num_point_pairs = 0 when:
    //   1. There are legitimately no point pairs or,
    //   2. the point pair model is not even in use.
    // We guard for case (2) since EvalPointPairPenetrations() cannot be called
    // when point contact is not used and would otherwise throw an exception.
    int num_point_pairs = 0;  // The number of point contact pairs.
    if (contact_model_ == ContactModel::kPoint ||
        contact_model_ == ContactModel::kHydroelasticWithFallback) {
      num_point_pairs = EvalPointPairPenetrations(context).size();
    }

    int num_quadrature_pairs = 0;
    if (contact_model_ == ContactModel::kHydroelastic ||
        contact_model_ == ContactModel::kHydroelasticWithFallback) {
      const std::vector<geometry::ContactSurface<T>>& surfaces =
          EvalContactSurfaces(context);
      for (const auto& s : surfaces) {
        num_quadrature_pairs += s.num_faces();
      }
    }

    const int num_contact_pairs = num_point_pairs + num_quadrature_pairs;
    contact_pairs.reserve(num_contact_pairs);

    const auto& query_object = EvalGeometryQueryInput(context);
    const geometry::SceneGraphInspector<T>& inspector =
        query_object.inspector();

    // Fill in the point contact pairs.
    if (num_point_pairs > 0) {
      const std::vector<PenetrationAsPointPair<T>>& point_pairs =
          EvalPointPairPenetrations(context);
      for (const PenetrationAsPointPair<T>& pair : point_pairs) {
        const auto [kA, dA] = GetPointContactParameters(pair.id_A, inspector);
        const auto [kB, dB] = GetPointContactParameters(pair.id_B, inspector);
        const auto [k, d] =
            internal::CombinePointContactParameters(kA, kB, dA, dB);
        const T phi0 = -pair.depth;
        const T fn0 = k * pair.depth;
        DRAKE_DEMAND(fn0 >= 0);  // it should be since depth >= 0.
        // For now place contact point midway between Ca and Cb.
        // TODO(amcastro-tri): Consider using stiffness weighted location of
        // point C between Ca and Cb.
        const Vector3<T> p_WC = 0.5 * (pair.p_WCa + pair.p_WCb);
        contact_pairs.push_back(
            {pair.id_A, pair.id_B, p_WC, pair.nhat_BA_W, phi0, fn0, k, d});
      }
    }

    // Append the Hydroelastics quadrature points.
    if (num_quadrature_pairs > 0) {
      const std::vector<geometry::ContactSurface<T>>& surfaces =
          EvalContactSurfaces(context);
      for (const auto& s : surfaces) {
        // Combined Hunt & Crossley dissipation.
        const T dissipation = hydroelastics_engine_.CalcCombinedDissipation(
            s.id_M(), s.id_N(), inspector);

        for (int face = 0; face < s.num_faces(); ++face) {
          const T& Ae = s.area(face);  // Face element area.

          // We found out that the hydroelastic query might report
          // infinitesimally small faces (consider for instance an initial
          // condition that perfectly places an object at zero distance from the
          // ground.) While the area of zero sized triangles is not a problem by
          // itself, the badly computed normal on these triangles leads to
          // problems when computing the contact Jacobians (since we need to
          // obtain an orthonormal basis based on that normal.)
          // We therefore ignore infinitesimally small triangles. The tolerance
          // below is somewhat arbitrary and could possibly be tightened.
          if (Ae > 1.0e-14) {
            // N.B Assuming rigid-soft contact, and thus only a single pressure
            // gradient is considered to be valid. We first verify this indeed
            // is the case by checking that only one side has gradient
            // information (the volumetric side).
            const bool M_is_soft = s.HasGradE_M();
            const bool N_is_soft = s.HasGradE_N();
            DRAKE_DEMAND(M_is_soft != N_is_soft);

            // Pressure gradient always points into the soft geometry by
            // construction.
            const Vector3<T>& grad_pres_W = M_is_soft
                                                ? s.EvaluateGradE_M_W(face)
                                                : s.EvaluateGradE_N_W(face);

            // From ContactSurface's documentation: The normal of each face is
            // guaranteed to point "out of" N and "into" M.
            const Vector3<T>& nhat_W = s.face_normal(face);

            // Position of quadrature point Q in the world frame (since mesh_W
            // is measured and expressed in W).
            const Vector3<T>& p_WQ = s.centroid(face);
            // TODO(DamrongGuoy) EvaluateCartesian() on the triangle mesh field
            //  is expensive if the field doesn't have the gradients. The
            //  field computation for triangle representation should capture
            //  the field gradients, just like the polygon field does. (That
            //  also unifies the APIs and logic on the geometry side better.)
            // Pressure at the quadrature point.
            const T p0 = s.is_triangle()
                             ? s.tri_e_MN().EvaluateCartesian(face, p_WQ)
                             : s.poly_e_MN().EvaluateCartesian(face, p_WQ);

            // Force contribution by this quadrature point.
            const T fn0 = Ae * p0;

            // Since the normal always points into M, regardless of which body
            // is soft, we must take into account the change of sign when body
            // N is soft and M is rigid.
            const T sign = M_is_soft ? 1.0 : -1.0;

            // In order to provide some intuition, and though not entirely
            // complete, here we document the first order idea that leads to
            // the discrete hydroelastic approximation used below. In
            // hydroelastics, each quadrature point contributes to the total
            // "elastic" force along the normal direction as:
            //   f₀ₚ = ωₚ Aₑ pₚ
            // where subindex p denotes a quantity evaluated at quadrature
            // point P and subindex e identifies the e-th contact surface
            // element in which the quadrature is being evaluated.
            // Notice f₀ only includes the "elastic" contribution. Dissipation
            // is dealt with by a separate multiplicative factor. Given the
            // local stiffness for the normal forces contribution, our
            // discrete TAMSI solver implicitly handles the dissipative Hunt &
            // Crossley forces.
            // In point contact, stiffness is related to changes in the normal
            // force with changes in the penetration distance. In that spirit,
            // the approximation used here is to define the discrete
            // hydroelastics stiffness as the directional derivative of the
            // scalar force f₀ₚ along the normal direction n̂:
            //   k := ∂f₀ₚ/∂n̂ ≈ ωₚ⋅Aₑ⋅∇pₚ⋅n̂ₚ
            // that is, the variation of the normal force experiences if the
            // quadrature point is pushed inwards in the direction of the
            // normal. Notice that this expression approximates the element
            // area and normal as constant. Keeping normals and Jacobians
            // is a very common approximation in first order methods. Keeping
            // the area constant here is a higher order approximation for
            // inner triangles given that shrinkage of a triangle is related
            // the growth of a neighboring triangle (i.e. the contact surface
            // does not stretch nor shrink). For triangles close to the
            // boundary of the contact surface, this is only a first order
            // approximation.
            const T k = sign * Ae * grad_pres_W.dot(nhat_W);

            // N.B. The normal is guaranteed to point into M. However, when M
            // is soft, the gradient is not guaranteed to be in the direction
            // of the normal. The geometry code that determines which
            // triangles to keep in the contact surface may keep triangles for
            // which the pressure gradient times normal is negative (see
            // IsFaceNormalInNormalDirection() in contact_surface_utility.cc).
            // Therefore there are cases for which the definition above of k
            // might lead to negative values. We observed that this condition
            // happens sparsely at some of the boundary triangles of the
            // contact surface, while the positive values in inner triangles
            // dominates the overall compliance. In practice we did not
            // observe this to cause stability issues. Since a negative value
            // of k is correct, we decided to keep these contributions.

            // N.B. Today 01/25/2021, Only TAMSI supports discrete
            // hydroelastics and uses the discrete force fn0 instead of the
            // distance function phi0. phi0 is only used in experimental
            // ContactSolver(s). Therefore we set phi0 to NaN since it is not
            // used by TAMSI.
            const T nan_phi0 = std::numeric_limits<double>::quiet_NaN();
            contact_pairs.push_back({s.id_M(), s.id_N(), p_WQ, nhat_W, nan_phi0,
                                     fn0, k, dissipation});
          }
        }
      }
    }
  } else {
    drake::unused(context);
    throw std::domain_error(fmt::format("This method doesn't support T = {}.",
                                        NiceTypeName::Get<T>()));
  }
}

template <typename T>
void MultibodyPlant<T>::CalcContactSolverResults(
    const drake::systems::Context<T>& context0,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  // Assert this method was called on a context storing discrete state.
  this->ValidateContext(context0);
  DRAKE_ASSERT(context0.num_continuous_states() == 0);

  // We use the custom manager if provided.
  // TODO(amcastro-tri): remove the entirety of the code we are bypassing here.
  // This requires one of our custom managers to become the default
  // MultibodyPlant manager.
  if (discrete_update_manager_ != nullptr) {
    discrete_update_manager_->CalcContactSolverResults(context0, results);
    return;
  } else {
    DRAKE_ASSERT(context0.num_discrete_state_groups() == 1);
  }

  const int nq = this->num_positions();
  const int nv = this->num_velocities();

  // Quick exit if there are no moving objects.
  if (nv == 0) return;

  // Get the system state as raw Eigen vectors
  // (solution at the previous time step).
  auto x0 = context0.get_discrete_state(0).get_value();
  VectorX<T> q0 = x0.topRows(nq);
  VectorX<T> v0 = x0.bottomRows(nv);

  // Mass matrix.
  MatrixX<T> M0(nv, nv);
  internal_tree().CalcMassMatrix(context0, &M0);

  // Forces at the previous time step.
  MultibodyForces<T> forces0(internal_tree());

  CalcNonContactForces(context0, true /* discrete */, &forces0);

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
      context0, vdot, F_BBo_W_array, minus_tau, &A_WB_array,
      &F_BBo_W_array, /* Note: these arrays get overwritten on output. */
      &minus_tau);

  // Compute all contact pairs, including both penetration pairs and quadrature
  // pairs for discrete hydroelastic.
  const std::vector<internal::DiscreteContactPair<T>>& contact_pairs =
      EvalDiscreteContactPairs(context0);
  const int num_contacts = contact_pairs.size();

  // Compute normal and tangential velocity Jacobians at t0.
  const internal::ContactJacobians<T>& contact_jacobians =
      EvalContactJacobians(context0);

  // Get friction coefficient into a single vector. Static friction is ignored
  // by the time stepping scheme.
  std::vector<CoulombFriction<double>> combined_friction_pairs =
      CalcCombinedFrictionCoefficients(context0, contact_pairs);
  VectorX<T> mu(num_contacts);
  std::transform(combined_friction_pairs.begin(), combined_friction_pairs.end(),
                 mu.data(),
                 [](const CoulombFriction<double>& coulomb_friction) {
                   return coulomb_friction.dynamic_friction();
                 });

  // Fill in data as required by our discrete solver.
  VectorX<T> fn0(num_contacts);
  VectorX<T> stiffness(num_contacts);
  VectorX<T> damping(num_contacts);
  VectorX<T> phi0(num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    fn0[i] = contact_pairs[i].fn0;
    stiffness[i] = contact_pairs[i].stiffness;
    damping[i] = contact_pairs[i].damping;
    phi0[i] = contact_pairs[i].phi0;
  }

  // Joint locking: quick exit if everything is locked.
  const auto& indices = EvalJointLockingIndices(context0);
  if (indices.empty()) {
    // Everything is locked! Return a result that indicates no velocity, but
    // reports normal forces.
    results->Resize(nv, num_contacts);
    results->v_next.setZero();
    results->fn = fn0;
    results->ft.setZero();
    results->vn.setZero();
    results->vt.setZero();
    results->tau_contact = contact_jacobians.Jn.transpose() * results->fn;
    return;
  }

  // Joint locking: reduce solver inputs.
  MatrixX<T> M0_unlocked = SelectRowsCols(M0, indices);
  VectorX<T> minus_tau_unlocked = SelectRows(minus_tau, indices);
  MatrixX<T> Jn_unlocked = SelectCols(contact_jacobians.Jn, indices);
  MatrixX<T> Jt_unlocked = SelectCols(contact_jacobians.Jt, indices);
  MatrixX<T> Jc_unlocked = SelectCols(contact_jacobians.Jc, indices);

  VectorX<T> v0_unlocked = SelectRows(v0, indices);

  contact_solvers::internal::ContactSolverResults<T> results_unlocked;
  results_unlocked.Resize(indices.size(), num_contacts);

  if (contact_solver_ != nullptr) {
    CallContactSolver(contact_solver_.get(), context0.get_time(), v0_unlocked,
                      M0_unlocked, minus_tau_unlocked, phi0, Jc_unlocked,
                      stiffness, damping, mu, &results_unlocked);
  } else {
    systems::CacheEntryValue& value =
        this->get_cache_entry(cache_indexes_.contact_solver_scratch)
        .get_mutable_cache_entry_value(context0);
    auto& tamsi_solver =
        value.GetMutableValueOrThrow<TamsiSolver<T>>();
    if (tamsi_solver.get_solver_parameters().stiction_tolerance !=
        friction_model_.stiction_tolerance()) {
      // Set the stiction tolerance according to the values set by users with
      // set_stiction_tolerance().
      TamsiSolverParameters solver_parameters;
      solver_parameters.stiction_tolerance =
          friction_model_.stiction_tolerance();
      tamsi_solver.set_solver_parameters(solver_parameters);
    }

    // TAMSI is initialized with num_velocities(). Resize the internal solver
    // workspace if needed.
    tamsi_solver.ResizeIfNeeded(indices.size());

    CallTamsiSolver(&tamsi_solver, context0.get_time(), v0_unlocked,
                    M0_unlocked, minus_tau_unlocked, fn0, Jn_unlocked,
                    Jt_unlocked, stiffness, damping, mu, &results_unlocked);
  }

  // Joint locking: expand reduced outputs.
  results->v_next = ExpandRows(results_unlocked.v_next,
                                num_velocities(), indices);
  results->tau_contact =
      contact_jacobians.Jn.transpose() * results_unlocked.fn +
      contact_jacobians.Jt.transpose() * results_unlocked.ft;

  results->fn = results_unlocked.fn;
  results->ft = results_unlocked.ft;
  results->vn = results_unlocked.vn;
  results->vt = results_unlocked.vt;
}

template <typename T>
void MultibodyPlant<T>::CalcJointLockingIndices(
    const systems::Context<T>& context,
    std::vector<int>* unlocked_velocity_indices) const {
  DRAKE_DEMAND(unlocked_velocity_indices != nullptr);
  auto& indices = *unlocked_velocity_indices;
  indices.resize(num_velocities());

  int velocity_cursor = 0;
  int unlocked_cursor = 0;
  for (JointIndex joint_index(0); joint_index < num_joints(); ++joint_index) {
    const Joint<T>& joint = get_joint(joint_index);
    if (joint.is_locked(context)) {
      velocity_cursor += joint.num_velocities();
    } else {
      for (int k = 0; k < joint.num_velocities(); ++k) {
        indices[unlocked_cursor++] = velocity_cursor++;
      }
    }
  }

  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    if (!body.is_floating()) {
      continue;
    }
    if (body.is_locked(context)) {
      velocity_cursor += 6;
    } else {
      for (int k = 0; k < 6; ++k) {
        indices[unlocked_cursor++] = velocity_cursor++;
      }
    }
  }
  DRAKE_ASSERT(velocity_cursor == num_velocities());
  DRAKE_ASSERT(unlocked_cursor <= num_velocities());

  // Use size to indicate exactly how many velocities are unlocked.
  indices.resize(unlocked_cursor);
  DemandIndicesValid(indices, num_velocities());
  DRAKE_DEMAND(static_cast<int>(indices.size()) == unlocked_cursor);
}

template <typename T>
void MultibodyPlant<T>::CallTamsiSolver(
    TamsiSolver<T>* tamsi_solver,
    const T& time0, const VectorX<T>& v0, const MatrixX<T>& M0,
    const VectorX<T>& minus_tau, const VectorX<T>& fn0, const MatrixX<T>& Jn,
    const MatrixX<T>& Jt, const VectorX<T>& stiffness,
    const VectorX<T>& damping, const VectorX<T>& mu,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  // Solve for v and the contact forces.
  TamsiSolverResult info{TamsiSolverResult::kMaxIterationsReached};

  TamsiSolverParameters params = tamsi_solver->get_solver_parameters();
  // A nicely converged NR iteration should not take more than 20 iterations.
  // Otherwise we attempt a smaller time step.
  params.max_iterations = 20;
  tamsi_solver->set_solver_parameters(params);

  // We attempt to compute the update during the time interval dt using a
  // progressively larger number of sub-steps (i.e each using a smaller time
  // step than in the previous attempt). This loop breaks on the first
  // successful attempt.
  // We only allow a maximum number of trials. If the solver is unsuccessful
  // in this number of trials, the user should probably decrease the discrete
  // update time step dt or evaluate the validity of the model.
  const int kNumMaxSubTimeSteps = 20;
  int num_substeps = 0;
  do {
    ++num_substeps;
    info = SolveUsingSubStepping(tamsi_solver, num_substeps, M0, Jn, Jt,
                                 minus_tau, stiffness, damping, mu, v0, fn0);
  } while (info != TamsiSolverResult::kSuccess &&
           num_substeps < kNumMaxSubTimeSteps);

  if (info != TamsiSolverResult::kSuccess) {
    const std::string msg = fmt::format(
        "MultibodyPlant's discrete update solver failed to converge at "
        "simulation time = {:7.3g} with discrete update period = {:7.3g}. "
        "This usually means that the plant's discrete update period is too "
        "large to resolve the system's dynamics for the given simulation "
        "conditions. This is often the case during abrupt collisions or during "
        "complex and fast changing contact configurations. Another common "
        "cause is the use of high gains in the simulation of closed loop "
        "systems. These might cause numerical instabilities given our discrete "
        "solver uses an explicit treatment of actuation inputs. Possible "
        "solutions include:\n"
        "  1. reduce the discrete update period set at construction,\n"
        "  2. decrease the high gains in your controller whenever possible,\n"
        "  3. switch to a continuous model (discrete update period is zero), "
        "     though this might affect the simulation run time.",
        time0, this->time_step());
    throw std::runtime_error(msg);
  }

  // TODO(amcastro-tri): implement capability to dump solver statistics to a
  // file for analysis.

  // Update the results.
  results->v_next = tamsi_solver->get_generalized_velocities();
  results->fn = tamsi_solver->get_normal_forces();
  results->ft = tamsi_solver->get_friction_forces();
  results->vn = tamsi_solver->get_normal_velocities();
  results->vt = tamsi_solver->get_tangential_velocities();
  results->tau_contact = tamsi_solver->get_generalized_contact_forces();
}

template <>
void MultibodyPlant<symbolic::Expression>::CallContactSolver(
    contact_solvers::internal::ContactSolver<symbolic::Expression>*,
    const symbolic::Expression&, const VectorX<symbolic::Expression>&,
    const MatrixX<symbolic::Expression>&, const VectorX<symbolic::Expression>&,
    const VectorX<symbolic::Expression>&, const MatrixX<symbolic::Expression>&,
    const VectorX<symbolic::Expression>&, const VectorX<symbolic::Expression>&,
    const VectorX<symbolic::Expression>&,
    contact_solvers::internal::ContactSolverResults<symbolic::Expression>*)
    const {
  throw std::logic_error(
      "This method doesn't support T = symbolic::Expression.");
}

template <typename T>
void MultibodyPlant<T>::CallContactSolver(
    contact_solvers::internal::ContactSolver<T>* contact_solver,
    const T& time0, const VectorX<T>& v0, const MatrixX<T>& M0,
    const VectorX<T>& minus_tau, const VectorX<T>& phi0, const MatrixX<T>& Jc,
    const VectorX<T>& stiffness, const VectorX<T>& damping,
    const VectorX<T>& mu,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  // Tolerance larger than machine epsilon by an arbitrary factor. Just large
  // enough so that entries close to machine epsilon, due to round-off errors,
  // still get pruned.
  const double kPruneTolerance = 20 * std::numeric_limits<double>::epsilon();
  // TODO(amcastro-tri): Here MultibodyPlant should provide an actual O(n)
  // operator per #12210.
  const Eigen::SparseMatrix<T> Jc_sparse = Jc.sparseView(kPruneTolerance);
  const contact_solvers::internal::SparseLinearOperator<T> Jc_op("Jc",
                                                                 &Jc_sparse);

  class MassMatrixInverseOperator
      : public contact_solvers::internal::LinearOperator<T> {
   public:
    MassMatrixInverseOperator(const std::string& name, const MatrixX<T>* M)
        : contact_solvers::internal::LinearOperator<T>(name), M_ldlt_{*M} {
      DRAKE_DEMAND(M != nullptr);
      nv_ = M->rows();
      // TODO(sherm1) Eliminate heap allocation.
      tmp_.resize(nv_);
    }
    ~MassMatrixInverseOperator() = default;

    int rows() const { return nv_; }
    int cols() const { return nv_; }

   private:
    void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                    Eigen::SparseVector<T>* y) const final {
      tmp_ = VectorX<T>(x);
      *y = M_ldlt_.Solve(tmp_).sparseView();
    }
    void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                    VectorX<T>* y) const final {
      *y = M_ldlt_.Solve(x);
    }
    int nv_;
    mutable VectorX<T> tmp_;  // temporary workspace.
    math::LinearSolver<Eigen::LDLT, MatrixX<T>> M_ldlt_;
  };
  MassMatrixInverseOperator Minv_op("Minv", &M0);

  // Perform the "predictor" step, in the absence of contact forces. See
  // ContactSolver's class documentation for details.
  // TODO(amcastro-tri): here the predictor step could be implicit in tau so
  // that for instance we'd be able do deal with force elements implicitly.
  const int nv = num_velocities();
  VectorX<T> v_star(nv);  // TODO(sherm1) Eliminate heap allocation.
  Minv_op.Multiply(minus_tau, &v_star);  // v_star = -M⁻¹⋅τ
  v_star *= -time_step();                // v_star = dt⋅M⁻¹⋅τ
  v_star += v0;                          // v_star = v₀ + dt⋅M⁻¹⋅τ

  contact_solvers::internal::SystemDynamicsData<T> dynamics_data(&Minv_op,
                                                                 &v_star);
  contact_solvers::internal::PointContactData<T> contact_data(
      &phi0, &Jc_op, &stiffness, &damping, &mu);
  const contact_solvers::internal::ContactSolverStatus info =
      contact_solver->SolveWithGuess(time_step(), dynamics_data, contact_data,
                                    v0, &*results);

  if (info != contact_solvers::internal::ContactSolverStatus::kSuccess) {
    const std::string msg =
        fmt::format("MultibodyPlant's contact solver of type '" +
                        NiceTypeName::Get(*contact_solver_) +
                        "' failed to converge at "
                        "simulation time = {:7.3g} with discrete update "
                        "period = {:7.3g}.",
                    time0, time_step());
    throw std::runtime_error(msg);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcGeneralizedContactForcesContinuous(
    const Context<T>& context, VectorX<T>* tau_contact) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(tau_contact != nullptr);
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
  this->ValidateContext(context);
  DRAKE_DEMAND(F_BBo_W_array != nullptr);
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
    case ContactModel::kPoint:
      // Note: consider caching the results from the following method (in which
      // case we would also want to introduce the Eval... naming convention for
      // the method).
      CalcAndAddContactForcesByPenaltyMethod(context, &(*F_BBo_W_array));
      break;

    case ContactModel::kHydroelastic:
      *F_BBo_W_array = EvalHydroelasticContactForces(context).F_BBo_W_array;
      break;

    case ContactModel::kHydroelasticWithFallback:
      // Combine the point-penalty forces with the contact surface forces.
      CalcAndAddContactForcesByPenaltyMethod(context, &(*F_BBo_W_array));
      const std::vector<SpatialForce<T>>& Fhydro_BBo_W_all =
          EvalHydroelasticContactForces(context).F_BBo_W_array;
      DRAKE_DEMAND(F_BBo_W_array->size() == Fhydro_BBo_W_all.size());
      for (int i = 0; i < static_cast<int>(Fhydro_BBo_W_all.size()); ++i) {
        // Both sets of forces are applied to the body's origins and expressed
        // in frame W. They should simply sum.
        (*F_BBo_W_array)[i] += Fhydro_BBo_W_all[i];
      }
      break;
  }
}

template <typename T>
void MultibodyPlant<T>::CalcNonContactForces(
    const drake::systems::Context<T>& context,
    bool discrete,
    MultibodyForces<T>* forces) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(*this));

  // Compute forces applied through force elements. Note that this resets
  // forces to empty so must come first.
  CalcForceElementsContribution(context, forces);

  AddInForcesFromInputPorts(context, forces);

  // Only discrete models support joint limits.
  if (discrete) {
    AddJointLimitsPenaltyForces(context, forces);
  } else {
    auto& warning = joint_limits_parameters_.pending_warning_message;
    if (!warning.empty()) {
      drake::log()->warn(warning);
      warning.clear();
    }
  }
}

template <typename T>
void MultibodyPlant<T>::AddInForcesContinuous(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  this->ValidateContext(context);

  // Forces from MultibodyTree elements are handled in MultibodyTreeSystem;
  // we need only handle MultibodyPlant-specific forces here.
  AddInForcesFromInputPorts(context, forces);

  // Add the contribution of contact forces.
  std::vector<SpatialForce<T>>& Fapp_BBo_W_array =
      forces->mutable_body_forces();
  const std::vector<SpatialForce<T>>& Fcontact_BBo_W_array =
      EvalSpatialContactForcesContinuous(context);
  for (int i = 0; i < static_cast<int>(Fapp_BBo_W_array.size()); ++i)
    Fapp_BBo_W_array[i] += Fcontact_BBo_W_array[i];
}

template <typename T>
void MultibodyPlant<T>::DoCalcForwardDynamicsDiscrete(
    const drake::systems::Context<T>& context0,
    AccelerationKinematicsCache<T>* ac) const {
  this->ValidateContext(context0);
  DRAKE_DEMAND(ac != nullptr);
  DRAKE_DEMAND(is_discrete());

  // TODO(amcastro-tri): remove the entirety of the code we are bypassing here.
  // This requires one of our custom managers to become the default
  // MultibodyPlant manager.
  if (discrete_update_manager_) {
    discrete_update_manager_->CalcAccelerationKinematicsCache(context0, ac);
    return;
  }

  // Evaluate contact results.
  const contact_solvers::internal::ContactSolverResults<T>& solver_results =
      EvalContactSolverResults(context0);

  // Retrieve the solution velocity for the next time step.
  const VectorX<T>& v_next = solver_results.v_next;

  auto x0 = context0.get_discrete_state(0).get_value();
  const VectorX<T> v0 = x0.bottomRows(this->num_velocities());

  ac->get_mutable_vdot() = (v_next - v0) / time_step();

  // N.B. Pool of spatial accelerations indexed by BodyNodeIndex.
  internal_tree().CalcSpatialAccelerationsFromVdot(
      context0, EvalPositionKinematics(context0),
      EvalVelocityKinematics(context0), ac->get_vdot(),
      &ac->get_mutable_A_WB_pool());
}

template<typename T>
void MultibodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context0,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  this->ValidateContext(context0);

  // TODO(amcastro-tri): remove the entirety of the code we are bypassing here.
  // This requires one of our custom managers to become the default
  // MultibodyPlant manager.
  if (discrete_update_manager_) {
    discrete_update_manager_->CalcDiscreteValues(context0, updates);
    return;
  }

  // Get the system state as raw Eigen vectors
  // (solution at the previous time step).
  auto x0 = context0.get_discrete_state(0).get_value();
  VectorX<T> q0 = x0.topRows(this->num_positions());
  VectorX<T> v0 = x0.bottomRows(this->num_velocities());

  // For a discrete model this evaluates vdot = (v_next - v0)/time_step() and
  // includes contact forces.
  const VectorX<T>& vdot = this->EvalForwardDynamics(context0).get_vdot();

  // TODO(amcastro-tri): Consider replacing this by:
  //   const VectorX<T>& v_next = solver_results.v_next;
  // to avoid additional vector operations.
  const VectorX<T>& v_next = v0 + time_step() * vdot;

  VectorX<T> qdot_next(this->num_positions());
  MapVelocityToQDot(context0, v_next, &qdot_next);
  VectorX<T> q_next = q0 + time_step() * qdot_next;

  VectorX<T> x_next(this->num_multibody_states());
  x_next << q_next, v_next;
  updates->set_value(0, x_next);
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
                instance_num_dofs)
            .get_index();
  }

  if (num_actuated_instances == 1) actuated_instance_ = last_actuated_instance;

  // Declare the generalized force input port.
  applied_generalized_force_input_port_ =
      this->DeclareVectorInputPort("applied_generalized_force",
                                   num_velocities())
          .get_index();

  // Declare applied spatial force input force port.
  applied_spatial_force_input_port_ = this->DeclareAbstractInputPort(
        "applied_spatial_force",
        Value<std::vector<ExternallyAppliedSpatialForce<T>>>()).get_index();

  // Declare one output port for the entire state vector.
  // TODO(sherm1) Rename this port to just "state" when #12214 is resolved so
  //              we can deprecate the old port name.
  state_output_port_ =
      this->DeclareVectorOutputPort("continuous_state", num_multibody_states(),
                                    &MultibodyPlant::CopyMultibodyStateOut,
                                    {this->all_state_ticket()})
          .get_index();

  // Declare the output port for the poses of all bodies in the world.
  body_poses_port_ =
      this->DeclareAbstractOutputPort(
              "body_poses", std::vector<math::RigidTransform<T>>(num_bodies()),
              &MultibodyPlant<T>::CalcBodyPosesOutput,
              {this->configuration_ticket()})
          .get_index();

  // Declare the output port for the spatial velocities of all bodies in the
  // world.
  body_spatial_velocities_port_ =
      this->DeclareAbstractOutputPort(
              "spatial_velocities",
              std::vector<SpatialVelocity<T>>(num_bodies()),
              &MultibodyPlant<T>::CalcBodySpatialVelocitiesOutput,
              {this->kinematics_ticket()})
          .get_index();

  // Declare the output port for the spatial accelerations of all bodies in the
  // world.
  body_spatial_accelerations_port_ =
      this->DeclareAbstractOutputPort(
              "spatial_accelerations",
              std::vector<SpatialAcceleration<T>>(num_bodies()),
              &MultibodyPlant<T>::CalcBodySpatialAccelerationsOutput,
              // Accelerations depend on both state and inputs.
              // All sources include: time, accuracy, state, input ports, and
              // parameters.
              {this->all_sources_ticket()})
          .get_index();

  // Declare one output port for the entire generalized acceleration vector
  // vdot (length is nv).
  generalized_acceleration_output_port_ =
      this->DeclareVectorOutputPort(
              "generalized_acceleration", num_velocities(),
              [this](const systems::Context<T>& context,
                     systems::BasicVector<T>* result) {
                result->SetFromVector(
                    this->EvalForwardDynamics(context).get_vdot());
              },
              {this->acceleration_kinematics_cache_entry().ticket()})
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
                instance_name + "_continuous_state", instance_num_states,
                [this, model_instance_index](const systems::Context<T>& context,
                                             systems::BasicVector<T>* result) {
                  this->CopyMultibodyStateOut(model_instance_index, context,
                                               result);
                },
                {this->all_state_ticket()})
            .get_index();

    const int instance_num_velocities =  // Might be zero.
        num_velocities(model_instance_index);
    instance_generalized_acceleration_output_ports_[model_instance_index] =
        this->DeclareVectorOutputPort(
                instance_name + "_generalized_acceleration",
                instance_num_velocities,
                [this, model_instance_index](const systems::Context<T>& context,
                                             systems::BasicVector<T>* result) {
                  const auto& vdot =
                      this->EvalForwardDynamics(context).get_vdot();
                  result->SetFromVector(
                      this->GetVelocitiesFromArray(model_instance_index, vdot));
                },
                {this->acceleration_kinematics_cache_entry().ticket()})
            .get_index();
  }

  // Declare per model instance output port of generalized contact forces.
  instance_generalized_contact_forces_output_ports_.resize(
      num_model_instances());
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    const int instance_num_velocities = num_velocities(model_instance_index);

    if (is_discrete()) {
      const auto& contact_solver_results_cache_entry =
          this->get_cache_entry(cache_indexes_.contact_solver_results);
      auto calc = [this, model_instance_index](
                      const systems::Context<T>& context,
                      systems::BasicVector<T>* result) {
        const contact_solvers::internal::ContactSolverResults<T>&
            solver_results = EvalContactSolverResults(context);
        this->CopyGeneralizedContactForcesOut(solver_results,
                                              model_instance_index, result);
      };
      instance_generalized_contact_forces_output_ports_[model_instance_index] =
          this->DeclareVectorOutputPort(
                  GetModelInstanceName(model_instance_index) +
                      "_generalized_contact_forces",
                  instance_num_velocities, calc,
                  {contact_solver_results_cache_entry.ticket()})
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
                  instance_num_velocities, calc,
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
              {this->acceleration_kinematics_cache_entry().ticket()})
          .get_index();

  // Contact results output port.
  const auto& contact_results_cache_entry =
      this->get_cache_entry(cache_indexes_.contact_results);
  contact_results_port_ = this->DeclareAbstractOutputPort(
                                  "contact_results", ContactResults<T>(),
                                  &MultibodyPlant<T>::CopyContactResultsOutput,
                                  {contact_results_cache_entry.ticket()})
                              .get_index();

  // Let external model managers declare their state, cache and ports in
  // `this` MultibodyPlant.
  for (auto& physical_model : physical_models_) {
    physical_model->DeclareSystemResources(this);
  }
}

template <typename T>
void MultibodyPlant<T>::DeclareCacheEntries() {
  DRAKE_DEMAND(this->is_finalized());

  // TODO(joemasterjohn): Create more granular parameter tickets for finer
  // control over cache dependencies on parameters. For example,
  // all_rigid_body_parameters, etc.

  // TODO(SeanCurtis-TRI): When SG caches the results of these queries itself,
  //  (https://github.com/RobotLocomotion/drake/issues/12767), remove these
  //  cache entries.
  auto& hydro_point_cache_entry = this->DeclareCacheEntry(
      std::string("Hydroelastic contact with point-pair fallback"),
      &MultibodyPlant::CalcHydroelasticWithFallback,
      {this->configuration_ticket()});
  cache_indexes_.hydro_fallback = hydro_point_cache_entry.cache_index();

  // Cache entry for point contact queries.
  auto& point_pairs_cache_entry = this->DeclareCacheEntry(
      std::string("Point pair penetrations."),
      &MultibodyPlant<T>::CalcPointPairPenetrations,
      {this->configuration_ticket()});
  cache_indexes_.point_pairs = point_pairs_cache_entry.cache_index();

  // Cache entry for hydroelastic contact surfaces.
  auto& contact_surfaces_cache_entry = this->DeclareCacheEntry(
      std::string("Hydroelastic contact surfaces."),
      &MultibodyPlant<T>::CalcContactSurfaces,
      {this->configuration_ticket()});
  cache_indexes_.contact_surfaces = contact_surfaces_cache_entry.cache_index();

  // Cache contact Jacobians.
  auto& contact_jacobians_cache_entry = this->DeclareCacheEntry(
      std::string("Contact Jacobians Jn(q), Jt(q), Jc(q) and frames R_WC."),
      &MultibodyPlant<T>::CalcContactJacobiansCache,
      {this->configuration_ticket(), this->all_parameters_ticket()});
  cache_indexes_.contact_jacobians =
      contact_jacobians_cache_entry.cache_index();

  if (is_discrete()) {
    // Cache TamsiSolver computations.
    auto& tamsi_results_cache_entry = this->DeclareCacheEntry(
        std::string("Implicit Stribeck solver computations."),
        &MultibodyPlant<T>::CalcContactSolverResults,
        // The Correct Solution:
        // The Implicit Stribeck solver solution S is a function of state x,
        // actuation input u (and externally applied forces) and even time if
        // any of the force elements in the model is time dependent. We can
        // write this as S = S(t, x, u).
        // Even though this variables can change continuously with time, we
        // want the solver solution to be updated periodically (with period
        // time_step()) only. That is, ContactSolverResults should be handled
        // as an abstract state with periodic updates. In the systems::
        // framework terminology, we'd like to have an "unrestricted update"
        // with a periodic event trigger.
        // The Problem (#10149):
        // From issue #10149 we know unrestricted updates incur a very
        // noticeably performance hit that at this stage we are not willing to
        // pay.
        // The Work Around (#10888):
        // To emulate the correct behavior until #10149 is addressed we declare
        // the Implicit Stribeck solver solution dependent only on the discrete
        // state. This is not the correct solution given these results do
        // depend on time and (even continuous) inputs. However it does emulate
        // the discrete update of these values as if zero-order held, which is
        // what we want.
        {this->xd_ticket(), this->all_parameters_ticket()});
    cache_indexes_.contact_solver_results =
        tamsi_results_cache_entry.cache_index();

    // This cache entry holds the entire TAMSI solver, since there was no
    // convenient way to isolate just the scratch data in its current form.
    auto& tamsi_scratch_cache_entry = this->DeclareCacheEntry(
        "solver scratch", systems::ValueProducer(
            TamsiSolver<T>(num_velocities()),
            &systems::ValueProducer::NoopCalc),
        {this->nothing_ticket()});
    cache_indexes_.contact_solver_scratch =
        tamsi_scratch_cache_entry.cache_index();
  }


  // Cache entry for spatial forces and contact info due to hydroelastic
  // contact.
  const bool use_hydroelastic =
      contact_model_ == ContactModel::kHydroelastic ||
      contact_model_ == ContactModel::kHydroelasticWithFallback;
  if (use_hydroelastic) {
    auto& contact_info_and_body_spatial_forces_cache_entry =
        this->DeclareCacheEntry(
            std::string("Hydroelastic contact info and body spatial forces."),
            internal::HydroelasticContactInfoAndBodySpatialForces<T>(
                this->num_bodies()),
            &MultibodyPlant<T>::CalcHydroelasticContactForces,
            // Compliant contact forces due to hydroelastics with Hunt &
            // Crosseley are function of the kinematic variables q & v only.
            {this->kinematics_ticket(), this->all_parameters_ticket()});
    cache_indexes_.contact_info_and_body_spatial_forces =
        contact_info_and_body_spatial_forces_cache_entry.cache_index();
  }

  // Cache contact results.
  // In discrete mode contact forces computation requires to advance the system
  // from step n to n+1. Therefore they are a function of state and input.
  // In continuous mode contact forces are simply a function of state.
  std::set<systems::DependencyTicket> dependency_ticket = [this,
                                                           use_hydroelastic]() {
    std::set<systems::DependencyTicket> tickets;
    if (is_discrete()) {
      tickets.insert(
          this->cache_entry_ticket(cache_indexes_.contact_solver_results));
    } else {
      tickets.insert(this->kinematics_ticket());
      if (use_hydroelastic) {
        tickets.insert(this->cache_entry_ticket(
            cache_indexes_.contact_info_and_body_spatial_forces));
      }
    }
    tickets.insert(this->all_parameters_ticket());

    return tickets;
  }();
  auto& contact_results_cache_entry = this->DeclareCacheEntry(
      std::string("Contact results."),
      is_discrete() ?
          &MultibodyPlant<T>::CalcContactResultsDiscrete :
          &MultibodyPlant<T>::CalcContactResultsContinuous,
      {dependency_ticket});
  cache_indexes_.contact_results = contact_results_cache_entry.cache_index();

  // Cache spatial continuous contact forces.
  auto& spatial_contact_forces_continuous_cache_entry = this->DeclareCacheEntry(
      "Spatial contact forces (continuous).",
      std::vector<SpatialForce<T>>(num_bodies()),
      &MultibodyPlant::CalcSpatialContactForcesContinuous,
      {this->kinematics_ticket(), this->all_parameters_ticket()});
  cache_indexes_.spatial_contact_forces_continuous =
      spatial_contact_forces_continuous_cache_entry.cache_index();

  // Cache generalized continuous contact forces.
  auto& generalized_contact_forces_continuous_cache_entry =
      this->DeclareCacheEntry(
          "Generalized contact forces (continuous).",
          VectorX<T>(num_velocities()),
          &MultibodyPlant::CalcGeneralizedContactForcesContinuous,
          {this->cache_entry_ticket(
               cache_indexes_.spatial_contact_forces_continuous),
           this->all_parameters_ticket()});
  cache_indexes_.generalized_contact_forces_continuous =
      generalized_contact_forces_continuous_cache_entry.cache_index();

  // Cache discrete contact pairs.
  const auto& discrete_contact_pairs_cache_entry = this->DeclareCacheEntry(
      "Discrete contact pairs.",
      &MultibodyPlant::CalcDiscreteContactPairs,
      {this->xd_ticket(), this->all_parameters_ticket()});
  cache_indexes_.discrete_contact_pairs =
      discrete_contact_pairs_cache_entry.cache_index();

  // Cache joint locking indices.
  const auto& joint_locking_data_cache_entry =
      this->DeclareCacheEntry("Joint Locking Indices.",
                              std::vector<int>(),
                              &MultibodyPlant::CalcJointLockingIndices,
                              {this->all_parameters_ticket()});
  cache_indexes_.joint_locking_data =
      joint_locking_data_cache_entry.cache_index();
}

template <typename T>
void MultibodyPlant<T>::CopyMultibodyStateOut(
    const Context<T>& context, BasicVector<T>* state_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  state_vector->SetFromVector(GetPositionsAndVelocities(context));
}

template <typename T>
void MultibodyPlant<T>::CopyMultibodyStateOut(
    ModelInstanceIndex model_instance,
    const Context<T>& context, BasicVector<T>* state_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  state_vector->SetFromVector(
      GetPositionsAndVelocities(context, model_instance));
}

template <typename T>
void MultibodyPlant<T>::CopyGeneralizedContactForcesOut(
    const contact_solvers::internal::ContactSolverResults<T>& solver_results,
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
using ModelQueryObject = typename std::conditional_t<
    std::is_same_v<T, symbolic::Expression>,
    SymbolicGeometryValue, geometry::QueryObject<T>>;
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
void MultibodyPlant<T>::CalcBodyPosesOutput(
    const Context<T>& context,
    std::vector<math::RigidTransform<T>>* X_WB_all) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  X_WB_all->resize(num_bodies());
  for (BodyIndex body_index(0); body_index < this->num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    X_WB_all->at(body_index) = EvalBodyPoseInWorld(context, body);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcBodySpatialVelocitiesOutput(
    const Context<T>& context,
    std::vector<SpatialVelocity<T>>* V_WB_all) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  V_WB_all->resize(num_bodies());
  for (BodyIndex body_index(0); body_index < this->num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    V_WB_all->at(body_index) = EvalBodySpatialVelocityInWorld(context, body);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcBodySpatialAccelerationsOutput(
    const Context<T>& context,
    std::vector<SpatialAcceleration<T>>* A_WB_all) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  A_WB_all->resize(num_bodies());
  const AccelerationKinematicsCache<T>& ac = this->EvalForwardDynamics(context);
  for (BodyIndex body_index(0); body_index < this->num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    A_WB_all->at(body_index) = ac.get_A_WB(body.node_index());
  }
}

template <typename T>
const SpatialAcceleration<T>&
MultibodyPlant<T>::EvalBodySpatialAccelerationInWorld(
    const Context<T>& context,
    const Body<T>& body_B) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  DRAKE_DEMAND(this == &body_B.GetParentPlant());
  this->ValidateContext(context);
  const AccelerationKinematicsCache<T>& ac = this->EvalForwardDynamics(context);
  return ac.get_A_WB(body_B.node_index());
}

template <typename T>
void MultibodyPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  this->ValidateContext(context);
  const internal::PositionKinematicsCache<T>& pc =
      EvalPositionKinematics(context);

  // NOTE: The body index to frame id map *always* includes the world body but
  // the world body does *not* get reported in the frame poses; only dynamic
  // frames do.
  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  poses->clear();
  for (const auto& it : body_index_to_frame_id_) {
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
  this->ValidateContext(context);
  DRAKE_DEMAND(F_CJc_Jc_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(F_CJc_Jc_array->size()) == num_joints());

  const VectorX<T>& vdot = this->EvalForwardDynamics(context).get_vdot();

  // TODO(sherm1) EvalForwardDynamics() should record the forces it used
  //              so that we don't have to attempt to reconstruct them
  //              here (and this is broken, see #13888).
  MultibodyForces<T> applied_forces(*this);
  CalcNonContactForces(context, is_discrete(), &applied_forces);
  auto& Fapplied_Bo_W_array = applied_forces.mutable_body_forces();
  auto& tau_applied = applied_forces.mutable_generalized_forces();

  // TODO(sherm1) This doesn't include hydroelastic contact forces
  //              in continuous mode (#13888).
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
  // TODO(amcastro-tri) Uncomment this line once issue #12473 is resolved.
  // DRAKE_ASSERT(tau_id.norm() <
  //              100 * num_velocities() *
  //              std::numeric_limits<double>::epsilon());

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
      const RotationMatrix<T> R_WJp =
          frame_Jp.CalcRotationMatrixInWorld(context);
      const RigidTransform<T> X_JpJc = frame_Jc.CalcPose(context, frame_Jp);
      const Vector3<T> p_JpJc_Jp = X_JpJc.translation();
      const Vector3<T> p_JpJc_W = R_WJp * p_JpJc_Jp;

      // Finally, we shift the spatial force at Jp.
      F_CJc_W = F_CJp_W.Shift(p_JpJc_W);
    }

    // Re-express in the joint's child frame Jc.
    const RotationMatrix<T> R_WJc = frame_Jc.CalcRotationMatrixInWorld(context);
    const RotationMatrix<T> R_JcW = R_WJc.inverse();
    F_CJc_Jc_array->at(joint_index) = R_JcW * F_CJc_W;
  }
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_body_poses_output_port()
const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return systems::System<T>::get_output_port(body_poses_port_);
}

template <typename T>
const OutputPort<T>&
MultibodyPlant<T>::get_body_spatial_velocities_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return systems::System<T>::get_output_port(body_spatial_velocities_port_);
}

template <typename T>
const OutputPort<T>&
MultibodyPlant<T>::get_body_spatial_accelerations_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return systems::System<T>::get_output_port(body_spatial_accelerations_port_);
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_poses_output_port()
const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
const systems::InputPort<T>&
MultibodyPlant<T>::get_geometry_query_input_port() const {
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
  DRAKE_THROW_UNLESS(plant != nullptr);
  plant->set_name("plant");
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

template <typename T>
AddMultibodyPlantSceneGraphResult<T> AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder, double time_step,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph) {
  DRAKE_DEMAND(builder != nullptr);
  auto plant = std::make_unique<MultibodyPlant<T>>(time_step);
  plant->set_name("plant");
  return AddMultibodyPlantSceneGraph(builder, std::move(plant),
                                     std::move(scene_graph));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    /* Use static_cast to disambiguate the two different overloads. */
    static_cast<AddMultibodyPlantSceneGraphResult<T>(*)(
        systems::DiagramBuilder<T>*, double,
        std::unique_ptr<geometry::SceneGraph<T>>)>(
            &AddMultibodyPlantSceneGraph),
    /* Use static_cast to disambiguate the two different overloads. */
    static_cast<AddMultibodyPlantSceneGraphResult<T>(*)(
        systems::DiagramBuilder<T>*,
        std::unique_ptr<MultibodyPlant<T>>,
        std::unique_ptr<geometry::SceneGraph<T>>)>(
            &AddMultibodyPlantSceneGraph)
))

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::MultibodyPlant)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct drake::multibody::AddMultibodyPlantSceneGraphResult)
