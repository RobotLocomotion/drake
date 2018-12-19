#include "drake/examples/scene_graph/solar_system.h"

#include <memory>
#include <string>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_values.h"

namespace drake {
namespace examples {
namespace solar_system {

using Eigen::Vector4d;
using geometry::Box;
using geometry::Convex;
using geometry::Cylinder;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::MakeDrakeVisualizerProperties;
using geometry::SceneGraph;
using geometry::Mesh;
using geometry::SourceId;
using geometry::Sphere;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::DiscreteValues;
using std::make_unique;

template <typename T>
SolarSystem<T>::SolarSystem(SceneGraph<T>* scene_graph) {
  DRAKE_DEMAND(scene_graph);
  source_id_ = scene_graph->RegisterSource("solar_system");

  this->DeclareContinuousState(kBodyCount /* num_q */, kBodyCount /* num_v */,
                               0 /* num_z */);

  AllocateGeometry(scene_graph);

  // Now that frames have been registered, allocate the output port.
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
          FramePoseVector<T>(source_id_, body_ids_),
          &SolarSystem::CalcFramePoseOutput)
      .get_index();
}

template <typename T>
const systems::OutputPort<T>& SolarSystem<T>::get_geometry_pose_output_port()
    const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
void SolarSystem<T>::SetDefaultState(const systems::Context<T>&,
                     systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  ContinuousState<T>& xc = state->get_mutable_continuous_state();
  VectorX<T> initial_state;
  initial_state.resize(kBodyCount * 2);
  // clang-format off
  initial_state << 0,               // Earth initial position
                   M_PI / 2,        // moon initial position
                    7 * M_PI / 6,   // convexsat initial position
                   11 * M_PI / 6,   // boxsat initial position
                   M_PI / 2,        // Mars initial position
                   0,               // phobos initial position
                   2 * M_PI / 5,    // Earth revolution lasts 5 seconds.
                   2 * M_PI,        // moon revolution lasts 1 second.
                   2 * M_PI,        // convexsat revolution lasts 1 second.
                   2 * M_PI,        // boxsat revolution lasts 1 second.
                   2 * M_PI / 6,    // Mars revolution lasts 6 seconds.
                   2 * M_PI / 1.1;  // phobos revolution lasts 1.1 seconds.
  // clang-format on
  DRAKE_DEMAND(xc.size() == initial_state.size());
  xc.SetFromVector(initial_state);
  DiscreteValues<T>& xd = state->get_mutable_discrete_state();
  for (int i = 0; i < xd.num_groups(); i++) {
    BasicVector<T>& s = xd.get_mutable_vector(i);
    s.SetFromVector(VectorX<T>::Zero(s.size()));
  }
}

// Registers geometry to form an L-shaped arm onto the given frame. The arm is
// defined as shown below:
//
//                        ◯          ← z = height
//   x = 0                │
//   ↓                    │ height
//   ─────────────────────┘          ← z = 0
//                        ↑
//                        x = length
//
// The arm's horizontal length is oriented with the x-axis. The vertical length
// is oriented with the z-axis. The origin of the arm is defined at the local
// origin, and the top of the arm is positioned at the given height.
template <class ParentId>
void MakeArm(SourceId source_id, ParentId parent_id, double length,
             double height, double radius,
             const IllustrationProperties& material,
             SceneGraph<double>* scene_graph) {
  Isometry3<double> arm_pose = Isometry3<double>::Identity();
  // tilt it horizontally
  arm_pose.linear() =
      Eigen::AngleAxis<double>(M_PI / 2, Vector3<double>::UnitY()).matrix();
  arm_pose.translation() << length / 2, 0, 0;
  GeometryId id = scene_graph->RegisterGeometry(
      source_id, parent_id, make_unique<GeometryInstance>(
                                arm_pose, make_unique<Cylinder>(radius, length),
                                "horz_arm"));
  scene_graph->AssignRole(source_id, id, material);

  Isometry3<double> post_pose = Isometry3<double>::Identity();
  post_pose.translation() << length, 0, height / 2;
  id = scene_graph->RegisterGeometry(
      source_id, parent_id,
      make_unique<GeometryInstance>(post_pose,
                                    make_unique<Cylinder>(radius, height),
                                    "vert_arm"));
  scene_graph->AssignRole(source_id, id, material);
}

template <typename T>
void SolarSystem<T>::AllocateGeometry(SceneGraph<T>* scene_graph) {
  body_ids_.reserve(kBodyCount);
  body_offset_.reserve(kBodyCount);
  axes_.reserve(kBodyCount);

  IllustrationProperties post_material =
      MakeDrakeVisualizerProperties(Vector4d(0.3, 0.15, 0.05, 1));
  const double orrery_bottom = -1.5;
  const double pipe_radius = 0.05;

  // Allocate the sun.
  // NOTE: we don't store the id of the sun geometry because we have no need
  // for subsequent access (the same is also true for dynamic geometries).
  GeometryId id = scene_graph->RegisterAnchoredGeometry(
      source_id_, make_unique<GeometryInstance>(
                      Isometry3<double>::Identity(), make_unique<Sphere>(1.f),
                      "sun"));
  scene_graph->AssignRole(source_id_, id,
                          MakeDrakeVisualizerProperties(Vector4d(1, 1, 0, 1)));

  // The fixed post on which Sun sits and around which all planets rotate.
  const double post_height = 1;
  Isometry3<double> post_pose = Isometry3<double>::Identity();
  post_pose.translation() << 0, 0, (orrery_bottom + post_height / 2);
  id = scene_graph->RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(
          post_pose, make_unique<Cylinder>(pipe_radius, post_height),
          "post"));
  scene_graph->AssignRole(source_id_, id, post_material);

  // Allocate the "celestial bodies": two planets orbiting on different planes,
  // each with a moon.

  // For the full description of the frame labels, see solar_system.h.

  // Earth's orbital frame Oe lies directly *below* the sun (to account for the
  // orrery arm).
  const double kEarthBottom = orrery_bottom + 0.25;
  Isometry3<double> X_SOe{Translation3<double>{0, 0, kEarthBottom}};
  FrameId planet_id = scene_graph->RegisterFrame(
      source_id_, GeometryFrame("EarthOrbit", X_SOe));
  body_ids_.push_back(planet_id);
  body_offset_.push_back(X_SOe);
  axes_.push_back(Vector3<double>::UnitZ());

  // The geometry is rigidly affixed to Earth's orbital frame so that it moves
  // in a circular path.
  const double kEarthOrbitRadius = 3.0;
  Isometry3<double> X_OeE{
      Translation3<double>{kEarthOrbitRadius, 0, -kEarthBottom}};
  id = scene_graph->RegisterGeometry(
      source_id_, planet_id,
      make_unique<GeometryInstance>(X_OeE, make_unique<Sphere>(0.25f),
                                    "Earth"));
  scene_graph->AssignRole(source_id_, id,
                          MakeDrakeVisualizerProperties(Vector4d(0, 0, 1, 1)));
  // Earth's orrery arm.
  MakeArm(source_id_, planet_id, kEarthOrbitRadius, -kEarthBottom, pipe_radius,
          post_material, scene_graph);

  // Luna's orbital frame Ol is at the center of Earth's geometry (E).
  // So, X_OeOl = X_OeE.
  const Isometry3<double>& X_OeOl = X_OeE;
  FrameId luna_id = scene_graph->RegisterFrame(
      source_id_, planet_id, GeometryFrame("LunaOrbit", X_OeOl));
  body_ids_.push_back(luna_id);
  body_offset_.push_back(X_OeOl);
  const Vector3<double> luna_axis_Oe{1, 1, 1};
  axes_.push_back(luna_axis_Oe.normalized());

  // The geometry is rigidly affixed to Luna's orbital frame so that it moves
  // in a circular path.
  const double kLunaOrbitRadius = 0.35;
  // Pick a position at kLunaOrbitRadius distance from the Earth's origin on
  // the plane _perpendicular_ to the moon's normal (<1, 1, 1>).
  // luna_position.dot(luna_axis_Oe) will be zero.
  Vector3<double> luna_position =
      Vector3<double>(-1, 0.5, 0.5).normalized() * kLunaOrbitRadius;
  Isometry3<double> X_OlL{Translation3<double>{luna_position}};
  id = scene_graph->RegisterGeometry(
      source_id_, luna_id, make_unique<GeometryInstance>(
          X_OlL, make_unique<Sphere>(0.075f), "Luna"));
  scene_graph->AssignRole(
      source_id_, id,
      MakeDrakeVisualizerProperties(Vector4d(0.5, 0.5, 0.35, 1)));

  // Convex satellite orbits Earth in the same revolution as Luna but with
  // different initial position. See SetDefaultState().
  FrameId convexsat_id = scene_graph->RegisterFrame(source_id_, planet_id,
                                          GeometryFrame("Convexsat", X_OeOl));
  body_ids_.push_back(convexsat_id);
  body_offset_.push_back(X_OeOl);
  axes_.push_back(luna_axis_Oe.normalized());

  std::string convexsat_absolute_path =
      FindResourceOrThrow("drake/examples/scene_graph/cuboctahedron.obj");
  id = scene_graph->RegisterGeometry(
      source_id_, convexsat_id,
      make_unique<GeometryInstance>(
          X_OlL, make_unique<Convex>(convexsat_absolute_path, 0.075),
          "convexsat"));
  scene_graph->AssignRole(source_id_, id,
                          MakeDrakeVisualizerProperties(Vector4d(1, 1, 0, 1)));

  // Box satellite orbits Earth in the same revolution as Luna but with
  // different initial position. See SetDefaultState().
  FrameId boxsat_id = scene_graph->RegisterFrame(
      source_id_, planet_id, GeometryFrame("Boxsat", X_OeOl));
  body_ids_.push_back(boxsat_id);
  body_offset_.push_back(X_OeOl);
  axes_.push_back(luna_axis_Oe.normalized());

  id = scene_graph->RegisterGeometry(
      source_id_, boxsat_id,
      make_unique<GeometryInstance>(
          X_OlL, make_unique<Box>(0.15, 0.15, 0.15), "boxsat"));
  scene_graph->AssignRole(source_id_, id,
                          MakeDrakeVisualizerProperties(Vector4d(1, 0, 1, 1)));

  // Mars's orbital frame Om lies directly *below* the sun (to account for the
  // orrery arm).
  Isometry3<double> X_SOm{Translation3<double>{0, 0, orrery_bottom}};
  planet_id =
      scene_graph->RegisterFrame(source_id_, GeometryFrame("MarsOrbit", X_SOm));
  body_ids_.push_back(planet_id);
  body_offset_.push_back(X_SOm);
  Vector3<double>  mars_axis_S {0, 0.1, 1};
  axes_.push_back(mars_axis_S.normalized());

  // The geometry is rigidly affixed to Mars's orbital frame so that it moves
  // in a circular path.
  const double kMarsOrbitRadius = 5.0;
  const double kMarsSize = 0.24;
  Isometry3<double> X_OmM{
      Translation3<double>{kMarsOrbitRadius, 0, -orrery_bottom}};
  GeometryId mars_geometry_id = scene_graph->RegisterGeometry(
      source_id_, planet_id,
      make_unique<GeometryInstance>(X_OmM, make_unique<Sphere>(kMarsSize),
                                    "Mars"));
  scene_graph->AssignRole(
      source_id_, mars_geometry_id,
      MakeDrakeVisualizerProperties(Vector4d(0.9, 0.1, 0, 1)));

  std::string rings_absolute_path =
      FindResourceOrThrow("drake/examples/scene_graph/planet_rings.obj");
  Vector3<double> axis = Vector3<double>(1, 1, 1).normalized();
  Isometry3<double> X_MR(AngleAxis<double>(M_PI / 3, axis));
  id = scene_graph->RegisterGeometry(
      source_id_, mars_geometry_id,
      make_unique<GeometryInstance>(
          X_MR, make_unique<Mesh>(rings_absolute_path, kMarsSize),
          "MarsRings"));
  scene_graph->AssignRole(
      source_id_, id, MakeDrakeVisualizerProperties(Vector4d(0.45, 0.9, 0, 1)));

  // Mars's orrery arm.
  MakeArm(source_id_, planet_id, kMarsOrbitRadius, -orrery_bottom, pipe_radius,
          post_material, scene_graph);

  // Phobos's orbital frame Op is at the center of Mars (M).
  // So, X_OmOp = X_OmM. The normal of the plane is negated so it orbits in the
  // opposite direction.
  const Isometry3<double>& X_OmOp = X_OmM;
  FrameId phobos_id = scene_graph->RegisterFrame(
      source_id_, planet_id, GeometryFrame("PhobosOrbit", X_OmOp));
  body_ids_.push_back(phobos_id);
  body_offset_.push_back(X_OmOp);
  mars_axis_S << 0, 0, -1;
  axes_.push_back(mars_axis_S.normalized());

  // The geometry is displaced from the Phobos's frame so that it orbits.
  const double kPhobosOrbitRadius = 0.34;
  Isometry3<double> X_OpP{Translation3<double>{kPhobosOrbitRadius, 0, 0}};
  id = scene_graph->RegisterGeometry(
      source_id_, phobos_id, make_unique<GeometryInstance>(
                                 X_OpP, make_unique<Sphere>(0.06f), "Phobos"));
  scene_graph->AssignRole(
      source_id_, id,
      MakeDrakeVisualizerProperties(Vector4d(0.65, 0.6, 0.8, 1)));

  DRAKE_DEMAND(static_cast<int>(body_ids_.size()) == kBodyCount);
}

template <typename T>
void SolarSystem<T>::CalcFramePoseOutput(const Context<T>& context,
                                         FramePoseVector<T>* poses) const {
  DRAKE_DEMAND(poses->size() == kBodyCount);
  DRAKE_DEMAND(poses->source_id() == source_id_);

  const BasicVector<T>& state = get_state(context);

  poses->clear();
  for (int i = 0; i < kBodyCount; ++i) {
    Isometry3<T> pose = body_offset_[i];
    // Frames only revolve around their origin; it is only necessary to set the
    // rotation value.
    T rotation{state[i]};
    pose.linear() = AngleAxis<T>(rotation, axes_[i]).matrix();
    poses->set_value(body_ids_[i], pose);
  }
}

template <typename T>
void SolarSystem<T>::DoCalcTimeDerivatives(
    const MyContext& context, MyContinuousState* derivatives) const {
  const BasicVector<T>& state = get_state(context);
  BasicVector<T>& derivative_vector = get_mutable_state(derivatives);
  derivative_vector.SetZero();
  derivative_vector.get_mutable_value().head(kBodyCount) =
      state.get_value().tail(kBodyCount);
}

template class SolarSystem<double>;

}  // namespace solar_system
}  // namespace examples
}  // namespace drake
