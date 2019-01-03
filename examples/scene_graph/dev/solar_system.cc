#include "drake/examples/scene_graph/dev/solar_system.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_values.h"

namespace drake {
namespace examples {
namespace solar_system {

using Eigen::Vector4d;
using geometry::Cylinder;
using geometry::dev::SceneGraph;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Mesh;
using geometry::SourceId;
using geometry::Sphere;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::DiscreteValues;
using std::make_unique;
using std::unique_ptr;

template <typename Shape, typename... ShapeArgs>
unique_ptr<GeometryInstance> MakeShape(const Isometry3<double>& pose,
                            const std::string& name,
                            const Vector4<double>& diffuse,
                            ShapeArgs&&... args) {
  auto instance = make_unique<GeometryInstance>(
      pose, make_unique<Shape>(std::forward<ShapeArgs>(args)...),
      name);
  IllustrationProperties properties;
  properties.AddProperty("phong", "diffuse", diffuse);
  instance->set_illustration_properties(properties);
  return instance;
}

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
                   M_PI / 2,        // Mars initial position
                   0,               // phobos initial position
                   2 * M_PI / 5,    // Earth revolution lasts 5 seconds.
                   2 * M_PI,        // moon revolution lasts 1 second.
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
             double height, double radius, const Vector4d& material,
             SceneGraph<double>* scene_graph) {
  Isometry3<double> arm_pose = Isometry3<double>::Identity();
  // tilt it horizontally
  arm_pose.linear() =
      Eigen::AngleAxis<double>(M_PI / 2, Vector3<double>::UnitY()).matrix();
  arm_pose.translation() << length / 2, 0, 0;
  scene_graph->RegisterGeometry(
      source_id, parent_id,
      MakeShape<Cylinder>(arm_pose, "horz_arm", material, radius, length));

  Isometry3<double> post_pose = Isometry3<double>::Identity();
  post_pose.translation() << length, 0, height / 2;
  scene_graph->RegisterGeometry(
      source_id, parent_id,
      MakeShape<Cylinder>(post_pose, "vert_arm", material, radius, height));
}

template <typename T>
void SolarSystem<T>::AllocateGeometry(SceneGraph<T>* scene_graph) {
  body_ids_.reserve(kBodyCount);
  body_offset_.reserve(kBodyCount);
  axes_.reserve(kBodyCount);

  Vector4d post_material(0.3, 0.15, 0.05, 1);
  const double orrery_bottom = -1.5;
  const double pipe_radius = 0.05;

  // Allocate the sun.
  // NOTE: we don't store the id of the sun geometry because we have no need
  // for subsequent access (the same is also true for dynamic geometries).
  scene_graph->RegisterAnchoredGeometry(
      source_id_, MakeShape<Sphere>(
          Isometry3<double>::Identity(), "sun", Vector4d(1, 1, 0, 1),
          1.f  /* radius */));

  // The fixed post on which Sun sits and around which all planets rotate.
  const double post_height = 1;
  Isometry3<double> post_pose = Isometry3<double>::Identity();
  post_pose.translation() << 0, 0, (orrery_bottom + post_height / 2);
  scene_graph->RegisterAnchoredGeometry(
      source_id_,
      MakeShape<Cylinder>(post_pose, "post", post_material, pipe_radius,
                          post_height));

  // Allocate the "celestial bodies": two planets orbiting on different planes,
  // each with a moon.

  // For the full description of the frame labels, see solar_system.h.

  // Earth's frame E lies directly *below* the sun (to account for the
  // orrery arm).
  const double kEarthBottom = orrery_bottom + 0.25;
  Isometry3<double> X_SE{Translation3<double>{0, 0, kEarthBottom}};
  FrameId planet_id =
      scene_graph->RegisterFrame(source_id_, GeometryFrame("Earth", X_SE));
  body_ids_.push_back(planet_id);
  body_offset_.push_back(X_SE);
  axes_.push_back(Vector3<double>::UnitZ());

  // The geometry is displaced from the Earth _frame_ so that it orbits.
  const double kEarthOrbitRadius = 3.0;
  Isometry3<double> X_EGe{
      Translation3<double>{kEarthOrbitRadius, 0, -kEarthBottom}};
  scene_graph->RegisterGeometry(
      source_id_, planet_id,
      MakeShape<Sphere>(X_EGe, "earth", Vector4d(0, 0, 1, 1), 0.25));
  // Earth's orrery arm.
  MakeArm(source_id_, planet_id, kEarthOrbitRadius, -kEarthBottom, pipe_radius,
          post_material, scene_graph);

  // Luna's frame L is at the center of Earth's geometry (Ge). So, X_EL = X_EGe.
  const Isometry3<double>& X_EL = X_EGe;
  FrameId luna_id = scene_graph->RegisterFrame(source_id_, planet_id,
                                               GeometryFrame("Luna", X_EL));
  body_ids_.push_back(luna_id);
  body_offset_.push_back(X_EL);
  Vector3<double> plane_normal{1, 1, 1};
  axes_.push_back(plane_normal.normalized());

  // The geometry is displaced from Luna's frame so that it orbits.
  const double kLunaOrbitRadius = 0.35;
  // Pick a position at kLunaOrbitRadius distance from the Earth's origin on
  // the plane _perpendicular_ to the moon's normal (<1, 1, 1>).
  // luna_position.dot(plane_normal) will be zero.
  Vector3<double> luna_position =
      Vector3<double>(-1, 0.5, 0.5).normalized() * kLunaOrbitRadius;
  Isometry3<double> X_LGl{Translation3<double>{luna_position}};
  scene_graph->RegisterGeometry(
      source_id_, luna_id,
      MakeShape<Sphere>(X_LGl, "luna", Vector4d(0.5, 0.5, 0.35, 0.1), 0.075));

  // Mars's frame M lies directly *below* the sun (to account for the orrery
  // arm).
  Isometry3<double> X_SM{Translation3<double>{0, 0, orrery_bottom}};
  planet_id =
      scene_graph->RegisterFrame(source_id_, GeometryFrame("Mars", X_SM));
  body_ids_.push_back(planet_id);
  body_offset_.push_back(X_SM);
  plane_normal << 0, 0.1, 1;
  axes_.push_back(plane_normal.normalized());

  // The geometry is displaced from the Mars _frame_ so that it orbits.
  const double kMarsOrbitRadius = 5.0;
  const double kMarsSize = 0.24;
  Isometry3<double> X_MGm{
      Translation3<double>{kMarsOrbitRadius, 0, -orrery_bottom}};
  GeometryId mars_geometry_id = scene_graph->RegisterGeometry(
      source_id_, planet_id,
      MakeShape<Sphere>(X_MGm, "mars", Vector4d(0.9, 0.1, 0, 1), kMarsSize));

  std::string rings_absolute_path =
      FindResourceOrThrow("drake/examples/scene_graph/planet_rings.obj");
  Vector3<double> axis = Vector3<double>(1, 1, 1).normalized();
  Isometry3<double> X_GmGr(AngleAxis<double>(M_PI / 3, axis));
  scene_graph->RegisterGeometry(
      source_id_, mars_geometry_id,
      MakeShape<Mesh>(X_GmGr, "rings", Vector4d(0.45, 0.9, 0, 1),
                      rings_absolute_path, kMarsSize));

  // Mars's orrery arm.
  MakeArm(source_id_, planet_id, kMarsOrbitRadius, -orrery_bottom, pipe_radius,
          post_material, scene_graph);

  // Phobos's frame P is at the center of Mars's geometry (Gm).
  // So, X_MP = X_MGm. The normal of the plane is negated so it orbits in the
  // opposite direction.
  const Isometry3<double>& X_MP = X_MGm;
  FrameId phobos_id = scene_graph->RegisterFrame(source_id_, planet_id,
                                                 GeometryFrame("phobos", X_MP));
  body_ids_.push_back(phobos_id);
  body_offset_.push_back(X_MP);
  plane_normal << 0, 0, -1;
  axes_.push_back(plane_normal.normalized());

  // The geometry is displaced from the Phobos's frame so that it orbits.
  const double kPhobosOrbitRadius = 0.34;
  Isometry3<double> X_PGp{Translation3<double>{kPhobosOrbitRadius, 0, 0}};
  scene_graph->RegisterGeometry(
      source_id_, phobos_id,
      MakeShape<Sphere>(X_PGp, "phobos", Vector4d(0.65, 0.6, 0.8, 1), 0.06));

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
