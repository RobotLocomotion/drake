#include "drake/examples/rod2d/rod2d_geometry.h"

#include <memory>

#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/rgba.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace rod2d {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

const Rod2dGeometry* Rod2dGeometry::AddToBuilder(
    double radius, double length, systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& rod2d_state_port,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto rod2d_geometry = builder->AddSystem(std::unique_ptr<Rod2dGeometry>(
      new Rod2dGeometry(radius, length, scene_graph)));
  builder->Connect(rod2d_state_port, rod2d_geometry->get_input_port());
  builder->Connect(
      rod2d_geometry->get_output_port(),
      scene_graph->get_source_pose_port(rod2d_geometry->source_id_));

  return rod2d_geometry;
}

Rod2dGeometry::Rod2dGeometry(double radius, double length,
                             geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  source_id_ = scene_graph->RegisterSource("rod2d");
  frame_id_ =
      scene_graph->RegisterFrame(source_id_, geometry::GeometryFrame("rod2d"));
  geometry::GeometryId geometry = scene_graph->RegisterGeometry(
      source_id_, frame_id_,
      std::make_unique<geometry::GeometryInstance>(
          RigidTransformd{},
          std::make_unique<geometry::Cylinder>(radius, length), "rod2d"));
  geometry::IllustrationProperties illus_props;
  illus_props.AddProperty("phong", "diffuse", geometry::Rgba(0.7, 0.7, 0.7, 1));
  scene_graph->AssignRole(source_id_, geometry, illus_props);

  // Only uses the qs of the state (x, y, θ).
  this->DeclareVectorInputPort("state", 6);
  this->DeclareAbstractOutputPort("geometry_pose",
                                  &Rod2dGeometry::OutputGeometryPose);
}

Rod2dGeometry::~Rod2dGeometry() = default;

void Rod2dGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(frame_id_.is_valid());

  const auto& state = get_input_port().Eval(context);
  // Converts the configuration of the rod to a pose in ℜ³. The 2D system lies
  // on the x-y plane with +y up. In 3D, we place it on the y = 0 plane with +z
  // up. This is compatible with the fact that a drake cylinder has its axis
  // aligned with the z-axis.
  math::RigidTransformd pose(RotationMatrixd::MakeYRotation(state(2) + M_PI_2),
                             Vector3d(state(0), 0, state(1)));

  *poses = {{frame_id_, pose}};
}

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
