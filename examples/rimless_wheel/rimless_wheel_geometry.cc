#include "drake/examples/rimless_wheel/rimless_wheel_geometry.h"

#include <memory>

#include <fmt/format.h>

#include "drake/examples/rimless_wheel/gen/rimless_wheel_params.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::MakePhongIllustrationProperties;
using geometry::Sphere;
using std::make_unique;

const RimlessWheelGeometry* RimlessWheelGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& floating_base_state_port,
    const RimlessWheelParams<double>& rimless_wheel_params,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto rimless_wheel_geometry =
      builder->AddSystem(std::unique_ptr<RimlessWheelGeometry>(
          new RimlessWheelGeometry(rimless_wheel_params, scene_graph)));
  builder->Connect(floating_base_state_port,
                   rimless_wheel_geometry->get_input_port(0));
  builder->Connect(
      rimless_wheel_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(rimless_wheel_geometry->source_id_));

  return rimless_wheel_geometry;
}

RimlessWheelGeometry::RimlessWheelGeometry(
    const RimlessWheelParams<double>& params,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource();

  // Note: the floating-base state output from RimlessWheel uses RPY.
  this->DeclareVectorInputPort("floating_base_state", 12);
  this->DeclareAbstractOutputPort("geometry_pose",
                                  &RimlessWheelGeometry::OutputGeometryPose);

  // Add the ramp.
  // Positioned so that the top center of the box is at x=y=z=0.
  GeometryId id = scene_graph->RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(
              math::RotationMatrixd::MakeYRotation(params.slope())) *
              math::RigidTransformd(Vector3d(0, 0, -10. / 2.)),
          make_unique<Box>(100, 1, 10), "ramp"));
  // Color is "desert sand" according to htmlcsscolor.com.
  scene_graph->AssignRole(
      source_id_, id,
      MakePhongIllustrationProperties(Vector4d(0.9297, 0.7930, 0.6758, 1)));

  // All of the remaining geometry is on one link.
  frame_id_ = scene_graph->RegisterFrame(source_id_, GeometryFrame("center"));

  // Add the hub.
  id = scene_graph->RegisterGeometry(
      source_id_, frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(math::RotationMatrixd::MakeXRotation(M_PI_2)),
          make_unique<Cylinder>(.2, .2), "hub"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.6, .2, .2, 1)));

  // Add the spokes.
  for (int i = 0; i < params.number_of_spokes(); i++) {
    id = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        make_unique<GeometryInstance>(
            math::RigidTransformd(math::RotationMatrixd::MakeYRotation(
                (2. * M_PI * i) / params.number_of_spokes())) *
                math::RigidTransformd(Vector3d(0, 0, -params.length() / 2.)),
            make_unique<Cylinder>(0.0075, params.length()),
            fmt::format("spoke{}", i)));
    scene_graph->AssignRole(
        source_id_, id, MakePhongIllustrationProperties(Vector4d(0, 0, 0, 1)));
  }
}

RimlessWheelGeometry::~RimlessWheelGeometry() = default;

void RimlessWheelGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(frame_id_.is_valid());

  const VectorXd& input = get_input_port(0).Eval(context);
  const math::RigidTransformd pose(math::RollPitchYawd(input.segment<3>(3)),
                                   input.head<3>());

  *poses = {{frame_id_, pose}};
}

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
