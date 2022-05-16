#include "drake/examples/compass_gait/compass_gait_geometry.h"

#include <memory>

#include <fmt/format.h>

#include "drake/examples/compass_gait/gen/compass_gait_params.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace compass_gait {

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

const CompassGaitGeometry* CompassGaitGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& floating_base_state_port,
    const CompassGaitParams<double>& compass_gait_params,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto compass_gait_geometry =
      builder->AddSystem(std::unique_ptr<CompassGaitGeometry>(
          new CompassGaitGeometry(compass_gait_params, scene_graph)));
  builder->Connect(floating_base_state_port,
                   compass_gait_geometry->get_input_port(0));
  builder->Connect(
      compass_gait_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(compass_gait_geometry->source_id_));

  return compass_gait_geometry;
}

CompassGaitGeometry::CompassGaitGeometry(
    const CompassGaitParams<double>& params,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource();

  // Note: the floating-base state output from CompassGait uses RPY (12
  // dims) + 2 for the position/velocity of the right leg.
  this->DeclareVectorInputPort("floating_base_state", 14);
  this->DeclareAbstractOutputPort("geometry_pose",
                                  &CompassGaitGeometry::OutputGeometryPose);

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

  left_leg_frame_id_ =
      scene_graph->RegisterFrame(source_id_, GeometryFrame("left_leg"));
  right_leg_frame_id_ =
      scene_graph->RegisterFrame(source_id_, left_leg_frame_id_,
          GeometryFrame("right_leg"));

  // Add the hip.  Both legs have the same origin of rotation and the hip
  // geometry is rotationally symmetric, so we opt to arbitrarily attach it
  // to the left leg.
  const double hip_mass_radius = .1;
  id = scene_graph->RegisterGeometry(
      source_id_, left_leg_frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(math::RotationMatrixd::MakeXRotation(M_PI_2)),
          make_unique<Sphere>(hip_mass_radius), "hip"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(0, 1, 0, 1)));

  // Scale the leg mass geometry relative to the hip mass (assuming constant
  // density).
  const double leg_mass_radius = std::cbrt(
      std::pow(hip_mass_radius, 3) * params.mass_leg() / params.mass_hip());

  // Add the left leg (which is attached to the hip).
  id = scene_graph->RegisterGeometry(
      source_id_, left_leg_frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(Vector3d(0, 0, -params.length_leg() / 2.)),
          make_unique<Cylinder>(0.0075, params.length_leg()), "left_leg"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(1, 0, 0, 1)));
  id = scene_graph->RegisterGeometry(
      source_id_, left_leg_frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(Vector3d(0, 0, -params.center_of_mass_leg())),
          make_unique<Sphere>(leg_mass_radius), "left_leg_mass"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(1, 0, 0, 1)));

  // Add the right leg.
  id = scene_graph->RegisterGeometry(
      source_id_, right_leg_frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(Vector3d(0, 0, -params.length_leg() / 2.)),
          make_unique<Cylinder>(0.0075, params.length_leg()), "right_leg"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(0, 0, 1, 1)));
  id = scene_graph->RegisterGeometry(
      source_id_, right_leg_frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(Vector3d(0, 0, -params.center_of_mass_leg())),
          make_unique<Sphere>(leg_mass_radius), "right_leg_mass"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(0, 0, 1, 1)));
}

CompassGaitGeometry::~CompassGaitGeometry() = default;

void CompassGaitGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(left_leg_frame_id_.is_valid());
  DRAKE_DEMAND(right_leg_frame_id_.is_valid());

  const VectorXd& input = get_input_port(0).Eval(context);
  const math::RigidTransformd left_pose(
      math::RollPitchYawd(input.segment<3>(3)), input.head<3>());
  const double hip_angle = input[6];
  const math::RigidTransformd right_pose(
      math::RotationMatrixd::MakeYRotation(hip_angle));

  *poses = {{left_leg_frame_id_, left_pose}, {right_leg_frame_id_, right_pose}};
}

}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
