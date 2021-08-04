#include "drake/examples/acrobot/acrobot_geometry.h"

#include <memory>

#include "drake/examples/acrobot/gen/acrobot_params.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace acrobot {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::MakePhongIllustrationProperties;
using geometry::Sphere;
using std::make_unique;

const AcrobotGeometry* AcrobotGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& acrobot_state_port,
    const AcrobotParams<double>& acrobot_params,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto acrobot_geometry = builder->AddSystem(std::unique_ptr<AcrobotGeometry>(
      new AcrobotGeometry(acrobot_params, scene_graph)));
  builder->Connect(acrobot_state_port, acrobot_geometry->get_input_port(0));
  builder->Connect(
      acrobot_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(acrobot_geometry->source_id_));

  return acrobot_geometry;
}

AcrobotGeometry::AcrobotGeometry(const AcrobotParams<double>& params,
                                 geometry::SceneGraph<double>* scene_graph)
    : l1_(params.l1()) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource();

  // Note: using AcrobotState as the port type would have complicated the
  // trajectory playback workflow used in run_swing_up_traj_optimization.cc.
  this->DeclareVectorInputPort("state", 4);
  this->DeclareAbstractOutputPort("geometry_pose",
                                  &AcrobotGeometry::OutputGeometryPose);

  // The base.
  GeometryId id = scene_graph->RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(math::RigidTransformd::Identity(),
                                    make_unique<Box>(.2, 0.2, 0.2), "base"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(0, 1, 0, 1)));

  // The upper link.
  upper_link_frame_id_ =
      scene_graph->RegisterFrame(source_id_, GeometryFrame("upper_link"));
  id = scene_graph->RegisterGeometry(
      source_id_, upper_link_frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(Vector3d(0., 0.15, -params.l1() / 2.)),
          make_unique<Cylinder>(0.05, params.l1()), "upper_link"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(1, 0, 0, 1)));

  // The lower link.
  lower_link_frame_id_ =
      scene_graph->RegisterFrame(source_id_, GeometryFrame("lower_link"));
  id = scene_graph->RegisterGeometry(
      source_id_, lower_link_frame_id_,
      make_unique<GeometryInstance>(
          math::RigidTransformd(Vector3d(0., 0.25, -params.l2() / 2.)),
          make_unique<Cylinder>(0.05, params.l2()), "lower_link"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(0, 0, 1, 1)));
}

AcrobotGeometry::~AcrobotGeometry() = default;

void AcrobotGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(upper_link_frame_id_.is_valid());
  DRAKE_DEMAND(lower_link_frame_id_.is_valid());

  // TODO(russt): Use AcrobotState here upon resolution of #12566.
  const auto& input =
      get_input_port(0).Eval<systems::BasicVector<double>>(context);
  const double theta1 = input[0], theta2 = input[1];
  const math::RigidTransformd upper_link_pose(
      math::RotationMatrixd::MakeYRotation(theta1));
  const math::RigidTransformd lower_link_pose(
      math::RotationMatrixd::MakeYRotation(theta1 + theta2),
      Vector3d(-l1_ * std::sin(theta1), 0, -l1_ * std::cos(theta1)));

  *poses = {{upper_link_frame_id_, upper_link_pose},
            {lower_link_frame_id_, lower_link_pose}};
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
