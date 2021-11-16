#include "drake/examples/pendulum/pendulum_geometry.h"

#include <memory>
#include <utility>

#include "drake/examples/pendulum/gen/pendulum_params.h"
#include "drake/examples/pendulum/gen/pendulum_state.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace pendulum {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::MakePhongIllustrationProperties;
using geometry::PerceptionProperties;
using geometry::render::RenderLabel;
using geometry::Rgba;
using geometry::Sphere;
using std::make_unique;

const PendulumGeometry* PendulumGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& pendulum_state_port,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto pendulum_geometry = builder->AddSystem(
      std::unique_ptr<PendulumGeometry>(
          new PendulumGeometry(scene_graph)));
  builder->Connect(
      pendulum_state_port,
      pendulum_geometry->get_input_port(0));
  builder->Connect(
      pendulum_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(pendulum_geometry->source_id_));

  return pendulum_geometry;
}

PendulumGeometry::PendulumGeometry(geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource();
  frame_id_ = scene_graph->RegisterFrame(source_id_, GeometryFrame("arm"));

  this->DeclareVectorInputPort("state", PendulumState<double>());
  this->DeclareAbstractOutputPort(
      "geometry_pose", &PendulumGeometry::OutputGeometryPose);

  // TODO(jwnimmer-tri) This registration fails to reflect any non-default
  // parameters.  Ideally, it should happen in an Initialize event that
  // modifies the Context, or the output port should express the geometries
  // themselves instead of just their poses, or etc.
  const PendulumParams<double> params;
  const double length = params.length();
  const double mass = params.mass();

  // The base.
  {
    GeometryId id = scene_graph->RegisterAnchoredGeometry(
        source_id_, make_unique<GeometryInstance>(
                        math::RigidTransformd(Vector3d(0., 0., .025)),
                        make_unique<Box>(.05, 0.05, 0.05), "base"));

    scene_graph->AssignRole(
        source_id_, id,
        MakePhongIllustrationProperties(Vector4d(.3, .6, .4, 1)));
    PerceptionProperties perception;
    perception.AddProperty("phong", "diffuse", Rgba{0.3, 0.6, 0.4, 1.0});
    perception.AddProperty("label", "id", RenderLabel::kDontCare);
    scene_graph->AssignRole(source_id_, id, perception);
  }

  // The arm.
  {
    GeometryId id = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        make_unique<GeometryInstance>(
            math::RigidTransformd(Vector3d(0., 0., -length / 2.)),
            make_unique<Cylinder>(0.01, length), "arm"));
    scene_graph->AssignRole(
        source_id_, id,
        MakePhongIllustrationProperties(Vector4d(.9, .1, 0, 1)));
    PerceptionProperties perception;
    perception.AddProperty("phong", "diffuse", Rgba{0.9, 0.1, 0, 1.0});
    perception.AddProperty("label", "id", RenderLabel::kDontCare);
    scene_graph->AssignRole(source_id_, id, perception);
  }

  // The mass at the end of the arm.
  {
    GeometryId id = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        make_unique<GeometryInstance>(
            math::RigidTransformd(Vector3d(0., 0., -length)),
            make_unique<Sphere>(mass / 40.), "arm point mass"));
    scene_graph->AssignRole(
        source_id_, id, MakePhongIllustrationProperties(Vector4d(0, 0, 1, 1)));
    PerceptionProperties perception;
    perception.AddProperty("phong", "diffuse", Rgba{0, 0, 1, 1});
    perception.AddProperty("label", "id", RenderLabel::kDontCare);
    scene_graph->AssignRole(source_id_, id, perception);
  }
}

PendulumGeometry::~PendulumGeometry() = default;

void PendulumGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(frame_id_.is_valid());

  const auto& input = get_input_port(0).Eval<PendulumState<double>>(context);
  const double theta = input.theta();
  const math::RigidTransformd pose(math::RotationMatrixd::MakeYRotation(theta));

  *poses = {{frame_id_, pose}};
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
