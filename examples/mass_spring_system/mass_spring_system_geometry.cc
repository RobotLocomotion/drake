#include "drake/examples/mass_spring_system/mass_spring_system_geometry.h"

#include <memory>
#include <utility>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace examples {
namespace mass_spring_system {

const MassSpringSystemGeometry* MassSpringSystemGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& mass_spring_system_state_port,
    geometry::SceneGraph<double>* scene_graph, int num_points) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto mass_spring_system_geometry =
      builder->AddSystem(std::unique_ptr<MassSpringSystemGeometry>(
          new MassSpringSystemGeometry(scene_graph, num_points)));
  builder->Connect(mass_spring_system_state_port,
                   mass_spring_system_geometry->get_input_port(0));
  builder->Connect(mass_spring_system_geometry->get_output_port(0),
                   scene_graph->get_source_pose_port(
                       mass_spring_system_geometry->source_id_));

  return mass_spring_system_geometry;
}

MassSpringSystemGeometry::MassSpringSystemGeometry(
    geometry::SceneGraph<double>* scene_graph, int num_points)
    : num_points_(num_points) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource();

  this->DeclareInputPort("state", systems::kVectorValued, 2 * num_points * 3);
  this->DeclareAbstractOutputPort(
      "geometry_pose", &MassSpringSystemGeometry::OutputGeometryPose);

  frame_ids_.resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    frame_ids_[i] = scene_graph->RegisterFrame(
        source_id_, geometry::GeometryFrame("mass_points" + std::to_string(i)));
    // Attach a sphere to the frame for simple visualization of the mass points.
    const geometry::GeometryId id = scene_graph->RegisterGeometry(
        source_id_, frame_ids_[i],
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd::Identity(),
            std::make_unique<geometry::Sphere>(0.04), "sphere_visual"));
    scene_graph->AssignRole(
        source_id_, id,
        geometry::MakePhongIllustrationProperties(Eigen::Vector4d(1, 0, 1, 1)));
  }
}

void MassSpringSystemGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  for (int i = 0; i < num_points_; ++i) {
    DRAKE_DEMAND(frame_ids_[i].is_valid());
  }
  const auto& input = get_input_port(0).Eval(context);
  poses->clear();
  // Set the frames to the positions of the points.
  for (int i = 0; i < static_cast<int>(frame_ids_.size()); ++i) {
    const double x = input(3 * i);
    const double y = input(3 * i + 1);
    const double z = input(3 * i + 2);
    const math::RigidTransformd pose(Eigen::Vector3d{x, y, z});
    poses->set_value(frame_ids_[i], pose);
  }
}

}  // namespace mass_spring_system
}  // namespace examples
}  // namespace drake
