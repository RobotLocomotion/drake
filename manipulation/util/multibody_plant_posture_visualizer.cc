#include "drake/manipulation/util/multibody_plant_posture_visualizer.h"

#include <utility>

#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace drake {
namespace manipulation {
MultibodyPlantPostureVisualizer::MultibodyPlantPostureVisualizer(
    const multibody::MultibodyPlant<double>& plant,
    std::unique_ptr<geometry::SceneGraph<double>> scene_graph) {
  systems::DiagramBuilder<double> builder;
  auto scene_graph_ptr = builder.AddSystem(std::move(scene_graph));
  posture_source_ = builder.AddSystem<systems::ConstantVectorSource>(
      Eigen::VectorXd::Zero(plant.num_positions()));
  auto to_pose = builder.AddSystem<
      systems::rendering::MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(posture_source_->get_output_port(),
                  to_pose->get_input_port());
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph_ptr->get_source_pose_port(plant.get_source_id().value()));

  geometry::ConnectDrakeVisualizer(&builder, *scene_graph_ptr);
  diagram_ = builder.Build();

  simulator_ = std::make_unique<systems::Simulator<double>>(*diagram_);
  simulator_->set_publish_every_time_step(false);
}

void MultibodyPlantPostureVisualizer::VisualizePosture(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
  posture_source_
      ->get_mutable_source_value(&(diagram_->GetMutableSubsystemContext(
          *posture_source_, &(simulator_->get_mutable_context()))))
      .set_value(q);
  simulator_->StepTo(0);
}
}  // namespace manipulation
}  // namespace drake
