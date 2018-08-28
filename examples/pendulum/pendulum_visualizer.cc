#include "drake/examples/pendulum/pendulum_visualizer.h"

#include <cmath>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace pendulum {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::Sphere;
using geometry::VisualMaterial;
using std::make_unique;

PendulumVisualizer::PendulumVisualizer(
    const PendulumParams<double>& params,
    geometry::SceneGraph<double>* scene_graph)
    : systems::LeafSystem<double>() {
  this->DeclareVectorInputPort(PendulumState<double>());


}

const systems::InputPort<double>& PendulumVisualizer::get_state_input_port()
    const {
  return systems::System<double>::get_input_port(0);
}

const systems::OutputPort<double>&
PendulumVisualizer::get_geometry_pose_output_port() const {
  return systems::System<double>::get_output_port(0);
}

void PendulumVisualizer::CopyPoseOut(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(poses->size() == 1);
  DRAKE_DEMAND(poses->source_id() == source_id_);

  const double theta =
      this->EvalVectorInput<PendulumState>(context, 0)->theta();

  poses->clear();
  Isometry3<double> pose = Isometry3<double>::Identity();
  pose.linear() = math::RotationMatrix<double>::MakeYRotation(theta).matrix();
  poses->set_value(pose_id_, pose);
}

PendulumVisualizer* AddPendulumVisualizerAndPublisher(
    const PendulumParams<double>& params,
    systems::DiagramBuilder<double>* builder, lcm::DrakeLcmInterface* lcm) {
  auto scene_graph = builder->AddSystem<geometry::SceneGraph<double>>();
  scene_graph->set_name("scene-graph");

  PendulumVisualizer* visualizer =
      builder->AddSystem<PendulumVisualizer>(params, scene_graph);
  visualizer->set_name("pendulum_visualizer");

  builder->Connect(visualizer->get_geometry_pose_output_port(),
                   scene_graph->get_source_pose_port(visualizer->source_id()));

  geometry::ConnectVisualization(*scene_graph, builder, lcm);
  geometry::DispatchLoadMessage(*scene_graph, lcm);

  return visualizer;
}

PendulumVisualizer* AddPendulumVisualizerAndPublisher(
    systems::DiagramBuilder<double>* builder, lcm::DrakeLcmInterface* lcm) {
  return AddPendulumVisualizerAndPublisher(PendulumParams<double>(), builder,
                                           lcm);
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
