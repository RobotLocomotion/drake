#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_building_util.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

std::unique_ptr<RigidBodyTree<double>> build_tree(
    std::vector<ModelInstanceInfo<double>>* iiwa) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel(
      "iiwa",
      "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf");

  iiwa->clear();

  int id =
      tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(0, 0, 0));
  iiwa->push_back(tree_builder->get_model_info_for_instance(id));

  id = tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(1, 0, 0));
  iiwa->push_back(tree_builder->get_model_info_for_instance(id));

  return tree_builder->Build();
}

void main() {
  drake::lcm::DrakeLcm lcm;
  std::vector<ModelInstanceInfo<double>> iiwa_info;
  SimDiagramBuilder<double> builder;

  builder.AddPlant(build_tree(&iiwa_info));
  builder.AddVisualizer(&lcm);

  // Generates a desired plan.
  std::vector<double> times = {0, 2, 4};
  std::vector<MatrixX<double>> knots(times.size(), MatrixX<double>::Zero(7, 1));
  knots[1] << M_PI, 0, 0, M_PI / 2., 0, 0, 0;
  PiecewisePolynomial<double> poly = PiecewisePolynomial<double>::Cubic(
      times, knots, MatrixX<double>::Zero(7, 1), MatrixX<double>::Zero(7, 1));
  std::unique_ptr<PiecewisePolynomialTrajectory> traj = std::make_unique<PiecewisePolynomialTrajectory>(poly);

  // Adds a trajectory source for desired state and accelerations.
  systems::DiagramBuilder<double>* diagram_builder = builder.get_mutable_builder();
  auto traj_src = diagram_builder->template AddSystem<DesiredTrajectorySource<double>>(std::move(traj), 2);

  // Adds controllers for all the iiwa arms.
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  for (const auto& info : iiwa_info) {
    auto controller = builder.AddController<systems::InverseDynamicsController<double>>(
        info.instance_id,
        info.model_path, info.world_offset, iiwa_kp, iiwa_ki, iiwa_kd,
        true /* with feedforward acceleration */);
    diagram_builder->Connect(traj_src->get_output_port_state(),
                             controller->get_input_port_desired_state());
    diagram_builder->Connect(traj_src->get_output_port_acceleration(),
                             controller->get_input_port_desired_acceleration());
  }

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(5);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::kuka_iiwa_arm::main();

  return 0;
}
