#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

std::unique_ptr<RigidBodyTree<double>> build_tree(std::vector<ModelInstanceInfo<double>>* iiwa) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf");

  iiwa->clear();

  int id = tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(0, 0, 0));
  iiwa->push_back(tree_builder->get_model_info_for_instance(id));

  id = tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(1, 0, 0));
  iiwa->push_back(tree_builder->get_model_info_for_instance(id));

  return tree_builder->Build();
}

void main() {
  drake::lcm::DrakeLcm lcm;
  std::vector<ModelInstanceInfo<double>> iiwa_info;
  SimDiagramBuilder<double> builder;

  {
    builder.AddPlant(build_tree(&iiwa_info));

    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

    for (const auto& info : iiwa_info) {
      std::unique_ptr<systems::StateFeedbackController<double>> controller =
          std::make_unique<systems::InverseDynamicsController<double>>(info.model_path, info.world_offset, iiwa_kp, iiwa_ki, iiwa_kd, true /* with feedforward acceleration */);
      builder.AddController(info.instance_id, std::move(controller));
    }
  }

  // Connects desired trajs to the controllers.
  std::vector<double> times = {0, 2, 4};
  std::vector<MatrixX<double>> knots(times.size(), MatrixX<double>::Zero(7, 1));
  knots[1] << M_PI, 0, 0, M_PI / 2., 0, 0, 0;
  PiecewisePolynomial<double> poly = PiecewisePolynomial<double>::Cubic(times, knots, MatrixX<double>::Zero(7, 1), MatrixX<double>::Zero(7, 1));
  PiecewisePolynomial<double> polyd = poly.derivative();
  PiecewisePolynomial<double> polydd = polyd.derivative();
  PiecewisePolynomialTrajectory poly_traj(poly);
  PiecewisePolynomialTrajectory polyd_traj(polyd);
  PiecewisePolynomialTrajectory polydd_traj(polydd);

  systems::TrajectorySource<double>* traj = builder.template AddSystem<systems::TrajectorySource<double>>(poly_traj);
  systems::TrajectorySource<double>* trajd = builder.template AddSystem<systems::TrajectorySource<double>>(polyd_traj);
  systems::TrajectorySource<double>* trajdd = builder.template AddSystem<systems::TrajectorySource<double>>(polydd_traj);

  systems::Multiplexer<double>* input_mux =
      builder.template AddSystem<systems::Multiplexer<double>>(std::vector<int>{7, 7});

  builder.Connect(traj->get_output_port(), input_mux->get_input_port(0));
  builder.Connect(trajd->get_output_port(), input_mux->get_input_port(1));

  for (const auto& info : iiwa_info) {
    systems::InverseDynamicsController<double>* controller =
        dynamic_cast<systems::InverseDynamicsController<double>*>(builder.get_controller(info.instance_id));
    builder.Connect(input_mux->get_output_port(0), controller->get_input_port_desired_state());
    builder.Connect(trajdd->get_output_port(), controller->get_input_port_desired_acceleration());
  }

  {
    // Connects visualizer.
    systems::DrakeVisualizer* viz_publisher = builder.template AddSystem<systems::DrakeVisualizer>(
        builder.get_plant()->get_rigid_body_tree(), &lcm);
    builder.Connect(builder.get_plant()->get_output_port(0),
                    viz_publisher->get_input_port(0));
  }

  // Makes a simulation
  builder.WireThingsTogether();
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

