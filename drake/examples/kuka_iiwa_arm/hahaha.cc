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
  systems::Diagram<double>* sim;
  std::vector<ModelInstanceInfo<double>> iiwa_info;
  SimDiagramBuilder<double> builder;

  systems::DiagramBuilder<double> Builder;

  {
    std::unique_ptr<RigidBodyTree<double>> world_tree = build_tree(&iiwa_info);

    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

    std::unordered_map<int, std::unique_ptr<systems::StateFeedbackController<double>>> controllers;
    for (const auto& info : iiwa_info) {
      controllers.emplace(info.instance_id, std::make_unique<systems::InverseDynamicsController<double>>(
        info.model_path, info.world_offset, iiwa_kp, iiwa_ki, iiwa_kd, true /* with feedforward acceleration */));
    }

    builder.Setup(std::move(world_tree), controllers);

    builder.ExposePlantOutputPortFullState();

    for (const auto& info : iiwa_info) {
      builder.ExposePlantOutputPortState(info.instance_id);
    }

    sim = Builder.template AddSystem<systems::Diagram<double>>(builder.Build());
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

  systems::TrajectorySource<double>* traj = Builder.template AddSystem<systems::TrajectorySource<double>>(poly_traj);
  systems::TrajectorySource<double>* trajd = Builder.template AddSystem<systems::TrajectorySource<double>>(polyd_traj);
  systems::TrajectorySource<double>* trajdd = Builder.template AddSystem<systems::TrajectorySource<double>>(polydd_traj);

  systems::Multiplexer<double>* input_mux =
    Builder.template AddSystem<systems::Multiplexer<double>>(std::vector<int>{7, 7});

  Builder.Connect(traj->get_output_port(), input_mux->get_input_port(0));
  Builder.Connect(trajd->get_output_port(), input_mux->get_input_port(1));

  for (const auto& info : iiwa_info) {
    const std::vector<SimDiagramBuilder<double>::InputPortIdLookup>& inputs = builder.get_exposed_controller_input_index_pairs(info.instance_id);
    systems::InverseDynamicsController<double>* controller =
      dynamic_cast<systems::InverseDynamicsController<double>*>(builder.get_controller(info.instance_id));
    for (const auto& input : inputs) {
      if (input.subsystem_input_idx == controller->get_input_port_desired_state().get_index()) {
        Builder.Connect(input_mux->get_output_port(0), sim->get_input_port(input.diagram_input_idx));
      } else if (input.subsystem_input_idx == controller->get_input_port_desired_acceleration().get_index()) {
        Builder.Connect(trajdd->get_output_port(), sim->get_input_port(input.diagram_input_idx));
      } else {
        DRAKE_DEMAND(false);
      }
    }
  }

  drake::lcm::DrakeLcm lcm;
  {
    // Connects visualizer.
    systems::DrakeVisualizer* viz_publisher = Builder.template AddSystem<systems::DrakeVisualizer>(
        builder.get_plant()->get_rigid_body_tree(), &lcm);
    Builder.Connect(sim->get_output_port(builder.get_plant_full_state_output_port_index()),
                    viz_publisher->get_input_port(0));
  }

  // Makes a simulation.
  std::unique_ptr<systems::Diagram<double>> diagram = Builder.Build();

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

