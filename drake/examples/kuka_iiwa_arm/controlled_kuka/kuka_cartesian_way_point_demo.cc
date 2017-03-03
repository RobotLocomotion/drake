/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to track the 4
/// vertices of a square in space.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/kuka_demo_plant_builder.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

using std::unique_ptr;

namespace drake {

using systems::Context;
using systems::Simulator;

namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  Eigen::Vector3d kSquareCorner1(0.3, -0.3, 0.3);
  Eigen::Vector3d kSquareCorner2(0.3, -0.3, 0.6);
  Eigen::Vector3d kSquareCorner3(0.3, 0.3, 0.6);
  Eigen::Vector3d kSquareCorner4(0.3, 0.3, 0.3);

  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(kSquareCorner1);
  way_points.push_back(kSquareCorner2);
  way_points.push_back(kSquareCorner3);
  way_points.push_back(kSquareCorner4);
  way_points.push_back(kSquareCorner1);

  std::vector<double> time_stamps{1.0, 2.0, 3.0, 4.0, 5.0};

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::lcm::DrakeLcm lcm;
  CreateTreedFromFixedModelAtPose(kUrdfPath, tree.get());

  std::unique_ptr<PiecewisePolynomialTrajectory> cartesian_trajectory =
      SimpleCartesianWayPointPlanner(*tree, "iiwa_link_ee", way_points,
                                     time_stamps);
  std::unique_ptr<Trajectory> cartesian_trajectory_dot = cartesian_trajectory->derivative(1);

  const int kInstanceId = RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;

  SimDiagramBuilder<double> builder;
  // Adds a plant
  builder.AddPlant(std::move(tree));

  // Adds a iiwa controller
  {
    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
    std::unique_ptr<systems::StateFeedbackController<double>> controller =
        std::make_unique<systems::InverseDynamicsController<double>>(GetDrakePath() + kUrdfPath, nullptr, iiwa_kp, iiwa_ki, iiwa_kd, false /* without feedforward acceleration */);
    builder.AddController(kInstanceId, std::move(controller));
  }
  systems::InverseDynamicsController<double>* controller = dynamic_cast<systems::InverseDynamicsController<double>*>(builder.get_controller(kInstanceId));

  // Adds desired trajectory sources.
  auto traj = builder.template AddSystem<systems::TrajectorySource<double>>(*cartesian_trajectory);
  auto trajd = builder.template AddSystem<systems::TrajectorySource<double>>(*cartesian_trajectory_dot);

  auto input_mux = builder.template AddSystem<systems::Multiplexer<double>>(std::vector<int>{traj->get_output_port().size(), trajd->get_output_port().size()});
  builder.Connect(traj->get_output_port(), input_mux->get_input_port(0));
  builder.Connect(trajd->get_output_port(), input_mux->get_input_port(1));
  builder.Connect(input_mux->get_output_port(0), controller->get_input_port_desired_state());

  // Connects visualizer.
  systems::DrakeVisualizer* viz_publisher = builder.template AddSystem<systems::DrakeVisualizer>(
      builder.get_plant()->get_rigid_body_tree(), &lcm);
  builder.Connect(builder.get_plant()->get_output_port(0),
      viz_publisher->get_input_port(0));

  // Finishes wiring.
  builder.WireThingsTogether();

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
