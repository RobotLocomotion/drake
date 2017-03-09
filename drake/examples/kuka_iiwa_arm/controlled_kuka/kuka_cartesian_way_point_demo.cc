/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to track the 4
/// vertices of a square in space.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

using std::unique_ptr;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char kUrdfPath[] =
    "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf";

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
  CreateTreedFromFixedModelAtPose(kUrdfPath, tree.get());

  std::unique_ptr<PiecewisePolynomialTrajectory> cartesian_trajectory =
      SimpleCartesianWayPointPlanner(*tree, "iiwa_link_ee", way_points,
                                     time_stamps);

  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;
  // Adds a plant
  builder.AddPlant(std::move(tree));
  builder.AddVisualizer(&lcm);

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller =
      builder.AddController<systems::InverseDynamicsController<double>>(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
          GetDrakePath() + kUrdfPath, nullptr, iiwa_kp, iiwa_ki, iiwa_kd,
          false /* no feedforward acceleration */);

  // Adds a trajectory source for desired state.
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();
  auto traj_src =
      diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
          *cartesian_trajectory, 1 /* outputs q + v */);

  diagram_builder->Connect(traj_src->get_output_port(),
                  controller->get_input_port_desired_state());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

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
