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
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

DEFINE_double(simulation_sec, 0.5, "Number of seconds to simulate.");

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

  RigidBodyTreed tree;
  CreateTreedFromFixedModelAtPose(kUrdfPath, &tree);

  std::unique_ptr<PiecewisePolynomialTrajectory> cartesian_trajectory =
      SimpleCartesianWayPointPlanner(tree, "iiwa_link_ee", way_points,
                                     time_stamps);

  KukaDemo<double> model(std::move(cartesian_trajectory));
  Simulator<double> simulator(model);

  Context<double>* context = simulator.get_mutable_context();
  model.SetDefaultState(*context, context->get_mutable_state());
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
