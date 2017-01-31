/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_inverse_dynamics_trajectory_follower.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/make_demo_plan.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

DEFINE_double(simulation_sec, 0.5, "Number of seconds to simulate.");

namespace drake {

using systems::Context;
using systems::Simulator;

namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  std::string model_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";
  std::string alias_group_path = GetDrakePath() +
                                 "/examples/kuka_iiwa_arm/controlled_kuka/"
                                 "inverse_dynamics_controller_config/"
                                 "kuka_alias_groups.yaml";
  std::string controller_config_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/controlled_kuka/"
      "inverse_dynamics_controller_config/kuka_controller.yaml";

  qp_inverse_dynamics::KukaInverseDynamicsTrajectoryFollower demo(
      model_path, alias_group_path, controller_config_path);

  Simulator<double> simulator(demo);
  Context<double>* context = simulator.get_mutable_context();

  // Sets initial condition.
  VectorX<double> desired_state = VectorX<double>::Zero(14);
  demo.get_kuka_plant().set_state_vector(demo.get_kuka_context(context),
                                         desired_state);

  // Sets desired trajectory.
  demo.SetDesiredTrajectory(MakePlan(model_path), context);

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
