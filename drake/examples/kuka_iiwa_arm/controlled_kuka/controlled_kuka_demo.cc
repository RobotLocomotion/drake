/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/kuka_demo_plant_builder.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/make_demo_plan.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

namespace drake {

using systems::Context;
using systems::Simulator;

namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  std::string path = GetDrakePath() + kUrdfPath;

  KukaDemo<double> model(MakeKukaDemoTrajectory(path));
  Simulator<double> simulator(model);
  Context<double>* context = simulator.get_mutable_context();

  VectorX<double> desired_state = VectorX<double>::Zero(14);
  model.get_kuka_plant().set_state_vector(model.get_kuka_context(context),
                                          desired_state);

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
