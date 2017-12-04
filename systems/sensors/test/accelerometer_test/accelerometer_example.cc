#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/sensors/test/accelerometer_test/accelerometer_example_diagram.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");

DEFINE_double(initial_q, 0,
    "The initial position (in rad) of the pendulum's pivot joint.");

DEFINE_double(initial_v, 0,
    "The initial velocity (in rad/s) of the pendulum's pivot joint.");

using std::unique_ptr;
using std::make_unique;
using std::move;

namespace drake {
namespace systems {
namespace sensors {
namespace {

int exec(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  DRAKE_DEMAND(FLAGS_simulation_sec >= 0);
  ::drake::lcm::DrakeLcm real_lcm;
  AccelerometerExampleDiagram diagram(&real_lcm);
  auto visualizer = make_unique<DrakeVisualizer>(diagram.get_tree(), &real_lcm);
  diagram.Initialize(move(visualizer));
  diagram.get_mutable_logger()->enable_log_to_console();

  unique_ptr<Context<double>> context = diagram.AllocateContext();
  diagram.SetDefaultState(*context, &context->get_mutable_state());

  // Sets the initial state of the pendulum.
  diagram.SetInitialState(context.get(), FLAGS_initial_q, FLAGS_initial_v);

  Simulator<double> simulator(diagram, std::move(context));
  simulator.Initialize();
  simulator.set_target_realtime_rate(1);
  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::systems::sensors::exec(argc, argv);
}
