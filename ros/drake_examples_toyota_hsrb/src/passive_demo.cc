/**
 * @file
 *
 * Implements a Drake + HSRb demo that is uncontrolled, i.e., the robot remains
 * stationary and its arm falls due to gravity. This demo illustrates the
 * ability to load and simulate the HSRb in Drake. See README.md for
 * instructions on how to run this demo.
 */
#include <chrono>
#include <gflags/gflags.h>

#include "ros/ros.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/toyota_hsrb/passive_demo_common.h"
#include "drake/lcm/drake_lcm.h"

using std::unique_ptr;

namespace drake {
namespace examples {
namespace toyota_hsrb {
namespace {

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

int exec(int argc, char* argv[]) {
  ::ros::init(argc, argv, "hsrb_demo_1");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  lcm::DrakeLcm lcm;
  unique_ptr<systems::Diagram<double>> demo_diagram;
  unique_ptr<systems::Simulator<double>> simulator =
      CreateSimulation(&lcm, &demo_diagram);
  DRAKE_DEMAND(simulator != nullptr);
  DRAKE_DEMAND(demo_diagram != nullptr);
  lcm.StartReceiveThread();

  // The amount of time to simulate between checking for ros::ok();
  const double kSimStepIncrement = 0.1;

  double current_time = std::min(kSimStepIncrement, FLAGS_simulation_sec);
  while (current_time <= FLAGS_simulation_sec && ::ros::ok()) {
    simulator->StepTo(current_time);
    current_time += kSimStepIncrement;
  }

  return 0;
}

}  // namespace
}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::toyota_hsrb::exec(argc, argv);
}
