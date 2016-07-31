#include "drake/examples/Cars/simple_car.h"

#include <cmath>

#include "drake/common/drake_path.h"

#include "drake/examples/Cars/car_simulation.h"
#include "drake/examples/Cars/gen/euler_floating_joint_state.h"
#include "drake/examples/Cars/lcm_tap.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

using drake::AffineSystem;
using drake::BotVisualizer;
using drake::NullVector;
using drake::cascade;

namespace drake {
namespace examples {
namespace cars {
namespace {

int do_main(int argc, const char* argv[]) {
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  auto car = std::make_shared<SimpleCar>();
  auto adapter = CreateSimpleCarVisualizationAdapter();

  //
  // Load a simplistic rendering from accompanying URDF file.
  //
  auto tree = std::make_shared<RigidBodyTree>(
      drake::GetDrakePath() + "/examples/Cars/models/boxcar.urdf");

  auto viz =
      std::make_shared<BotVisualizer<EulerFloatingJointState> >(
          lcm, tree);

  // Make some taps to publish intermediate states to LCM.
  auto car_tap = std::make_shared<LcmTap<SimpleCarState> >(lcm);
  auto adapter_tap = std::make_shared<LcmTap<EulerFloatingJointState> >(lcm);

  // Assemble car, adapter, and visualizer, with intervening taps.
  auto car_tapped = cascade(car, car_tap);
  auto adapter_tapped = cascade(adapter, adapter_tap);
  auto adapt_viz = cascade(adapter_tapped, viz);
  auto sys = cascade(car_tapped, adapt_viz);

  SimpleCarState<double> initial_state;
  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), initial_state);

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::examples::cars::do_main(argc, argv);
}
