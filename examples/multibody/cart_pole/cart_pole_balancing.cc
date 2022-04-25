#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

int do_main() {
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/multibody/cart_pole/cart_pole.sdf");
  MultibodyPlant<double> plant(FLAGS_time_step);
  Parser(&plant).AddModelFromFile(full_name);
  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  plant.get_actuation_input_port().FixValue(context.get(), 0.);
  plant.SetPositionsAndVelocities(context.get(),
                                  Eigen::Vector4d{0, M_PI, 0, 0});

  auto linearized_plant = systems::Linearize(
      plant, *context, plant.get_actuation_input_port().get_index(),
      systems::OutputPortSelection::kNoOutput);

  std::cout << "A = " << linearized_plant->A() << std::endl;
  std::cout << "B = " << linearized_plant->B() << std::endl;

  return 0;
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::cart_pole::do_main();
}
