#include <iostream>
#include <memory>

#include "drake/automotive/car_simulation.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_receive_thread.h"

namespace drake {
namespace automotive {
namespace {

int do_main(int argc, const char* argv[]) {
  int num_cars = 100;
  if (argc == 2) {
    num_cars = atoi(argv[1]);
    if (num_cars < 1) {
      std::cerr << "The number of cars must be >= 1.\n";
      std::exit(1);
    }
  }

  // Objects shared across systems.
  auto lcm = std::make_unique<lcm::LCM>();
  const SimpleCarStateTranslator simple_car_state_translator;
  const EulerFloatingJointStateTranslator euler_floating_joint_state_translator;
  std::vector<std::unique_ptr<systems::System<double>>> owned_systems;

  // Add all of the desired cars.
  auto builder = std::make_unique<systems::DiagramBuilder<double>>();
  for (int i = 0; i < num_cars; ++i) {
    const std::string suffix("_" + std::to_string(i));
    auto trajectory_car = CreateTrajectoryCarSystem(i);
    auto coord_transform =
        std::make_unique<SimpleCarToEulerFloatingJoint<double>>();
    auto simple_car_state_publisher =
        std::make_unique<systems::lcm::LcmPublisherSystem>(
            "SIMPLE_CAR_STATE" + suffix,
            simple_car_state_translator, lcm.get());
    auto euler_floating_joint_state_publisher =
        std::make_unique<systems::lcm::LcmPublisherSystem>(
            "FLOATING_JOINT_STATE" + suffix,
            euler_floating_joint_state_translator, lcm.get());

    builder->Connect(*trajectory_car, *simple_car_state_publisher);
    builder->Connect(*trajectory_car, *coord_transform);
    builder->Connect(*coord_transform, *euler_floating_joint_state_publisher);

    owned_systems.emplace_back(std::move(trajectory_car));
    owned_systems.emplace_back(std::move(coord_transform));
    owned_systems.emplace_back(std::move(simple_car_state_publisher));
    owned_systems.emplace_back(std::move(euler_floating_joint_state_publisher));
  }

  auto diagram = builder->Build();

  auto lcm_receive_thread = std::make_unique<systems::lcm::LcmReceiveThread>(
      lcm.get());

  auto simulator = std::make_unique<systems::Simulator<double>>(*diagram);
  simulator->Initialize();
  while (true) {
    const double time = simulator->get_context().get_time();
    SPDLOG_TRACE(drake::log(), "Time is now {}", time);
    simulator->StepTo(time + 0.01);
  }

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::automotive::do_main(argc, argv);
}
