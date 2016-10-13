#include "drake/examples/schunk_gripper/simulated_schunk_system.h"

#include "drake/common/drake_export.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/common/drake_path.h"

namespace drake {
namespace examples {
namespace schunk_gripper {

template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSimulatedSchunkSystem() {
  auto rigid_body_tree = std::make_unique<RigidBodyTree>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
      "/examples/schunk_gripper/models/schunk_gripper.sdf",
      drake::systems::plants::joints::kFixed, nullptr /* weld to frame */,
      rigid_body_tree.get());

  return std::make_unique<drake::systems::RigidBodyPlant<T>>(
      std::move(rigid_body_tree));
}

template DRAKE_EXPORT std::unique_ptr<drake::systems::RigidBodyPlant<double>>
CreateSimulatedSchunkSystem();

}  // namespace schunk_gripper
}  // namespace examples
}  // namespace drake
