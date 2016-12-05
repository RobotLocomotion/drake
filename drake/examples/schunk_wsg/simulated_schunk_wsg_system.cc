#include "drake/examples/schunk_wsg/simulated_schunk_wsg_system.h"

#include "drake/common/drake_path.h"
#include "drake/multibody/parser_sdf.h"

namespace drake {
namespace examples {
namespace schunk_wsg {

template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSimulatedSchunkWsgSystem() {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<T>>();
  const PackageMap package_map;
  drake::parsers::sdf::AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
      "/examples/schunk_wsg/models/schunk_wsg_50.sdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      rigid_body_tree.get());

  return std::make_unique<drake::systems::RigidBodyPlant<T>>(
      std::move(rigid_body_tree));
}

template std::unique_ptr<drake::systems::RigidBodyPlant<double>>
CreateSimulatedSchunkWsgSystem();

}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
