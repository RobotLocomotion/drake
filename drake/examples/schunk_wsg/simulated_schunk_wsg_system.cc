#include "drake/examples/schunk_wsg/simulated_schunk_wsg_system.h"

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/sdf_parser.h"

namespace drake {

using multibody::joints::kFixed;

namespace examples {
namespace schunk_wsg {

template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSimulatedSchunkWsgSystem() {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<T>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(
      GetDrakePath() + "/examples/schunk_wsg/models/schunk_wsg_50.sdf",
      kFixed, nullptr /* weld to frame */, rigid_body_tree.get());
  return std::make_unique<drake::systems::RigidBodyPlant<T>>(
      std::move(rigid_body_tree));
}

template std::unique_ptr<drake::systems::RigidBodyPlant<double>>
CreateSimulatedSchunkWsgSystem();

}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
