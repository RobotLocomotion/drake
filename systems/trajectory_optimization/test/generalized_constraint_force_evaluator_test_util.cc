#include "drake/systems/trajectory_optimization/test/generalized_constraint_force_evaluator_test_util.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
// Construct a RigidBodyTree containing a four bar linkage.
std::unique_ptr<RigidBodyTree<double>> ConstructFourBarTree() {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/simple_four_bar/FourBar.urdf"),
      multibody::joints::kFixed, tree);
  DRAKE_DEMAND(tree->get_num_actuators() != 0);
  return std::unique_ptr<RigidBodyTree<double>>(tree);
}
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
