/**
 * @file test demo to visualize a given tree in a random set of configurations.
 */
#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

DEFINE_int32(num_configurations, 10,
             "Number of random test configurations to display in the demo");

namespace drake {
namespace manipulation {
namespace {

int DoMain() {
  drake::lcm::DrakeLcm lcm;

  // Adds a demo tree.
  const std::string kModelPath =
      // "drake/manipulation/models/iiwa_description/urdf/"
      // "iiwa14_polytope_collision.urdf";
      "drake/manipulation/models/allegro_hand_description/urdf/"
      "allegro_hand_description_left.urdf";

  auto tree = std::make_unique<RigidBodyTree<double>>();

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      Eigen::Vector3d::Zero() /* base position */,
      Eigen::Vector3d::Zero() /* base orientation */);

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(kModelPath), drake::multibody::joints::kFixed,
      weld_to_frame, tree.get());

  tree->compile();
  SimpleTreeVisualizer simple_tree_visualizer(*tree.get(), &lcm);


  std::cout<<"Number of actuators: "<<tree->get_num_actuators()<<"\n";
  std::cout<<"Number of bodies: "<<tree->get_num_bodies()<<"\n";
  std::cout<<"Number of velocities: "<<tree->get_num_velocities()<<"\n";
  std::cout<<"Number of positions: "<<tree->get_num_positions()<<"\n";
  std::cout<<"Number of model instances: "<<tree->get_num_model_instances()<<"\n";

  // Simple demo that iterates through a bunch of joint configurations.
  for (int i = 0; i < FLAGS_num_configurations; ++i) {
    simple_tree_visualizer.visualize(Eigen::VectorXd::Random(
                  tree->get_num_positions()));

    // Sleep for a second just so that the new configuration can be seen
    // on the visualizer.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::manipulation::DoMain();
}
