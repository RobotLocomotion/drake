#include "drake/multibody/test/rigid_body_tree/rigid_body_tree_compare_to_clone.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/test/rigid_body_actuator_compare_to_clone.h"
#include "drake/multibody/test/rigid_body_compare_to_clone.h"
#include "drake/multibody/test/rigid_body_frame_compare_to_clone.h"
#include "drake/multibody/test/rigid_body_loop_compare_to_clone.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_tree {

bool CompareToClone(const RigidBodyTree<double>& tree) {
  std::unique_ptr<RigidBodyTree<double>> clone = tree.Clone();
  if (tree.get_num_model_instances() != clone->get_num_model_instances()) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): num model instances mismatch:\n"
        "  - this: {}\n"
        "  - clone: {}",
        tree.get_num_model_instances(),
        clone->get_num_model_instances());
    return false;
  }
  if (tree.get_num_bodies() != clone->get_num_bodies()) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): num bodies mismatch:\n"
        "  - this: {}\n"
        "  - clone: {}",
        tree.get_num_bodies(),
        clone->get_num_bodies());
    return false;
  }
  if (tree.get_num_frames() != clone->get_num_frames()) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): num frames mismatch:\n"
        "  - this: {}\n"
        "  - clone: {}",
        tree.get_num_frames(),
        clone->get_num_frames());
    return false;
  }
  if (tree.get_num_positions() != clone->get_num_positions()) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): num positions mismatch:\n"
        "  - this: {}\n"
        "  - clone: {}",
        tree.get_num_positions(),
        clone->get_num_positions());
    return false;
  }
  if (tree.get_num_velocities() != clone->get_num_velocities()) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): num velocities mismatch:\n"
        "  - this: {}\n"
        "  - clone: {}",
        tree.get_num_velocities(),
        clone->get_num_velocities());
    return false;
  }
  if (tree.get_num_actuators() != clone->get_num_actuators()) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): num actuators mismatch:\n"
        "  - this: {}\n"
        "  - clone: {}",
        tree.get_num_actuators(),
        clone->get_num_actuators());
    return false;
  }
  for (int i = 0; i < tree.get_num_bodies(); ++i) {
    if (!rigid_body::CompareToClone(*tree.get_bodies().at(i),
                                    *clone->get_bodies().at(i))) {
      drake::log()->debug(
          "CompareToClone(RigidBodyTree): bodies mismatch at index {}.", i);
      return false;
    }
  }
  for (int i = 0; i < tree.get_num_frames(); ++i) {
    if (!rigid_body_frame::CompareToClone(*tree.get_frames().at(i),
        *clone->get_frames().at(i))) {
      drake::log()->debug(
          "CompareToClone(RigidBodyTree): frames mismatch at index {}.", i);
      return false;
    }
  }
  for (int i = 0; i < tree.get_num_actuators(); ++i) {
    if (!rigid_body_actuator::CompareToClone(
        tree.actuators.at(i), clone->actuators.at(i))) {
      drake::log()->debug(
          "CompareToClone(RigidBodyTree): actuators mismatch at index {}.", i);
      return false;
    }
  }
  for (int i = 0; i < static_cast<int>(tree.loops.size()); ++i) {
    const RigidBodyLoop<double>* this_loop = &tree.loops.at(i);
    const RigidBodyLoop<double>* other_loop = &clone->loops.at(i);
    if (!rigid_body_loop::CompareToClone(*this_loop, *other_loop)) {
      drake::log()->debug(
        "CompareToClone(RigidBodyTree): loops mismatch at index {}.", i);
      return false;
    }
  }
  if (tree.a_grav != clone->a_grav) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): gravity vector mismatch:\n"
        "  - this: {}\n"
        "  - clone: {}",
        tree.a_grav,
        clone->a_grav);
    return false;
  }
  if (tree.B != clone->B) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): B matrix mismatch:\n"
        "  - this:\n{}\n"
        "  - clone:\n{}",
        tree.B,
        clone->B);
    return false;
  }
  if (tree.initialized() != clone->initialized()) {
    drake::log()->debug(
        "CompareToClone(RigidBodyTree): initialized_ mismatch:\n"
        "  - this:\n{}\n"
        "  - clone:\n{}",
        tree.initialized(),
        clone->initialized());
    return false;
  }
  return true;
}

}  // namespace rigid_body_tree
}  // namespace test
}  // namespace multibody
}  // namespace drake
