#include "drake/examples/Fetch/fetch_common.h"

#include <map>

#include "drake/multibody/parsers/urdf_parser.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace Fetch {

// See the @file docblock in fetch_common.h for joint index descriptions.
void VerifyFetchTree(const RigidBodyTree<double>& tree) {
  std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

  int joint_idx = 7;  // joints 0-6 are the floating base
  DRAKE_DEMAND(name_to_idx.size() == kNumDofs);
  DRAKE_DEMAND(name_to_idx.count("r_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["r_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("l_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["l_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("torso_lift_joint"));
  DRAKE_DEMAND(name_to_idx["torso_lift_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("head_pan_joint"));
  DRAKE_DEMAND(name_to_idx["head_pan_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("head_tilt_joint"));
  DRAKE_DEMAND(name_to_idx["head_tilt_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("shoulder_pan_joint"));
  DRAKE_DEMAND(name_to_idx["shoulder_pan_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("shoulder_lift_joint"));
  DRAKE_DEMAND(name_to_idx["shoulder_lift_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("upperarm_roll_joint"));
  DRAKE_DEMAND(name_to_idx["upperarm_roll_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("elbow_flex_joint"));
  DRAKE_DEMAND(name_to_idx["elbow_flex_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("forearm_roll_joint"));
  DRAKE_DEMAND(name_to_idx["forearm_roll_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("wrist_flex_joint"));
  DRAKE_DEMAND(name_to_idx["wrist_flex_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("wrist_roll_joint"));
  DRAKE_DEMAND(name_to_idx["wrist_roll_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("r_gripper_finger_joint"));
  DRAKE_DEMAND(name_to_idx["r_gripper_finger_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("l_gripper_finger_joint"));
  DRAKE_DEMAND(name_to_idx["l_gripper_finger_joint"] == joint_idx++);
}

void CreateTreeFromFloatingModelAtPose(const std::string& model_file_name,
                                       RigidBodyTreed* tree,
                                       const Eigen::Isometry3d& pose) {
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      pose);

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      model_file_name, drake::multibody::joints::kQuaternion, weld_to_frame,
      tree);
}

}  // namespace Fetch
}  // namespace examples
}  // namespace drake
