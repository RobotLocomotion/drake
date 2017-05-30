#include "drake/examples/kinova_jaco_arm/jaco_common.h"

#include <map>

#include "drake/multibody/parsers/urdf_parser.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {

// See the @file docblock in jaco_common.h for joint index descriptions.
void VerifyJacoTree(const RigidBodyTree<double>& tree) {
  std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.size() == kNumDofs);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_1"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_1"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_2"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_2"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_3"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_3"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_4"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_4"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_5"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_5"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_6"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_6"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_finger_1"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_finger_1"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_finger_2"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_finger_2"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("j2n6s300_joint_finger_3"));
  DRAKE_DEMAND(name_to_idx["j2n6s300_joint_finger_3"] == joint_idx++);
}

void CreateTreeFromFixedModelAtPose(const std::string& model_file_name,
                                     RigidBodyTreed* tree,
                                     const Vector3d& position,
                                     const Vector3d& orientation) {
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world",
      nullptr, position, orientation);

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      model_file_name, drake::multibody::joints::kFixed,
      weld_to_frame, tree);
}

void SetPositionControlledJacoGains(VectorXd* Kp, VectorXd* Ki,
                                    VectorXd* Kd) {
  // All gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(kNumDofs);
  *Kp = VectorXd::Constant(kNumDofs, 100.0);
  Kd->resize(Kp->size());
  *Kd = 2.0 * Kp->array().sqrt();
  *Ki = VectorXd::Zero(kNumDofs);
}

}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
