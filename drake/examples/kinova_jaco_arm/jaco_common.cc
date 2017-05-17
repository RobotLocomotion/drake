#include <map>

#include "drake/common/drake_path.h"
#include "drake/examples/kinova_jaco_arm/jaco_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::aligned_allocator;
using std::string;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {

void VerifyJacoTree(const RigidBodyTree<double>& tree) {
  std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

  int joint_idx = 0;
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

void CreateTreedFromFixedModelAtPose(const std::string& model_file_name,
                                     RigidBodyTreed* tree,
                                     const Vector3d& position,
                                     const Vector3d& orientation) {
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr, position,
      orientation);

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + model_file_name, drake::multibody::joints::kFixed,
      weld_to_frame, tree);
}

void SetPositionControlledJacoGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  Kp->resize(9);
  *Kp << 100, 100, 100, 100, 100, 100, 100, 100, 100;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
  }
  *Ki = Eigen::VectorXd::Zero(9);
}

}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
