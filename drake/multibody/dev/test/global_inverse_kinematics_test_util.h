#include "drake/multibody/global_inverse_kinematics.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
std::unique_ptr<RigidBodyTree<double>> ConstructKuka();

class KukaTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KukaTest)

  KukaTest();

  ~KukaTest() override {};

  /**
   * Given the solution computed from global IK, reconstruct the posture, then
   * compute the body pose using forward kinematics with the reconstructed
   * posture. Compare that forward kinematics body pose, with the body pose
   * in the global IK.
   */
  void CheckGlobalIKSolution(double pos_tol, double orient_tol) const;

 protected:
  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree_;
  GlobalInverseKinematics global_ik_;
  int ee_idx_;  // end effector's body index.
};
}  // namespace multibody
}  // namespace drake
