#include "drake/multibody/global_inverse_kinematics.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace {
GTEST_TEST(TestGlobakIK, KukaTest) {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<double>>();
  const std::string model_path = drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/iiwa14/"
          "iiwa14_simplified_collision.urdf";

  parsers::urdf::AddModelInstanceFromUrdfFile(
    model_path,
    drake::multibody::joints::kFixed,
    nullptr,
    rigid_body_tree.get());

  AddFlatTerrainToWorld(rigid_body_tree.get());

  GlobalInverseKinematics global_ik(rigid_body_tree);
}
}  // namespace
}  // namespace multibody
}  // namespace drake