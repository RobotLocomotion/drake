#include <vector>

#include "drake/common/drake_path.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_urdf.h"

using Eigen::Matrix3Xd;
using Eigen::VectorXd;

using drake::systems::plants::joints::kRollPitchYaw;
using drake::systems::plants::joints::kFixed;

int main(int argc, char* argv[]) {
  RigidBodyTree tree;

  for (int i = 0; i < 10; ++i) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/systems/plants/test/PointMass.urdf",
            kRollPitchYaw, &tree);
  }

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/systems/plants/test/FallingBrick.urdf",
          kFixed, &tree);

  VectorXd q = VectorXd::Random(tree.get_num_positions());
  VectorXd v = VectorXd::Random(tree.get_num_velocities());
  auto kinsol = tree.doKinematics(q, v);

  VectorXd phi;
  Matrix3Xd normal, xA, xB;
  std::vector<int> bodyA_idx, bodyB_idx;
  tree.collisionDetect(kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                       false);
}
