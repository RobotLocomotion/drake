
#include "drake/common/drake_path.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace drake;

int main(int argc, char* argv[]) {
  RigidBodyTree tree;

  for (int i = 0; i < 10; i++) {
    RigidBodyTree::ModelToInstanceIDMap model_instance_id_table;
    drake::parsers::urdf::AddModelInstanceFromURDF(
        GetDrakePath() + "/systems/plants/test/PointMass.urdf",
        DrakeJoint::ROLLPITCHYAW, &tree, &model_instance_id_table);
  }

  RigidBodyTree::ModelToInstanceIDMap> model_instance_id_table;
  drake::parsers::urdf::AddModelInstanceFromURDF(
      GetDrakePath() + "/systems/plants/test/FallingBrick.urdf",
      DrakeJoint::FIXED, &tree, &model_instance_id_table);

  VectorXd q = VectorXd::Random(tree.number_of_positions());
  VectorXd v = VectorXd::Random(tree.number_of_velocities());
  auto kinsol = tree.doKinematics(q, v);

  VectorXd phi;
  Matrix3Xd normal, xA, xB;
  std::vector<int> bodyA_idx, bodyB_idx;
  tree.collisionDetect(kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                       false);
}
