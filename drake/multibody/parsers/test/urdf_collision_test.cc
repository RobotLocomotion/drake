#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using std::cerr;
using std::cout;
using std::endl;
using std::vector;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: urdf_collision_test urdf_filename" << endl;
    exit(-1);
  }

  auto tree = std::make_unique<RigidBodyTree<double>>();

  // TODO(liang.fok) The following method call assumes this program is only
  // called using URDFs that satisfy the requirements of this method. Noteably,
  // if the URDF references meshes using package://, the referenced packages
  // can be found by crawling up the directory tree relative to the URDF file.
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      argv[1], drake::multibody::joints::kRollPitchYaw, tree.get());

  // run kinematics with second derivatives 100 times
  Eigen::VectorXd q = tree->getZeroConfiguration();
  if (argc >= 2 + tree->get_num_positions()) {
    for (int i = 0; i < tree->get_num_positions(); i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  // for (i=0; i<tree->num_dof; i++)
  //   q(i)=(double)rand() / RAND_MAX;
  KinematicsCache<double> cache = tree->doKinematics(q);
  //  }

  Eigen::VectorXd phi;
  Eigen::Matrix3Xd normal, xA, xB;
  vector<int> bodyA_idx, bodyB_idx;

  tree->collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx);

  cout << "=======" << endl;
  for (int j = 0; j < phi.rows(); ++j) {
    cout << phi(j) << " ";
    for (int i = 0; i < 3; ++i) {
      cout << normal(i, j) << " ";
    }
    for (int i = 0; i < 3; ++i) {
      cout << xA(i, j) << " ";
    }
    for (int i = 0; i < 3; ++i) {
      cout << xB(i, j) << " ";
    }
    cout << tree->bodies[bodyA_idx.at(j)]->get_name() << " ";
    cout << tree->bodies[bodyB_idx.at(j)]->get_name() << endl;
  }

  return 0;
}
