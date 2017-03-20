#include <cstdlib>
#include <iostream>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::VectorXd;
using std::cout;
using std::default_random_engine;
using std::endl;

int main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/Atlas/urdf/atlas_minimal_contact.urdf",
      drake::multibody::joints::kRollPitchYaw, tree.get());

  default_random_engine generator;
  VectorXd q = tree->getRandomConfiguration(generator);
  VectorXd v = VectorXd::Random(tree->get_num_velocities());
  KinematicsCache<double> cache = tree->doKinematics(q, v);

  auto points = drake::Matrix3X<double>::Random(3, 5).eval();
  int body_or_frame_ind = 8;
  int base_or_frame_ind = 0;
  tree->transformPointsJacobianDotTimesV(cache, points, body_or_frame_ind,
                                         base_or_frame_ind);

  auto M = tree->massMatrix<double>(cache);
  cout << M << endl << endl;

  RigidBodyTree<double>::BodyToWrenchMap external_wrenches;
  drake::WrenchVector<double> f_ext_r_foot;
  f_ext_r_foot.setRandom();
  external_wrenches.insert({tree->FindBody("r_foot"), f_ext_r_foot});

  VectorXd vd(tree->get_num_velocities());
  vd.setRandom();

  auto C = tree->inverseDynamics(cache, external_wrenches, vd);
  cout << C << endl << endl;
  return 0;
}
