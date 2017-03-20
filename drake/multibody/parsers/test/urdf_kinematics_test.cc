#include <cstdlib>
#include <iostream>
#include <memory>

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: urdf_kinematics_test urdf_filename" << endl;
    exit(-1);
  }

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      argv[1], drake::multibody::joints::kRollPitchYaw, tree.get());

  cout << "=======" << endl;

  // run kinematics with second derivatives 100 times
  Eigen::VectorXd q = tree->getZeroConfiguration();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(tree->get_num_velocities());
  int i;

  if (argc >= 2 + tree->get_num_positions()) {
    for (i = 0; i < tree->get_num_positions(); i++)
      sscanf(argv[2 + i], "%lf", &q(i));
  }

  // for (i=0; i<tree->num_dof; i++)
  //   q(i)=(double)rand() / RAND_MAX;
  KinematicsCache<double> cache = tree->doKinematics(q, v);
  //  }

  //  const Vector4d zero(0, 0, 0, 1);
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  for (i = 0; i < static_cast<int>(tree->bodies.size()); i++) {
    //    tree->forwardKin(i, zero, 1, pt);
    auto pt = tree->transformPoints(cache, zero, i, 0);
    auto rpy = tree->relativeRollPitchYaw(cache, i, 0);
    Eigen::Matrix<double, 6, 1> x;
    x << pt, rpy;
    //    cout << i << ": forward kin: " << tree->bodies[i].name_ << " is at
    //    " << pt.transpose() << endl;
    cout << tree->bodies[i]->get_name() << " ";
    cout << x.transpose() << endl;
    //    for (int j=0; j<pt.size(); j++)
    //        cout << pt(j) << " ";
  }

  auto phi = tree->positionConstraints<double>(cache);
  cout << "phi = " << phi.transpose() << endl;

  return 0;
}
