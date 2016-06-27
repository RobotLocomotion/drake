#include <iostream>
#include <cstdlib>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/plants/RigidBodyTree.h"

using namespace std;

using Eigen::VectorXd;

int main() {
  RigidBodyTree model("examples/Atlas/urdf/atlas_minimal_contact.urdf");

  default_random_engine generator;
  VectorXd q = model.getRandomConfiguration(generator);
  VectorXd v = VectorXd::Random(model.number_of_velocities());
  KinematicsCache<double> cache = model.doKinematics(q, v);

  auto points = drake::Matrix3X<double>::Random(3, 5).eval();
  int body_or_frame_ind = 8;
  int base_or_frame_ind = 0;
  model.transformPointsJacobianDotTimesV(cache, points, body_or_frame_ind,
                                         base_or_frame_ind);

  auto M = model.massMatrix<double>(cache);
  cout << M << endl << endl;

  eigen_aligned_unordered_map<RigidBody const *, drake::TwistVector<double>>
      f_ext;
  drake::TwistVector<double> f_ext_r_foot;
  f_ext_r_foot.setRandom();
  f_ext.insert({model.FindBody("r_foot"), f_ext_r_foot});

  VectorXd vd(model.number_of_velocities());
  vd.setRandom();

  auto C = model.inverseDynamics(cache, f_ext, vd);
  cout << C << endl << endl;
  return 0;
}
