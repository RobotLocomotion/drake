#include "drake/systems/plants/RigidBodyTree.h"
#include <iostream>
#include <cstdlib>
#include <memory>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyTree model("examples/Atlas/urdf/atlas_minimal_contact.urdf");

  default_random_engine generator;
  VectorXd q = model.getRandomConfiguration(generator);
  VectorXd v = VectorXd::Random(model.num_velocities);
  KinematicsCache<double> cache = model.doKinematics(q, v);

  auto points = Matrix<double, 3, Eigen::Dynamic>::Random(3, 5).eval();
  int body_or_frame_ind = 8;
  int base_or_frame_ind = 0;
  model.transformPointsJacobianDotTimesV(cache, points, body_or_frame_ind, base_or_frame_ind);

  auto M = model.massMatrix<double>(cache);
  cout << M << endl << endl;

  eigen_aligned_unordered_map<RigidBody const *, Matrix<double, TWIST_SIZE, 1> > f_ext;
  Matrix<double, TWIST_SIZE, 1> f_ext_r_foot;
  f_ext_r_foot.setRandom();
  f_ext.insert({model.findLink("r_foot").get(), f_ext_r_foot});

  Matrix<double, Eigen::Dynamic, 1> vd(model.num_velocities);
  vd.setRandom();

  auto C = model.inverseDynamics(cache, f_ext, vd);
  cout << C << endl << endl;
  return 0;
}
