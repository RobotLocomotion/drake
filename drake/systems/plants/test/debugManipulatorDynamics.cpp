#include "RigidBodyManipulator.h"
#include <iostream>
#include <cstdlib>
#include <memory>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyManipulator model("examples/Atlas/urdf/atlas_minimal_contact.urdf");

  int gradient_order = 1;
  int nq = model.num_positions;
  int nv = model.num_velocities;

  VectorXd q = VectorXd::Random(model.num_positions);
  VectorXd v = VectorXd::Random(model.num_velocities);
  KinematicsCache<double> cache = model.doKinematics(q, v, 1);

  auto points = Matrix<double, 3, Eigen::Dynamic>::Random(3, 5).eval();
  int body_or_frame_ind = 8;
  int base_or_frame_ind = 0;
  int rotation_type = 0;
  model.forwardJacDotTimesV(cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);

  auto M = model.massMatrix<double>(cache, gradient_order);
  cout << M.value() << endl << endl;

  eigen_aligned_unordered_map<RigidBody const *, GradientVar<double, TWIST_SIZE, 1> > f_ext;
  GradientVar<double, TWIST_SIZE, 1> f_ext_r_foot(TWIST_SIZE, 1, nq + nv, gradient_order);
  f_ext_r_foot.value().setRandom();
  f_ext_r_foot.gradient().value().setRandom();
  f_ext.insert({model.findLink("r_foot").get(), f_ext_r_foot});

  GradientVar<double, Eigen::Dynamic, 1> vd(model.num_velocities, 1, nq + nv, gradient_order);
  vd.value().setRandom();
  if (vd.hasGradient())
    vd.gradient().value().setRandom();

  auto C = model.inverseDynamics(cache, f_ext, vd, gradient_order);
  cout << C.value() << endl << endl;
  return 0;
}
