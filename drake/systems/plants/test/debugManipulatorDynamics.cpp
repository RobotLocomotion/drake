#include "RigidBodyManipulator.h"
#include "RigidBodyManipulator.h"
#include "drakeGeometryUtil.h"
#include <iostream>
#include <cstdlib>
#include <random>
#include <memory>

using namespace std;
using namespace Eigen;
int main()
{
  RigidBodyManipulator* model = new RigidBodyManipulator("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  int gradient_order = 1;
  int nq = model->num_positions;
  int nv = model->num_velocities;

  default_random_engine generator;
  uniform_real_distribution<double> uniform;

  for (vector<shared_ptr<RigidBody> >::const_iterator it = model->bodies.begin(); it != model->bodies.end(); ++it) {
    RigidBody& body = **it;
    double mass = uniform(generator);
    Matrix3d moment_of_inertia = Matrix3d::Random();
    moment_of_inertia *= moment_of_inertia;
    Vector3d com = Vector3d::Random();

    body.I.topLeftCorner<3, 3>() = moment_of_inertia;
    body.I.bottomRightCorner<3, 3>() = mass * Matrix3d::Identity();
    body.I.topRightCorner<3, 3>() = vectorToSkewSymmetric((mass * com).eval());
    body.I.bottomLeftCorner<3, 3>() = vectorToSkewSymmetric((-mass * com).eval());
  }

  VectorXd q = VectorXd::Random(model->num_positions);
  VectorXd v = VectorXd::Random(model->num_velocities);
  model->setUseNewKinsol(true);
  model->doKinematicsNew(q, v, true, true);


  auto points = Matrix<double, 3, Eigen::Dynamic>::Random(3, 5).eval();
  int body_or_frame_ind = 8;
  int base_or_frame_ind = 0;
  int rotation_type = 0;
  model->forwardJacDotTimesV(points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);

  auto M = model->massMatrix<double>(gradient_order);
  cout << M.value() << endl << endl;

  map<int, unique_ptr<GradientVar<double, TWIST_SIZE, 1>> > f_ext;
  f_ext[3] = unique_ptr< GradientVar<double, TWIST_SIZE, 1> >(new GradientVar<double, TWIST_SIZE, 1>(TWIST_SIZE, 1, nq + nv, gradient_order));
  f_ext[3]->value().setRandom();
  f_ext[3]->gradient().value().setRandom();

  GradientVar<double, Eigen::Dynamic, 1> vd(model->num_velocities, 1, nq + nv, gradient_order);
  vd.value().setRandom();
  if (vd.hasGradient())
    vd.gradient().value().setRandom();

  auto C = model->inverseDynamics(f_ext, &vd, gradient_order);
  cout << C.value() << endl << endl;
  return 0;
}
