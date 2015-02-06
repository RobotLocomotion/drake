#include "RigidBodyManipulator.h"
#include "URDFRigidBodyManipulator.h"
#include "drakeGeometryUtil.h"
#include <iostream>
#include <cstdlib>
#include <random>

using namespace std;
using namespace Eigen;
int main()
{
  URDFRigidBodyManipulator* model = loadURDFfromFile("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  int gradient_order = 0;
  int nq = model->num_dof;
  int nv = model->num_velocities;

  std::default_random_engine generator;
  std::uniform_real_distribution<double> uniform;

  for (std::vector<std::unique_ptr<RigidBody> >::const_iterator it = model->bodies.begin(); it != model->bodies.end(); ++it) {
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

  VectorXd q = VectorXd::Random(model->num_dof);
  VectorXd v = VectorXd::Random(model->num_velocities);
  model->doKinematicsNew(q.data(), true, v.data(), true);

  auto M = model->massMatrix<double>(gradient_order);
  std::cout << M.value() << std::endl << std::endl;


  GradientVar<double, TWIST_SIZE, Eigen::Dynamic> f_ext(TWIST_SIZE, model->num_bodies, nq + nv, gradient_order);
  f_ext.value().setRandom();
  if (f_ext.hasGradient())
    f_ext.gradient().value().setRandom();

  GradientVar<double, Eigen::Dynamic, 1> vd(model->num_velocities, 1, nq + nv, gradient_order);
  vd.value().setRandom();
  if (vd.hasGradient())
    vd.gradient().value().setRandom();

  auto C = model->inverseDynamics(f_ext, vd);
  std::cout << C.value() << std::endl << std::endl;
  return 0;
}
