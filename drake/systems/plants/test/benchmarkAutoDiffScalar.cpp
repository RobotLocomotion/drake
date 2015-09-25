#include "RigidBodyManipulator.h"
#include "testUtil.h"

using namespace std;
using namespace Eigen;

default_random_engine generator;

void testUserGradients(const RigidBodyManipulator &model, int ntests) {
  VectorXd q(model.num_positions);
  KinematicsCache<double> cache(model.bodies, 1);

  for (int i = 0; i < ntests; i++) {
    model.getRandomConfiguration(q, generator);
    cache.initialize(q);
    model.doKinematics(cache, false);
    auto H_gradientvar = model.massMatrix(cache, 1);
  }
}

void testAutoDiffScalarGradients(const RigidBodyManipulator &model, int ntests) {
  const int NUM_POSITIONS = Dynamic;
  typedef Matrix<double, NUM_POSITIONS, 1> DerivativeType;
  typedef AutoDiffScalar<DerivativeType> TaylorVar;
  VectorXd q(model.num_positions);
  Matrix<TaylorVar, NUM_POSITIONS, 1> q_taylorvar;
  auto grad = Matrix<double, NUM_POSITIONS, NUM_POSITIONS>::Identity(model.num_positions, model.num_positions).eval();
  KinematicsCache<TaylorVar> cache(model.bodies, 0);

  for (int i = 0; i < ntests; i++) {
    model.getRandomConfiguration(q, generator);
    q_taylorvar = q.cast<TaylorVar>().eval();
    gradientMatrixToAutoDiff(grad, q_taylorvar);
    cache.initialize(q_taylorvar);
    model.doKinematics(cache, false);
    auto H_taylorvar = model.massMatrix(cache, 0);
  }
}

int main() {
  RigidBodyManipulator model("examples/Atlas/urdf/atlas_minimal_contact.urdf");

  int ntests = 1000;
  std::cout << "user gradients: " << measure<>::execution(testUserGradients, model, ntests) / static_cast<double>(ntests) << std::endl;
  std::cout << "autodiff gradients: " << measure<>::execution(testAutoDiffScalarGradients, model, ntests) / static_cast<double>(ntests) << std::endl;

  return 0;
}