#include "mex.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{

  RigidBodyManipulator model("../../examples/Atlas/urdf/robotiq.urdf");
  for(int x=0;x<model.bodies.size();x++)
    cout << model.bodies[x]->linkname << endl;
  VectorXd q = VectorXd::Zero(model.num_positions);
  VectorXd qd = VectorXd::Zero(model.num_velocities);

  cout << "dof: " << model.num_positions << endl;
  model.doKinematicsNew(q, qd, false, false);
  
  auto positionConstraints = model.positionConstraints<double>(1);
  cout << "0 order: " << positionConstraints.value().rows() << "x" << positionConstraints.value().cols() << endl;
  cout << positionConstraints.value() << endl;

  cout << "1 order" << endl;
  cout << positionConstraints.gradient().value() << endl;
  cout << positionConstraints.gradient().value().rows() << "x" << positionConstraints.gradient().value().cols() << endl;
}