#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "Constraint.h"
#include "RigidBodyManipulator.h"
#include <cstdio>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs!= 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrQuasiStaticConstraintmex:BadInputs","Usage ptr = constructPtrQuasiStaticConstraintmex(robot_ptr)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  QuasiStaticConstraint* cnst = new QuasiStaticConstraint(model);
  plhs[0] = createDrakeConstraintMexPointer((void*) cnst,"deleteConstraintmex","QuasiStaticConstraint");
}
