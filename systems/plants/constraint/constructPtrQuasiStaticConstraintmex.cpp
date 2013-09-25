#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "Constraint.h"
#include "RigidBodyManipulator.h"
#include "constructPtrKinematicConstraint.h"
#include <cstdio>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs!= 1 && nrhs != 2)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrQuasiStaticConstraintmex:BadInputs","Usage ptr = constructPtrQuasiStaticConstraintmex(robot_ptr,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs == 1)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[1],tspan);
  } 
  QuasiStaticConstraint* cnst = new QuasiStaticConstraint(model,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*) cnst,"deleteConstraintmex","QuasiStaticConstraint");
}
