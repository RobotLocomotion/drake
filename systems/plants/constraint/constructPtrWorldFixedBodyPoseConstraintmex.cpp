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

void mexFunction(int nlhs,mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 3 && nrhs != 2)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldFixedBodyPoseConstraintmex:BadInputs", "Usage ptr = constructPtrWorldFixedBodyPoseConstraintmex(obj.robot.mex_model_ptr,body,tspan)");
  }
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldFixedBodyPoseConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 2)
  {
    tspan<< -mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[2],tspan);
  }
  int body = (int) mxGetScalar(prhs[1])-1;

  cnst = new WorldFixedBodyPoseConstraint(model,body,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteConstraintmex","WorldFixedBodyPoseConstraint");
}
