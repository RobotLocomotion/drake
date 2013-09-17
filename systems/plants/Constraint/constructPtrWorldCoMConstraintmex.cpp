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

string bool2str(bool flag)
{
  string flag_str = flag?"true":"false";
  return flag_str;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 4 && nrhs != 3)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldCoMConstraintmex:BadInputs","Usage ptr = constructPtrWorldCoMConstraintmex(obj.robot.mex_model_ptr,lb,ub,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldCoMConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 3)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[3],tspan);
  }
  int n_pts = 1;
  if(mxGetM(prhs[1]) != 3 || mxGetM(prhs[2]) != 3 || mxGetN(prhs[1]) != n_pts || mxGetN(prhs[2]) != n_pts || !mxIsNumeric(prhs[1]) || !mxIsNumeric(prhs[2]))
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldCoMConstraintmex:BadInputs","lb and ub should both be 3x1 double vectors");
  }
  Vector3d lb;
  Vector3d ub;
  memcpy(lb.data(),mxGetPr(prhs[1]),sizeof(double)*3);
  memcpy(ub.data(),mxGetPr(prhs[2]),sizeof(double)*3);
  cnst = new WorldCoMConstraint(model,lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteConstraintmex","WorldCoMConstraint");
}


