#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "RigidBodyConstraint.h"
#include "RigidBodyManipulator.h"
#include "constructPtrDrakeConstraint.h"
#include <cstdio>

using namespace Eigen;
using namespace std;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs!= 5 && nrhs!=4)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldEulerConstraintmex:BadInputs","Usage ptr = constructPtrWorldEulerConstraintmex(obj.robot.mex_model_ptr,body,lb,ub,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldEulerConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 4)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[4],tspan);
  }
  int body = (int) mxGetScalar(prhs[1])-1;

  if(mxGetM(prhs[2]) != 3 || mxGetM(prhs[3]) != 3 || mxGetN(prhs[2]) != 1 || mxGetN(prhs[3]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldEulerConstraintmex:BadInputs","lb and ub should both be 3x1 double vectors");
  }
  Vector3d lb;
  Vector3d ub;
  memcpy(lb.data(),mxGetPr(prhs[2]),sizeof(double)*3);
  memcpy(ub.data(),mxGetPr(prhs[3]),sizeof(double)*3);
  cnst = new WorldEulerConstraint(model,body,lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteRigidBodyConstraintmex","WorldEulerConstraint");
}
