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
    mexErrMsgIdAndTxt("Drake:constructPtrWorldOrientConstraintmex:BadInputs","Usage ptr = constructPtrWorldOrientConstraintmex(obj.robot.mex_model_ptr,body,quat_des,tol,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldQuatConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 4)
  {
    tspan<<-mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[4],tspan);
  }
  int body = (int) mxGetScalar(prhs[1])-1;
  Vector4d quat_des;
  drakeKinCnstParseQuat(prhs[2],quat_des);
  double tol = mxGetScalar(prhs[3]);
  if(tol<0.0||tol>1.0)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldQuatConstraintmex:BadInputs","tol must be within [0 1]");
  }
  cnst = new WorldQuatConstraint(model,body,quat_des,tol,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteRigidBodyConstraintmex","WorldQuatConstraint");
}
  
  
