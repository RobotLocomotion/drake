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
  if(nrhs != 6 && nrhs != 7&& nrhs != 8)
  {  
    mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","Usage ptr = constructPtrRelativeGazeTargetConstraintmex(obj.robot.mex_model_ptr,bodyA,bodyB,axis,target,gaze_origin,conethreshold,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs < 8)
  {
    tspan<<-mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[7],tspan);
  }
  if(!mxIsNumeric(prhs[1]) || !mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[1]) != 1 || mxGetNumberOfElements(prhs[2]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","bodyA and bodyB should be numeric scalars");
  }
  int bodyA_idx = (int) mxGetScalar(prhs[1])-1;
  int bodyB_idx = (int) mxGetScalar(prhs[2])-1;
  if(!mxIsNumeric(prhs[3]) || mxGetM(prhs[3]) != 3 || mxGetN(prhs[3]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","axis should be 3x1 vector");
  }
  Vector3d axis;
  memcpy(axis.data(),mxGetPr(prhs[3]),sizeof(double)*3);
  double axis_norm = axis.norm();
  if(axis_norm<1e-10)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","axis should be a nonzero vector");
  }
  axis = axis/axis_norm;
  if(!mxIsNumeric(prhs[4]) || mxGetM(prhs[4]) != 3 || mxGetN(prhs[4]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","target should be 3x1 vector");
  }
  Vector3d target;
  memcpy(target.data(),mxGetPr(prhs[4]),sizeof(double)*3);
  if(!mxIsNumeric(prhs[5]) || mxGetM(prhs[5]) != 3 || mxGetN(prhs[5]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","gaze_origin should be 3x1 vector");
  }
  Vector4d gaze_origin;
  memcpy(gaze_origin.data(),mxGetPr(prhs[5]),sizeof(double)*3);
  gaze_origin(3) = 1.0;
  double conethreshold;
  if(nrhs<7)
  {
    conethreshold = 0.0;
  }
  else
  {
    if(!mxIsNumeric(prhs[6]) || mxGetNumberOfElements(prhs[6]) != 1)
    {
      if(mxGetNumberOfElements(prhs[6]) == 0)
      {
        conethreshold = 0.0;
      }
      else
      {
        mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","conethreshold should be a double scalar");
      }
    }
    conethreshold = mxGetScalar(prhs[6]);
    if(conethreshold<0)
    {
      mexErrMsgIdAndTxt("Drake:constructPtrRelativeGazeTargetConstraintmex:BadInputs","conethreshold should be nonnegative");
    }
  }
  RelativeGazeTargetConstraint* cnst = new RelativeGazeTargetConstraint(model,bodyA_idx, bodyB_idx,axis,target,gaze_origin,conethreshold,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst, "deleteRigidBodyConstraintmex","RelativeGazeTargetConstraint");
}
