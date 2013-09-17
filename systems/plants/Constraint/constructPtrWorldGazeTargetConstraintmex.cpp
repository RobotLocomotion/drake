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
  if(nrhs != 7&& nrhs != 6)
  {  
    mexErrMsgIdAndTxt("Drake:constructPtrWorldGazeTargetConstraintmex:BadInputs","Usage ptr = constructPtrWorldGazeTargetConstraintmex(obj.robot.mex_model_ptr,body,axis,target,gaze_origin,conethreshold,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldGazeTargetConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 6)
  {
    tspan<< -mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[6],tspan);
  }
  int body = (int) mxGetScalar(prhs[1])-1;
  Vector3d axis;
  drakeKinCnstParse3dUnitVector(prhs[2],axis);
  Vector3d target;
  assert(mxIsNumeric(prhs[3]));
  assert(mxGetM(prhs[3]) == 3 &&mxGetN(prhs[3]) == 1);
  memcpy(target.data(),mxGetPr(prhs[3]),sizeof(double)*3);
  Vector4d gaze_origin;
  Vector3d gaze_origin_tmp;
  assert(mxIsNumeric(prhs[4]));
  assert(mxGetM(prhs[4]) == 3 && mxGetN(prhs[4]) == 1);
  memcpy(gaze_origin_tmp.data(),mxGetPr(prhs[4]),sizeof(double)*3);
  gaze_origin.head(3) = gaze_origin_tmp;
  gaze_origin(3) = 1.0;
  double conethreshold = drakeKinCnstParseGazeConethreshold(prhs[5]);
  cnst = new WorldGazeTargetConstraint(model,body,axis,target,gaze_origin,conethreshold,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst, "deleteConstraintmex","WorldGazeTargetConstraint");
}

