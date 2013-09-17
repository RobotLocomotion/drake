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
  if(nrhs != 6&& nrhs != 7)
  {  
    mexErrMsgIdAndTxt("Drake:constructPtrWorldGazeOrientConstraintmex:BadInputs","Usage ptr = constructPtrWorldGazeOrientConstraintmex(obj.robot.mex_model_ptr,body,axis,quat_des,conethreshold,threshold,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldGazeOrientConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 6)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[6],tspan);
  }
  int body = (int) mxGetScalar(prhs[1])-1;
  Vector3d axis;
  drakeKinCnstParse3dUnitVector(prhs[2],axis);
  Vector4d quat_des;
  drakeKinCnstParseQuat(prhs[3],quat_des);
  double conethreshold = drakeKinCnstParseGazeConethreshold(prhs[4]);
  double threshold = drakeKinCnstParseGazeThreshold(prhs[5]);
  cnst = new WorldGazeOrientConstraint(model,body,axis,quat_des,conethreshold,threshold,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst, "deleteConstraintmex","WorldGazeOrientConstraint");
}

