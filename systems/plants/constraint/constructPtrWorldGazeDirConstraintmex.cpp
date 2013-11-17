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
  if(nrhs != 6 && nrhs != 5)
  {  
    mexErrMsgIdAndTxt("Drake:constructPtrWorldGazeDirConstraintmex:BadInputs","Usage ptr = constructPtrWorldGazeDirConstraintmex(obj.robot.mex_model_ptr,body,axis,dir,conethreshold,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldGazeDirConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 5)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[5],tspan);
  }
  int body = (int) mxGetScalar(prhs[1])-1;
  Vector3d axis;
  drakeKinCnstParse3dUnitVector(prhs[2], axis);
  Vector3d dir;
  drakeKinCnstParse3dUnitVector(prhs[3],dir);
  double conethreshold = drakeKinCnstParseGazeConethreshold(prhs[4]); 
  cnst = new WorldGazeDirConstraint(model,body,axis,dir,conethreshold,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst, "deleteRigidBodyConstraintmex","WorldGazeDirConstraint");
}

