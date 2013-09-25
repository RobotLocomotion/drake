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
  if(nrhs != 4 && nrhs != 3)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldFixedPositionConstraintmex:BadInputs", "Usage ptr = constructPtrWorldFixedPositionConstraintmex(obj.robot.mex_model_ptr,body,pts,tspan)");
  }
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldFixedPositionConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 3)
  {
    tspan<< -mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[3],tspan);
  }
  int body = (int) mxGetScalar(prhs[1])-1;
  int n_pts = mxGetN(prhs[2]);
  if(!mxIsNumeric(prhs[2])||mxGetM(prhs[2]) != 3)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldFixedPositionConstraintmex:BadInputs","Argument 3 should be a double matrix with 3 rows");
  }
  MatrixXd pts_tmp(3,n_pts);
  memcpy(pts_tmp.data(),mxGetPr(prhs[2]),sizeof(double)*3*n_pts);
  MatrixXd pts(4,n_pts);
  pts.block(0,0,3,n_pts) = pts_tmp;
  pts.block(3,0,1,n_pts) = MatrixXd::Ones(1,n_pts);

  cnst = new WorldFixedPositionConstraint(model,body,pts,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteConstraintmex","WorldFixedPositionConstraint");
}
