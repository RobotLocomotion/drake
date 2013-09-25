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
  if(nrhs!= 6 && nrhs != 5)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldPositionConstraintmex:BadInputs","Usage ptr = constructPtrWorldPositionConstraintmex(obj.robot.mex_model_ptr,body,pts,lb,ub,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldPositionConstraint* cnst = NULL;
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
  int n_pts = mxGetN(prhs[2]);
  if(!mxIsNumeric(prhs[2])||mxGetM(prhs[2]) != 3)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldPositionConstraintmex:BadInputs","Argument 3 should be a double matrix with 3 rows");
  }
  MatrixXd pts_tmp(3,n_pts);
  memcpy(pts_tmp.data(),mxGetPr(prhs[2]),sizeof(double)*3*n_pts);
  MatrixXd pts(4,n_pts);
  pts.block(0,0,3,n_pts) = pts_tmp;
  pts.block(3,0,1,n_pts) = MatrixXd::Ones(1,n_pts);

  MatrixXd lb(3,n_pts);
  MatrixXd ub(3,n_pts);
  if(mxGetM(prhs[3]) != 3 || mxGetN(prhs[3]) != n_pts || mxGetM(prhs[4]) != 3 || mxGetN(prhs[4]) != n_pts)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldPositionConstraintmex:BadInputs","lb and ub should both be 3xn_pts double matrix");
  }
  memcpy(lb.data(),mxGetPr(prhs[3]),sizeof(double)*3*n_pts);
  memcpy(ub.data(),mxGetPr(prhs[4]),sizeof(double)*3*n_pts);
  cnst = new WorldPositionConstraint(model,body,pts,lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteConstraintmex","WorldPositionConstraint");
}
