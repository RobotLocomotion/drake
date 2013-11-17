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
  if(nrhs!= 7 && nrhs != 6)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldPositionConstraintmex:BadInputs","Usage ptr = constructPtrWorldPositionConstraintmex(obj.robot.mex_model_ptr,body,pts,lb,ub,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldPositionInFrameConstraint* cnst = NULL;
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

  if(!mxIsNumeric(prhs[3]) || mxGetM(prhs[3]) != 4 || mxGetN(prhs[3]) != 4)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldPositionConstraintmex:BadInputs","Argument 4 should be a 4x4 double matrix");
  }
  Matrix4d T_world_to_frame;
  memcpy(T_world_to_frame.data(),mxGetPr(prhs[3]),sizeof(double)*16);

  MatrixXd lb(3,n_pts);
  MatrixXd ub(3,n_pts);
  if(mxGetM(prhs[4]) != 3 || mxGetN(prhs[4]) != n_pts || mxGetM(prhs[5]) != 3 || mxGetN(prhs[5]) != n_pts)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldPositionConstraintmex:BadInputs","lb and ub should both be 3xn_pts double matrix");
  }
  memcpy(lb.data(),mxGetPr(prhs[4]),sizeof(double)*3*n_pts);
  memcpy(ub.data(),mxGetPr(prhs[5]),sizeof(double)*3*n_pts);

  cnst = new WorldPositionInFrameConstraint(model,body,pts,T_world_to_frame,
                                            lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteRigidBodyConstraintmex","WorldPositionConstraint");
}
