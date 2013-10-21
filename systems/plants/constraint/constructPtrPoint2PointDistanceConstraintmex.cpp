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
  if(nrhs != 7 && nrhs != 8)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","Usage ptr = constructPtrPoint2PointConstraintmex(obj.robot.mex_model_ptr,bodyA,bodyB,ptA,ptB,lb,ub,tspan");
  }
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs == 7)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[7],tspan);
  }
  if(!mxIsNumeric(prhs[1]) || !mxIsNumeric(prhs[2]))
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","bodyA and bodyB must be numeric");
  }
  if(mxGetNumberOfElements(prhs[1]) != 1 || mxGetNumberOfElements(prhs[2]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","bodyA and bodyB must be a scalar");
  }
  int bodyA = (int) mxGetScalar(prhs[1])-1;
  int bodyB = (int) mxGetScalar(prhs[2])-1;
  if(bodyA>=model->num_bodies || bodyA < -1 || bodyB>= model->num_bodies || bodyB < -1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","bodyA and bodyB must be within [0 robot.getNumBodies]");
  }
  if(bodyA == bodyB)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","bodyA and bodyB should be different");
  }
  if(!mxIsNumeric(prhs[3]) || !mxIsNumeric(prhs[4]))
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","ptA and ptB should be numeric");
  }
  int npts = mxGetN(prhs[3]);
  if(mxGetM(prhs[3]) != 3 || mxGetM(prhs[4]) != 3)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","ptA and ptB should have 3 rows");
  }
  if(mxGetN(prhs[4]) != npts)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","ptA and ptB should have the same number of columns");
  }
  MatrixXd ptA_tmp(3,npts);
  MatrixXd ptB_tmp(3,npts);
  memcpy(ptA_tmp.data(),mxGetPr(prhs[3]),sizeof(double)*3*npts);
  memcpy(ptB_tmp.data(),mxGetPr(prhs[4]),sizeof(double)*3*npts);
  MatrixXd ptA(4,npts);
  MatrixXd ptB(4,npts);
  ptA.block(0,0,3,npts) = ptA_tmp;
  ptB.block(0,0,3,npts) = ptB_tmp;
  ptA.row(3) = MatrixXd::Ones(1,npts);
  ptB.row(3) = MatrixXd::Ones(1,npts);
  if(!mxIsNumeric(prhs[5]) || !mxIsNumeric(prhs[6]) || mxGetM(prhs[5]) != 1 || mxGetM(prhs[6]) != 1 || mxGetN(prhs[5]) != npts || mxGetN(prhs[6]) != npts)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","lb and ub must be 1 x npts numeric");
  }
  VectorXd lb(npts);
  VectorXd ub(npts);
  memcpy(lb.data(),mxGetPr(prhs[5]),sizeof(double)*npts);
  memcpy(ub.data(),mxGetPr(prhs[6]),sizeof(double)*npts);
  for(int i = 0;i<npts;i++)
  {
    if(lb(i)>ub(i))
    {
      mexErrMsgIdAndTxt("Drake:constructPtrPoint2PointDistanceConstraintmex:BadInputs","lb must be no larger than ub");
    }
  }
  Point2PointDistanceConstraint* cnst = new Point2PointDistanceConstraint(model,bodyA,bodyB,ptA,ptB,lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteConstraintmex","Point2PointDistanceConstraint");
}
