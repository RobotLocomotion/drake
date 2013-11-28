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
  if(nrhs != 6 && nrhs != 7)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrSingleTimeLinearPostureConstraintmex:BadInputs","Usage ptr = constructPtrSingleTimeLinearPostureConstraintmex(robot.mex_model_ptr,iAfun,jAvar,A,lb,ub,tspan");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs<=6)
  {
    tspan<<-mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[6],tspan);
  }
  if(!mxIsNumeric(prhs[1]) || !mxIsNumeric(prhs[2]) || !mxIsNumeric(prhs[3]))
  {
    mexErrMsgIdAndTxt("Drake:constructPtrSingleTimeLinearPostureConstraintmex:BadInputs","iAfun, jAvar and A must be numeric");
  }
  int lenA = mxGetM(prhs[1]);
  if(mxGetM(prhs[2]) != lenA || mxGetM(prhs[3]) != lenA || mxGetN(prhs[1]) != 1 || mxGetN(prhs[2]) != 1 || mxGetN(prhs[3]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrSingleTimeLinearPostureConstraintmex:BadInputs","iAfun,jAvar,A must be column vectors of the same size");
  }
  VectorXi iAfun(lenA);
  VectorXi jAvar(lenA);
  VectorXd A(lenA);
  for(int i = 0;i<lenA;i++)
  {
    iAfun(i) = (int) *(mxGetPr(prhs[1])+i)-1;
    jAvar(i) = (int) *(mxGetPr(prhs[2])+i)-1;
    A(i) = *(mxGetPr(prhs[3])+i);
  }
  if(!mxIsNumeric(prhs[4]) || !mxIsNumeric(prhs[5]) || mxGetM(prhs[4]) != mxGetM(prhs[5]) || mxGetN(prhs[4]) != 1 || mxGetN(prhs[5]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrSingleTimeLinearPostureConstraintmex:BadInputs","lb and ub must be numeric column vectors of the same size");
  }
  int num_constraint = mxGetM(prhs[4]);
  VectorXd lb(num_constraint);
  VectorXd ub(num_constraint);
  memcpy(lb.data(),mxGetPr(prhs[4]),sizeof(double)*num_constraint);
  memcpy(ub.data(),mxGetPr(prhs[5]),sizeof(double)*num_constraint);
  SingleTimeLinearPostureConstraint* cnst = new SingleTimeLinearPostureConstraint(model,iAfun,jAvar,A,lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteRigidBodyConstraintmex","SingleTimeKinematicConstraint");
}
