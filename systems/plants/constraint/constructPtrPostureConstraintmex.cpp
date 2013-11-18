#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "RigidBodyConstraint.h"
#include "RigidBodyManipulator.h"
#include "constructPtrDrakeConstraint.h"
#include <cstdio>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 1 && nrhs != 2 && nrhs != 4)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPostureConstraintmex:BadInputs","Usage ptr = constructPtrPostureConstraintmex(obj.robot.mex_model_ptr,tspan) or pc_ptr = constructPtrPostureConstraintmex(pc_ptr,joint_idx,lb,ub)");
  }
  if(nrhs == 1 || nrhs == 2)
  { // PostureConstraint(robot,tspan)
    Vector2d tspan;
    if(nrhs == 1)
    {
      tspan<<-mxGetInf(),mxGetInf();
    }
    else
    {
      drakeKinCnstParseTspan(prhs[1],tspan);
    }
    RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
    PostureConstraint* cnst = new PostureConstraint(robot,tspan);
    plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteRigidBodyConstraintmex","PostureConstraint");
  }
  else if(nrhs == 4)
  { // setJointLimits(pc,joint_idx,lb,ub)
    PostureConstraint* pc = (PostureConstraint*) getDrakeMexPointer(prhs[0]);
    int num_idx = mxGetM(prhs[1]);
    if(!mxIsNumeric(prhs[1]) || mxGetN(prhs[1]) != 1 || !mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != num_idx || mxGetN(prhs[2]) != 1 || !mxIsNumeric(prhs[3]) || mxGetM(prhs[3]) != num_idx || mxGetN(prhs[3]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:constructPtrPostureConstraint:BadInputs","joint_idx, lb and ub must be of the same length numerical vector");
    }
    double* joint_idx_tmp = new double[num_idx];
    int* joint_idx = new int[num_idx];
    memcpy(joint_idx_tmp,mxGetPr(prhs[1]),sizeof(double)*num_idx);
    for(int i = 0;i<num_idx;i++)
    {
      joint_idx[i] = (int) joint_idx_tmp[i]-1;
    }
    double* lb = new double[num_idx];
    double* ub = new double[num_idx];
    memcpy(lb,mxGetPr(prhs[2]),sizeof(double)*num_idx);
    memcpy(ub,mxGetPr(prhs[3]),sizeof(double)*num_idx);
    PostureConstraint* pc_new = new PostureConstraint(*pc);
    pc_new->setJointLimits(num_idx,joint_idx,lb,ub);
    delete[] joint_idx_tmp; delete[] joint_idx; delete[] lb; delete[] ub;
    plhs[0] = createDrakeConstraintMexPointer((void*)pc_new,"deleteRigidBodyConstraintmex","PostureConstraint");
  }
}
