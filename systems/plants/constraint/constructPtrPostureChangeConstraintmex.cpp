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
  if(nrhs != 5 && nrhs != 4)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPostureChangeConstraintmex:BadInputs","Usage ptr = constructPtrPostureChangeConstraintmex(robot.mex_model_ptr,joint_ind,lb_change,ub_change,tspan");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs < 4)
  {
    tspan<<-mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[4],tspan);
  }
  if(!mxIsNumeric(prhs[1]) || mxGetN(prhs[1]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPostureChangeConstraintmex:BadInputs","joint_ind must be a column numeric vector");
  }
  int num_joints = mxGetM(prhs[1]);
  VectorXd joint_ind_tmp(num_joints);
  memcpy(joint_ind_tmp.data(),mxGetPr(prhs[1]),sizeof(double)*num_joints);
  VectorXi joint_ind(num_joints);
  for(int i = 0;i<num_joints;i++)
  {
    joint_ind(i) = (int) joint_ind_tmp(i)-1;
    if(joint_ind(i)<0 || joint_ind(i)>=model->num_dof)
    {
      mexErrMsgIdAndTxt("Drake:constructPtrPostureChangeConstraintmex:BadInputs","joint_ind must be within [1,nq]");
    }
  }
  if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != num_joints || mxGetN(prhs[2]) != 1 ||!mxIsNumeric(prhs[3]) || mxGetM(prhs[3]) != num_joints || mxGetN(prhs[3]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPostureChangeConstraintmex:BadInputs","lb_change and upper bound change must be both numeric vector, with the same size as joint_ind");
  }
  VectorXd lb_change(num_joints);
  VectorXd ub_change(num_joints);
  memcpy(lb_change.data(),mxGetPr(prhs[2]),sizeof(double)*num_joints);
  memcpy(ub_change.data(),mxGetPr(prhs[3]),sizeof(double)*num_joints);
  for(int i = 0;i<num_joints;i++)
  {
    double lb_change_min = model->joint_limit_min[joint_ind(i)]-model->joint_limit_max[joint_ind(i)];
    double ub_change_max = model->joint_limit_max[joint_ind(i)]-model->joint_limit_min[joint_ind(i)];
    lb_change(i) = (lb_change_min<lb_change(i)?lb_change(i):lb_change_min);
    ub_change(i) = (ub_change_max>ub_change(i)?ub_change(i):ub_change_max);
    if(lb_change(i)>ub_change(i))
    {
      mexErrMsgIdAndTxt("Drake:constructPtrPostureChangeConstraintmex:BadInputs","lb_change must be no larger than ub_change");
    }
  }
  PostureChangeConstraint* cnst = new PostureChangeConstraint(model,joint_ind,lb_change,ub_change,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteRigidBodyConstraintmex","PostureChangeConstraint");
}
