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

string bool2str(bool flag)
{
  string flag_str = flag?"true":"false";
  return flag_str;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 4 && nrhs != 5)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldCoMConstraintmex:BadInputs","Usage ptr = constructPtrWorldCoMConstraintmex(obj.robot.mex_model_ptr,robotnum,lb,ub,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  WorldCoMConstraint* cnst = NULL;
  Vector2d tspan;
  if(nrhs == 4)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[4],tspan);
  }
  int num_robot = mxGetNumberOfElements(prhs[1]);
  double* robotnum_tmp = new double[num_robot];
  int* robotnum = new int[num_robot];
  memcpy(robotnum_tmp,mxGetPr(prhs[1]),sizeof(double)*num_robot);
  for(int i = 0;i<num_robot;i++)
  {
    robotnum[i] = (int) robotnum_tmp[i]-1;
  }
  set<int> robotnumset(robotnum,robotnum+num_robot);
  int n_pts = 1;
  if(mxGetM(prhs[2]) != 3 || mxGetM(prhs[3]) != 3 || mxGetN(prhs[2]) != n_pts || mxGetN(prhs[3]) != n_pts || !mxIsNumeric(prhs[2]) || !mxIsNumeric(prhs[3]))
  {
    mexErrMsgIdAndTxt("Drake:constructPtrWorldCoMConstraintmex:BadInputs","lb and ub should both be 3x1 double vectors");
  }
  Vector3d lb;
  Vector3d ub;
  memcpy(lb.data(),mxGetPr(prhs[2]),sizeof(double)*3);
  memcpy(ub.data(),mxGetPr(prhs[3]),sizeof(double)*3);
  cnst = new WorldCoMConstraint(model,robotnumset,lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteConstraintmex","WorldCoMConstraint");
  delete[] robotnum_tmp;
  delete[] robotnum;
}


