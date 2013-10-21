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
  if(nrhs!= 2 && nrhs != 3)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrQuasiStaticConstraintmex:BadInputs","Usage ptr = constructPtrQuasiStaticConstraintmex(robot_ptr,robotnum,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs == 2)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[2],tspan);
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
  QuasiStaticConstraint* cnst = new QuasiStaticConstraint(model,robotnumset,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*) cnst,"deleteConstraintmex","QuasiStaticConstraint");
  delete[] robotnum_tmp;
  delete[] robotnum;
}
