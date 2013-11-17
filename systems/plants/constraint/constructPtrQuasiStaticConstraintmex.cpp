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
  if(nrhs != 1 &&nrhs!= 2 && nrhs != 3)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrQuasiStaticConstraintmex:BadInputs","Usage ptr = constructPtrQuasiStaticConstraintmex(robot_ptr,tspan,robotnum)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  int* robotnum;
  int num_robot;
  if(nrhs <= 2)
  {
    num_robot = 1;
    robotnum = new int[num_robot];
    robotnum[0] = 0;
  }
  else
  {
    num_robot = mxGetNumberOfElements(prhs[2]);
    double* robotnum_tmp = new double[num_robot];
    robotnum = new int[num_robot];
    memcpy(robotnum_tmp,mxGetPr(prhs[2]),sizeof(double)*num_robot);
    for(int i = 0;i<num_robot;i++)
    {
      robotnum[i] = (int) robotnum_tmp[i]-1;
    }
    delete[] robotnum_tmp;
  }
  if(nrhs<=1)
  {
    tspan<<-mxGetInf(),mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[1],tspan);
  }
  set<int> robotnumset(robotnum,robotnum+num_robot);
  QuasiStaticConstraint* cnst = new QuasiStaticConstraint(model,tspan,robotnumset);
  plhs[0] = createDrakeConstraintMexPointer((void*) cnst,"deleteRigidBodyConstraintmex","QuasiStaticConstraint");
  delete[] robotnum;
}
