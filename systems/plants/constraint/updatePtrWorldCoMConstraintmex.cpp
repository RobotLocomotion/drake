#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "Constraint.h"
#include "RigidBodyManipulator.h"
#include <cstdio>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs,mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  WorldCoMConstraint* cnst = (WorldCoMConstraint*) getDrakeMexPointer(prhs[0]);
  mwSize strlen = mxGetNumberOfElements(prhs[1])+1;
  char* field = new char[strlen];
  mxGetString(prhs[1],field,strlen);
  string field_str(field);
  if(field_str=="robot")
  {
    RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
    cnst->updateRobot(robot);
  }
  else if(field_str=="robotnum")
  {
    int num_robot = mxGetNumberOfElements(prhs[2]);
    double* robotnum_tmp = new double[num_robot];
    int* robotnum = new int[num_robot];
    memcpy(robotnum_tmp,mxGetPr(prhs[2]),sizeof(double)*num_robot);
    for(int i = 0;i<num_robot;i++)
    {
      robotnum[i] = (int) robotnum_tmp[i]-1;
    }
    set<int> robotnumset(robotnum,robotnum+num_robot);
    cnst->updateRobotnum(robotnumset);
    delete[] robotnum_tmp;
    delete[] robotnum;
  }
  else
  {
    mexErrMsgIdAndTxt("Drake:updatePtrWorldCoMConstraintmex:BadInputs","argument 2 is not accepted");
  }
  delete[] field;
}
