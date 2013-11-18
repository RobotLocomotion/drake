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

void mexFunction(int nlhs,mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  WorldEulerConstraint* cnst = (WorldEulerConstraint*) getDrakeMexPointer(prhs[0]);
  mwSize strlen = mxGetNumberOfElements(prhs[1])+1;
  char* field = new char[strlen];
  mxGetString(prhs[1],field,strlen);
  string field_str(field);
  if(field_str=="robot")
  {
    RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
    WorldEulerConstraint* cnst_new = new WorldEulerConstraint(*cnst);
    cnst_new->updateRobot(robot);
    plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"deleteRigidBodyConstraintmex","WorldEulerConstraint");
  }
  else
  {
    mexErrMsgIdAndTxt("Drake:updatePtrWorldEulerConstraintmex:BadInputs","argument 2 is not accepted");
  }
  delete[] field;
}
