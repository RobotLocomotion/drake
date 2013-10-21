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
  Point2PointDistanceConstraint* cnst = (Point2PointDistanceConstraint*) getDrakeMexPointer(prhs[0]);
  mwSize strlen = mxGetNumberOfElements(prhs[1])+1;
  char* field = new char[strlen];
  mxGetString(prhs[1],field,strlen);
  string field_str(field);
  if(field_str=="robot")
  {
    RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
    cnst->updateRobot(robot);
  }
  else
  {
    mexErrMsgIdAndTxt("Drake:updatePtrPoint2PointDistanceConstraintmex:BadInputs","argument 2 is not accepted");
  }
  delete[] field;
}
