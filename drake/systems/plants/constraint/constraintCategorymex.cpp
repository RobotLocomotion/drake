/*
 * return the category of the RigidBodyConstraint. Can be SingleTimeKinematicConstraint, MultipleTimeKinematicConstraint, QuasiStaticConstraint, PostureConstraint, SingleTimeLinearPostureConstraint and MultipleTimeLinearPostureConstraint
 */


#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include <Eigen/Dense>
#include "RigidBodyConstraint.h"
#include "RigidBodyManipulator.h"
#include <cstdio>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs!= 1)
  {
    mexErrMsgIdAndTxt("Drake:constraintCategorymex:BadInputs","Usage constraint_category = constraintCategorymex(constraint_ptr)");
  }
  RigidBodyConstraint* cnst = (RigidBodyConstraint*) getDrakeMexPointer(prhs[0]);
  int category = cnst->getCategory();
  switch(category)
  {
    case RigidBodyConstraint::SingleTimeKinematicConstraintCategory:
      plhs[0] = mxCreateString("SingleTimeKinematicConstraint");
      break;
    case RigidBodyConstraint::MultipleTimeKinematicConstraintCategory:
      plhs[0] = mxCreateString("MultipleTimeKinematicConstraint");
      break;
    case RigidBodyConstraint::QuasiStaticConstraintCategory:
      plhs[0] = mxCreateString("QuasiStaticConstraint");
      break;
    case RigidBodyConstraint::PostureConstraintCategory:
      plhs[0] = mxCreateString("PostureConstraint");
      break;
    case RigidBodyConstraint::MultipleTimeLinearPostureConstraintCategory:
      plhs[0] = mxCreateString("MultipleTimeLinearPostureConstraint");
      break;
    case RigidBodyConstraint::SingleTimeLinearPostureConstraintCategory:
      plhs[0] = mxCreateString("SingleTimeLinearPostureConstraint");
      break;
    default:
      mexErrMsgIdAndTxt("Drake:constraintCategorymex:BadInputs","The constraint category is not supported");
      break;
  }
}

