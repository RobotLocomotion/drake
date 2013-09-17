/*
 * =====================================================================================
 *
 *       Filename:  constraintTypemex.cpp
 *
 *    Description:  return the type of the constraint, KinematicConstraint, PostureConstraint or QuasiStaticConstraint 
 *
 *        Version:  1.0
 *        Created:  08/20/2013 09:32:04 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "Constraint.h"
#include "RigidBodyManipulator.h"
#include <cstdio>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs!= 1)
  {
    mexErrMsgIdAndTxt("Drake:constraintTypemex:BadInputs","Usage constraint_type = constraintTypemex(constraint_ptr)");
  }
  Constraint* cnst = (Constraint*) getDrakeMexPointer(prhs[0]);
  DrakeConstraintType type = cnst->getType();
  switch(type)
  {
    case DrakeConstraintType::KinematicConstraintType:
      plhs[0] = mxCreateString("KinematicConstraint");
      break;
    case DrakeConstraintType::QuasiStaticConstraintType:
      plhs[0] = mxCreateString("QuasiStaticConstraint");
      break;
    case DrakeConstraintType::PostureConstraintType:
      plhs[0] = mxCreateString("PostureConstraint");
      break;
  }
}

