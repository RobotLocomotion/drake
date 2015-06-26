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
    mexErrMsgIdAndTxt("Drake:constraintTypemex:BadInputs","Usage constraint_type = constraintTypemex(constraint_ptr)");
  }
  RigidBodyConstraint* cnst = (RigidBodyConstraint*) getDrakeMexPointer(prhs[0]);
  int type = cnst->getType();
  switch(type)
  {
    case RigidBodyConstraint::QuasiStaticConstraintType:
      plhs[0] = mxCreateString("QuasiStaticConstraint");
      break;
    case RigidBodyConstraint::PostureConstraintType:
      plhs[0] = mxCreateString("PostureConstraint");
      break;
    case RigidBodyConstraint::SingleTimeLinearPostureConstraintType:
      plhs[0] = mxCreateString("SingleTimeLinearPostureConstraint");
      break;
    case RigidBodyConstraint::AllBodiesClosestDistanceConstraintType:
      plhs[0] = mxCreateString("AllBodiesClosestDistanceConstraint");
      break;
    case RigidBodyConstraint::WorldEulerConstraintType:
      plhs[0] = mxCreateString("WorldEulerConstraint");
      break;
    case RigidBodyConstraint::WorldGazeDirConstraintType:
      plhs[0] = mxCreateString("WorldGazeDirConstraint");
      break;
    case RigidBodyConstraint::WorldGazeOrientConstraintType:
      plhs[0] = mxCreateString("WorldGazeOrientConstraint");
      break;
    case RigidBodyConstraint::WorldGazeTargetConstraintType:
      plhs[0] = mxCreateString("WorldGazeTargetConstraint");
      break;
    case RigidBodyConstraint::RelativeGazeTargetConstraintType:
      plhs[0] = mxCreateString("RelativeGazeTargetConstraint");
      break;
    case RigidBodyConstraint::RelativeGazeDirConstraintType:
      plhs[0] = mxCreateString("RelativeGazeDirConstraint");
      break;
    case RigidBodyConstraint::WorldCoMConstraintType:
      plhs[0] = mxCreateString("WorldCoMConstraint");
      break;
    case RigidBodyConstraint::WorldPositionConstraintType:
      plhs[0] = mxCreateString("WorldPositionConstraint");
      break;
    case RigidBodyConstraint::WorldPositionInFrameConstraintType:
      plhs[0] = mxCreateString("WorldPositionInFrameConstraint");
      break;
    case RigidBodyConstraint::WorldQuatConstraintType:
      plhs[0] = mxCreateString("WorldQuatConstraint");
      break;
    case RigidBodyConstraint::Point2PointDistanceConstraintType:
      plhs[0] = mxCreateString("Point2PointDistanceConstraint");
      break;
    case RigidBodyConstraint::Point2LineSegDistConstraintType:
      plhs[0] = mxCreateString("Point2LineSegDistConstraint");
      break;
    case RigidBodyConstraint::WorldFixedPositionConstraintType:
      plhs[0] = mxCreateString("WorldFixedPositionConstraint");
      break;
    case RigidBodyConstraint::WorldFixedOrientConstraintType:
      plhs[0] = mxCreateString("WorldFixedOrientConstraint");
      break;
    case RigidBodyConstraint::WorldFixedBodyPoseConstraintType:
      plhs[0] = mxCreateString("WorldFixedBodyPoseConstraint");
      break;
    case RigidBodyConstraint::PostureChangeConstraintType:
      plhs[0] = mxCreateString("PostureChangeConstraint");
      break;
    case RigidBodyConstraint::RelativePositionConstraintType:
      plhs[0] = mxCreateString("RelativePositionConstraint");
      break;
    case RigidBodyConstraint::RelativeQuatConstraintType:
      plhs[0] = mxCreateString("RelativeQuatConstraint");
      break;
    case RigidBodyConstraint::MinDistanceConstraintType:
      plhs[0] = mxCreateString("MinDistanceConstraint");
      break;
    case RigidBodyConstraint::GravityCompensationTorqueConstraintType:
      plhs[0] = mxCreateString("GravityCompensationTorqueConstraint");
      break;
    default:
      mexErrMsgIdAndTxt("Drake:constraintTypemex:BadInputs","The constraint type is not supported");
      break;
  }
}

