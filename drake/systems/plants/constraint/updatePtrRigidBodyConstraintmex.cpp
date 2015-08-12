#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include <Eigen/Dense>
#include "RigidBodyConstraint.h"
#include "RigidBodyManipulator.h"
#include "constructPtrRigidBodyConstraint.h"
#include <cstdio>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  RigidBodyConstraint* constraint = (RigidBodyConstraint*) getDrakeMexPointer(prhs[0]);
  int constraint_type = constraint->getType();
  if (nrhs<2) { // then it's just calling delete
    destroyDrakeMexPointer<RigidBodyConstraint*>(prhs[0]);
    return;
  }
  mwSize strlen = static_cast<mwSize>(mxGetNumberOfElements(prhs[1]) + 1);
  char* field = new char[strlen];
  mxGetString(prhs[1],field,strlen);
  string field_str(field);
  switch(constraint_type)
  {
    case RigidBodyConstraint::QuasiStaticConstraintType:
      {
        QuasiStaticConstraint* cnst = (QuasiStaticConstraint*) constraint;
        if(field_str=="active")
        {
          // setActive(qsc_ptr,flag)
          if(mxGetNumberOfElements(prhs[2]) != 1)
          {
            mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","QuasiStaticConstraint:flag must be a single boolean");
          }
          bool* flag = mxGetLogicals(prhs[2]);
          QuasiStaticConstraint* cnst_new = new QuasiStaticConstraint(*cnst);
          cnst_new->setActive(*flag);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"QuasiStaticConstraint");
        }
        else if(field_str=="factor")
        {// setShrinkFactor(qsc_ptr,factor)
          if(mxGetNumberOfElements(prhs[2]) != 1)
          {
            mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","QuasiStaticConstraint:shrink factor must be a double scalar");
          }
          double factor = mxGetScalar(prhs[2]);
          if(factor<=0.0)
          {
            mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","QuasiStaticConstraint:shrink factor should be a positive scalar");
          }
          QuasiStaticConstraint* cnst_new = new QuasiStaticConstraint(*cnst);
          cnst_new->setShrinkFactor(factor);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"QuasiStaticConstraint");
        }
        else if(field_str=="contact")
        {
          // addContact(qsc_ptr,body1, body1_pts, body2, body2_pts,...)
          int num_new_bodies = (nrhs-2)/2;
          int* new_bodies = new int[num_new_bodies];
          Matrix3Xd* new_body_pts = new Matrix3Xd[num_new_bodies];
          for(int idx = 0;idx<num_new_bodies;idx++)
          {
            new_bodies[idx] = (int) mxGetScalar(prhs[2+idx*2])-1;
            size_t npts = mxGetN(prhs[3+idx*2]);
            Matrix3Xd new_body_pts_tmp(3,npts);
            memcpy(new_body_pts_tmp.data(),mxGetPrSafe(prhs[3+idx*2]),sizeof(double)*3*npts);
            new_body_pts[idx].resize(3,npts);
            new_body_pts[idx].block(0,0,3,npts) = new_body_pts_tmp;
          }
          QuasiStaticConstraint* cnst_new = new QuasiStaticConstraint(*cnst);
          cnst_new->addContact(num_new_bodies,new_bodies,new_body_pts);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"QuasiStaticConstraint");
          delete[] new_bodies;
          delete[] new_body_pts;
        }
        else if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          QuasiStaticConstraint* cnst_new = new QuasiStaticConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"QuasiStaticConstraint");
        }
        else if(field_str=="robotnum")
        {
          size_t num_robot = mxGetNumberOfElements(prhs[2]);
          double* robotnum_tmp = new double[num_robot];
          int* robotnum = new int[num_robot];
          memcpy(robotnum_tmp,mxGetPrSafe(prhs[2]),sizeof(double)*num_robot);
          for(int i = 0;i<num_robot;i++)
          {
            robotnum[i] = (int) robotnum_tmp[i]-1;
          }
          set<int> robotnumset(robotnum,robotnum+num_robot);
          QuasiStaticConstraint* cnst_new = new QuasiStaticConstraint(*cnst);
          cnst_new->updateRobotnum(robotnumset);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"QuasiStaticConstraint");
          delete[] robotnum_tmp;
          delete[] robotnum;
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","QuasiStaticConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::PostureConstraintType:
      {
        PostureConstraint* pc = (PostureConstraint*) constraint;
        if(field_str=="bounds")
        { // setJointLimits(pc,joint_idx,lb,ub)
          size_t num_idx = mxGetM(prhs[2]);
          if(!mxIsNumeric(prhs[2]) || mxGetN(prhs[2]) != 1 || !mxIsNumeric(prhs[3]) || mxGetM(prhs[3]) != num_idx || mxGetN(prhs[3]) != 1 || !mxIsNumeric(prhs[4]) || mxGetM(prhs[4]) != num_idx || mxGetN(prhs[4]) != 1)
          {
            mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","PostureConstraint:joint_idx, lb and ub must be of the same length numerical vector");
          }
          int* joint_idx = new int[num_idx];
          for(int i = 0;i<num_idx;i++)
          {
            joint_idx[i] = (int) *(mxGetPrSafe(prhs[2])+i)-1;
          }
          VectorXd lb(num_idx),ub(num_idx);
          memcpy(lb.data(),mxGetPrSafe(prhs[3]),sizeof(double)*num_idx);
          memcpy(ub.data(),mxGetPrSafe(prhs[4]),sizeof(double)*num_idx);
          PostureConstraint* pc_new = new PostureConstraint(*pc);
          pc_new->setJointLimits(static_cast<int>(num_idx), joint_idx, lb, ub);
          delete[] joint_idx;
          plhs[0] = createDrakeConstraintMexPointer((void*)pc_new,"PostureConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","PostureConstraint: argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::AllBodiesClosestDistanceConstraintType:
      {
        AllBodiesClosestDistanceConstraint* cnst = (AllBodiesClosestDistanceConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          AllBodiesClosestDistanceConstraint* cnst_new = new AllBodiesClosestDistanceConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"AllBodiesClosestDistanceConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","AllBodiesClosestDistanceConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldEulerConstraintType:
      {
        WorldEulerConstraint* cnst = (WorldEulerConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldEulerConstraint* cnst_new = new WorldEulerConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldEulerConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldEulerConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldGazeDirConstraintType:
      {
        WorldGazeDirConstraint* cnst = (WorldGazeDirConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldGazeDirConstraint* cnst_new = new WorldGazeDirConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldGazeDirConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldGazeDirConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldGazeOrientConstraintType:
      {
        WorldGazeOrientConstraint* cnst = (WorldGazeOrientConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldGazeOrientConstraint* cnst_new = new WorldGazeOrientConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldGazeOrientConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldGazeOrientConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldGazeTargetConstraintType:
      {
        WorldGazeTargetConstraint* cnst = (WorldGazeTargetConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldGazeTargetConstraint* cnst_new = new WorldGazeTargetConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldGazeTargetConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldGazeTargetConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::RelativeGazeTargetConstraintType:
      {
        RelativeGazeTargetConstraint* cnst = (RelativeGazeTargetConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          RelativeGazeTargetConstraint* cnst_new = new RelativeGazeTargetConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"RelativeGazeTargetConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","RelativeGazeTargetConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::RelativeGazeDirConstraintType:
      {
        RelativeGazeDirConstraint* cnst = (RelativeGazeDirConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          RelativeGazeDirConstraint* cnst_new = new RelativeGazeDirConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"RelativeGazeDirConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","RelativeGazeDirConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldCoMConstraintType:
      {
        WorldCoMConstraint* cnst = (WorldCoMConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldCoMConstraint* cnst_new = new WorldCoMConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldCoMConstraint");
        }
        else if(field_str=="robotnum")
        {
          size_t num_robot = mxGetNumberOfElements(prhs[2]);
          double* robotnum_tmp = new double[num_robot];
          int* robotnum = new int[num_robot];
          memcpy(robotnum_tmp,mxGetPrSafe(prhs[2]),sizeof(double)*num_robot);
          for(int i = 0;i<num_robot;i++)
          {
            robotnum[i] = (int) robotnum_tmp[i]-1;
          }
          set<int> robotnumset(robotnum,robotnum+num_robot);
          WorldCoMConstraint* cnst_new = new WorldCoMConstraint(*cnst);
          cnst_new->updateRobotnum(robotnumset);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldCoMConstraint");
          delete[] robotnum_tmp;
          delete[] robotnum;
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldCoMConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldPositionConstraintType:
      {
        WorldPositionConstraint* cnst = (WorldPositionConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldPositionConstraint* cnst_new = new WorldPositionConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"WorldPositionConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldPositionConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldPositionInFrameConstraintType:
      {
        WorldPositionInFrameConstraint* cnst = (WorldPositionInFrameConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldPositionInFrameConstraint* cnst_new = new WorldPositionInFrameConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"WorldPositionInFrameConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldPositionInFrameConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldQuatConstraintType:
      {
        WorldQuatConstraint* cnst = (WorldQuatConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldQuatConstraint* cnst_new = new WorldQuatConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldQuatConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldQuatConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::Point2PointDistanceConstraintType:
      {
        Point2PointDistanceConstraint* cnst = (Point2PointDistanceConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          Point2PointDistanceConstraint* cnst_new = new Point2PointDistanceConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"Point2PointDistanceConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","Point2PointDistanceConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::Point2LineSegDistConstraintType:
      {
        Point2LineSegDistConstraint* cnst = (Point2LineSegDistConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          Point2LineSegDistConstraint* cnst_new = new Point2LineSegDistConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"Point2LineSegDistConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","Point2LineSegDistConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldFixedPositionConstraintType:
      {
        WorldFixedPositionConstraint* cnst = (WorldFixedPositionConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldFixedPositionConstraint* cnst_new = new WorldFixedPositionConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldFixedPositionConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldFixedPositionConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldFixedOrientConstraintType:
      {
        WorldFixedOrientConstraint* cnst = (WorldFixedOrientConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldFixedOrientConstraint* cnst_new = new WorldFixedOrientConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldFixedOrientConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldFixedOrientConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::WorldFixedBodyPoseConstraintType:
      {
        WorldFixedBodyPoseConstraint* cnst = (WorldFixedBodyPoseConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          WorldFixedBodyPoseConstraint* cnst_new = new WorldFixedBodyPoseConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"WorldFixedBodyPoseConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","WorldFixedBodyPoseConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::RelativePositionConstraintType:
      {
        RelativePositionConstraint* cnst = static_cast<RelativePositionConstraint*>(constraint);
        if(field_str == "robot")
        {
            RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
            RelativePositionConstraint* cnst_new = new RelativePositionConstraint(*cnst);
            cnst_new->updateRobot(robot);
            plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"RelativePositionConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","RelativePositionConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::RelativeQuatConstraintType:
      {
        RelativeQuatConstraint* cnst = static_cast<RelativeQuatConstraint*>(constraint);
        if(field_str == "robot")
        {
            RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
            RelativeQuatConstraint* cnst_new = new RelativeQuatConstraint(*cnst);
            cnst_new->updateRobot(robot);
            plhs[0] = createDrakeConstraintMexPointer((void*)cnst_new,"RelativeQuatConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","RelativeQuatConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::MinDistanceConstraintType:
      {
        MinDistanceConstraint* cnst = (MinDistanceConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          MinDistanceConstraint* cnst_new = new MinDistanceConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"MinDistanceConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","MinDistanceConstraint:argument 2 is not accepted");
        }
      }
      break;
    case RigidBodyConstraint::GravityCompensationTorqueConstraintType:
      {
        GravityCompensationTorqueConstraint* cnst = (GravityCompensationTorqueConstraint*) constraint;
        if(field_str=="robot")
        {
          RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
          GravityCompensationTorqueConstraint* cnst_new = new GravityCompensationTorqueConstraint(*cnst);
          cnst_new->updateRobot(robot);
          plhs[0] = createDrakeConstraintMexPointer((void*) cnst_new,"GravityCompensationTorqueConstraint");
        }
        else
        {
          mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","GravityCompensationTorqueConstraint:argument 2 is not accepted");
        }
      }
      break;
    default:
      mexErrMsgIdAndTxt("Drake:updatePtrRigidBodyConstraintmex:BadInputs","Unsupported constraint type");
      break;
  }
  delete[] field;
}
