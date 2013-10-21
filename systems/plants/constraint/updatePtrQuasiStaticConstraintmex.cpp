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
  QuasiStaticConstraint* cnst = (QuasiStaticConstraint*) getDrakeMexPointer(prhs[0]);
  mwSize strlen = mxGetNumberOfElements(prhs[1])+1;
  char* field = new char[strlen];
  mxGetString(prhs[1],field,strlen);
  string field_str(field);
  if(field_str=="active")
  {
    // setActive(qsc_ptr,flag)
    if(mxGetNumberOfElements(prhs[2]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrQuasiStaticConstraintmex:BadInputs","flag must be a single boolean");
    }
    bool* flag = mxGetLogicals(prhs[2]);
    cnst->setActive(*flag);
  }
  else if(field_str=="factor")
  {// setShrinkFactor(qsc_ptr,factor)
    if(mxGetNumberOfElements(prhs[2]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrQuasiStaticConstraintmex:BadInputs","shrink factor must be a double scalar");
    }
    double factor = mxGetScalar(prhs[2]);
    if(factor<=0.0)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrQuasiStaticConstraintmex:BadInputs","shrink factor should be a positive scalar");
    }
    cnst->setShrinkFactor(factor);
  }
  else if(field_str=="contact")
  {
    // addContact(qsc_ptr,body1, body1_pts, body2, body2_pts,...)
    int num_new_bodies = (nrhs-2)/2;
    int* new_bodies = new int[num_new_bodies];
    MatrixXd* new_body_pts = new MatrixXd[num_new_bodies];
    for(int idx = 0;idx<num_new_bodies;idx++)
    {
      new_bodies[idx] = (int) mxGetScalar(prhs[2+idx*2])-1;
      int npts = mxGetN(prhs[3+idx*2]);
      MatrixXd new_body_pts_tmp(3,npts);
      memcpy(new_body_pts_tmp.data(),mxGetPr(prhs[3+idx*2]),sizeof(double)*3*npts);
      new_body_pts[idx].resize(4,npts);
      new_body_pts[idx].block(0,0,3,npts) = new_body_pts_tmp;
      new_body_pts[idx].row(3) = MatrixXd::Ones(1,npts);
    }
    cnst->addContact(num_new_bodies,new_bodies,new_body_pts);
    delete[] new_bodies;
    delete[] new_body_pts;
  }
  else if(field_str=="robot")
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
    mexErrMsgIdAndTxt("Drake:updatePtrQuasiStaticConstraintmex:BadInputs","argument 2 is not accepted");
  }
}
