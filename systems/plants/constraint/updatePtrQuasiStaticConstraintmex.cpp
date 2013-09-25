#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "Constraint.h"
#include "RigidBodyManipulator.h"
#include <cstdio>

void mexFunction(int nlhs,mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  QuasiStaticConstraint* cnst = (QuasiStaticConstraint*) getDrakeMexPointer(prhs[0]);
  if(nrhs == 2)
  {
    if(mxIsLogical(prhs[1]))
    {
      // setActive(qsc_ptr,flag)
      if(mxGetNumberOfElements(prhs[1]) != 1)
      {
        mexErrMsgIdAndTxt("Drake:updatePtrQuasiStaticConstraintmex:BadInputs","flag must be a single boolean");
      }
      bool* flag = mxGetLogicals(prhs[1]);
      cnst->setActive(*flag);
    }
    else if(mxIsNumeric(prhs[1]))
    {// setShrinkFactor(qsc_ptr,factor)
      if(mxGetNumberOfElements(prhs[1]) != 1)
      {
        mexErrMsgIdAndTxt("Drake:updatePtrQuasiStaticConstraintmex:BadInputs","shrink factor must be a double scalar");
      }
      double factor = mxGetScalar(prhs[1]);
      if(factor<=0.0)
      {
        mexErrMsgIdAndTxt("Drake:updatePtrQuasiStaticConstraintmex:BadInputs","shrink factor should be a positive scalar");
      }
      cnst->setShrinkFactor(factor);
    }
  }
  else
  {
    // addContact(qsc_ptr,body1, body1_pts, body2, body2_pts,...)
    int num_new_bodies = (nrhs-1)/2;
    int* new_bodies = new int[num_new_bodies];
    MatrixXd* new_body_pts = new MatrixXd[num_new_bodies];
    for(int idx = 0;idx<num_new_bodies;idx++)
    {
      new_bodies[idx] = (int) mxGetScalar(prhs[1+idx*2])-1;
      int npts = mxGetN(prhs[2+idx*2]);
      MatrixXd new_body_pts_tmp(3,npts);
      memcpy(new_body_pts_tmp.data(),mxGetPr(prhs[2+idx*2]),sizeof(double)*3*npts);
      new_body_pts[idx].resize(4,npts);
      new_body_pts[idx].block(0,0,3,npts) = new_body_pts_tmp;
      new_body_pts[idx].row(3) = MatrixXd::Ones(1,npts);
    }
    cnst->addContact(num_new_bodies,new_bodies,new_body_pts);
    delete[] new_bodies;
    delete[] new_body_pts;
  }
}
