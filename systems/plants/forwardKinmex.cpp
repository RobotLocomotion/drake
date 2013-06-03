#include <mex.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the forwardKin function
 *
 * Piggybacks on HandCpmex.cpp to properly initialize and destroy models
 * Call with doKinematicsmex(q,b_compute_second_derivatives);
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 3) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs","Usage forwardKinmex(model_ptr,q_cache,0) for center of mass, or forwardKinmex(model_pts,q_cache,body_ind,pts,rotation_type,b_jacdot)");
  }

  RigidBodyManipulator *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));

  double* q = mxGetPr(prhs[1]);
  for (int i = 0; i < model->num_dof; i++) {
    if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:forwardKinmex:InvalidKinematics","This kinsol is no longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }
  
  int body_ind = ((int) mxGetScalar(prhs[2])) - 1;  // note: this is body_ind-1 (so 0 to num_bodies-1)
  bool b_jacdot = nrhs>5 && (bool) mxGetScalar(prhs[5]);

  if (body_ind==-1) {  // compute center of mass
    if (b_jacdot) {
      plhs[0] = mxCreateDoubleMatrix(3,model->num_dof,mxREAL);
      Map<MatrixXd> Jdot(mxGetPr(plhs[0]),3,model->num_dof);
      model->getCOMJacDot(Jdot);
      return;
    }
    
    if (nlhs>0) {
      plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
      Map<Vector3d> x(mxGetPr(plhs[0]));
      model->getCOM(x);
    }
    if (nlhs>1) {
      plhs[1] = mxCreateDoubleMatrix(3,model->num_dof,mxREAL);
      Map<MatrixXd> J(mxGetPr(plhs[1]),3,model->num_dof);
      model->getCOMJac(J);
    }
  
    if (nlhs>2) {
      plhs[2] = mxCreateDoubleMatrix(3,model->num_dof*model->num_dof,mxREAL);
      Map<MatrixXd> dJ(mxGetPr(plhs[2]),3,model->num_dof*model->num_dof);
      model->getCOMdJac(dJ);
    }
    
    return;
  } else if (body_ind<0 || body_ind>=model->num_bodies) {
      mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","body_ind must be -1 (for com) or between 0 and num_bodies-1");
  }

  if (nrhs != 5 &&nrhs !=6) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs", "Usage forwardKinmex(model_ptr,q_cache,body_index,pts,rotation_type,b_jacdot)");
  }

  int n_pts = mxGetN(prhs[3]);
  int dim = mxGetM(prhs[3]), dim_with_rot=dim;
  
//  if (dim != 2 && dim != 3)
//    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 2 or 3");
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 3");
  
  int rotation_type = (int)mxGetScalar(prhs[4]);
  if (rotation_type==1) dim_with_rot += 3;
  else if (rotation_type==2) dim_with_rot += 4;

  Map<MatrixXd> pts_tmp(mxGetPr(prhs[3]),dim,n_pts);
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);

  if (b_jacdot) {
    if (rotation_type>0) mexErrMsgIdAndTxt("Drake:forwardKinmex:NotImplemented","Jacobian dot of rotations are not implemented yet");
    plhs[0] = mxCreateDoubleMatrix(dim*n_pts,model->num_dof,mxREAL);  
    Map<MatrixXd> Jdot(mxGetPr(plhs[0]),dim*n_pts,model->num_dof);
    model->forwardJacDot(body_ind,pts,Jdot);
    return;
  } else {
    if (nlhs>0) {
      plhs[0] = mxCreateDoubleMatrix(dim_with_rot,n_pts,mxREAL);
      Map<MatrixXd> x(mxGetPr(plhs[0]),dim_with_rot,n_pts);
      model->forwardKin(body_ind,pts,rotation_type,x);
    }
    if (nlhs>1) {
      plhs[1] = mxCreateDoubleMatrix(dim_with_rot*n_pts,model->num_dof,mxREAL);
      Map<MatrixXd> J(mxGetPr(plhs[1]),dim_with_rot*n_pts,model->num_dof);
      model->forwardJac(body_ind,pts,rotation_type,J);
    }
    
    if (nlhs>2) {
      if (rotation_type>0) mexErrMsgIdAndTxt("Drake:forwardKinmex:NotImplemented","Second derivatives of rotations are not implemented yet");
      plhs[2] = mxCreateDoubleMatrix(dim*n_pts,model->num_dof*model->num_dof,mxREAL);
      Map<MatrixXd> dJ(mxGetPr(plhs[2]),dim*n_pts,model->num_dof*model->num_dof);
      model->forwarddJac(body_ind,pts,dJ);
    }
  }
}
