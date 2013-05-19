#include "mex.h"
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
    mexErrMsgIdAndTxt("Drake:bodyKinmex:NotEnoughInputs","Usage x=bodyKinmex(model_ptr,q_cache,body_ind,pts)");
  }

  RigidBodyManipulator *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:bodyKinmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));

  double* q = mxGetPr(prhs[1]);
  for (int i = 0; i < model->num_dof; i++) {
    if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:bodyKinmex:InvalidKinematics","This kinsol is no longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }
  
  int body_ind = ((int) mxGetScalar(prhs[2])) - 1;  // note: this is body_ind-1 (so 0 to num_bodies-1)

  if (body_ind<0 || body_ind>=model->num_bodies) {
      mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","body_ind must be -1 (for com) or between 0 and num_bodies-1");
  }

  int n_pts = mxGetN(prhs[3]);
  int dim = mxGetM(prhs[3]);
  
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:bodyKinmex:BadInputs", "number of rows in pts must be 3");
  
  Map<MatrixXd> pts_tmp(mxGetPr(prhs[3]),dim,n_pts);
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);

  if (nlhs>0) {
  	plhs[0] = mxCreateDoubleMatrix(dim,n_pts,mxREAL);
  	if (n_pts>0) {
  		Map<MatrixXd> x(mxGetPr(plhs[0]),dim,n_pts);
  		model->bodyKin(body_ind,pts,x);
  	}
  }
}
