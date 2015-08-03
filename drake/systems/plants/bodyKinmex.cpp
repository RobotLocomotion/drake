#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the bodyKin function
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 3) {
    mexErrMsgIdAndTxt("Drake:bodyKinmex:NotEnoughInputs","Usage [x,J,P]=bodyKinmex(model_ptr,q_cache,body_ind,pts)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double* q = mxGetPrSafe(prhs[1]);
  for (int i = 0; i < model->num_positions; i++) {
    if (q[i] - model->cached_q_old[i] > 1e-8 || q[i] - model->cached_q_old[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:bodyKinmex:InvalidKinematics","This kinsol is no longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }
  
  int body_ind = ((int) mxGetScalar(prhs[2])) - 1;  // note: this is body_ind-1 (so 0 to num_bodies-1)

  if (body_ind<-(model->num_frames+1) || body_ind>=model->num_bodies) {
      mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","body_ind must be -1 (for com) or between -num_frames-1 and num_bodies-1");
  }

  int n_pts = static_cast<int>(mxGetN(prhs[3]));
  int dim = static_cast<int>(mxGetM(prhs[3]));
  
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:bodyKinmex:BadInputs", "number of rows in pts must be 3");
  
  Map<MatrixXd> pts_tmp(mxGetPrSafe(prhs[3]),dim,n_pts);
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);
  
  plhs[0] = mxCreateDoubleMatrix(dim,n_pts,mxREAL);
  Map<MatrixXd> x(mxGetPrSafe(plhs[0]),dim,n_pts);

  Map<MatrixXd> *J=NULL, *P=NULL;

  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(dim*n_pts,model->num_positions,mxREAL);
    J = new Map<MatrixXd>(mxGetPrSafe(plhs[1]),dim*n_pts,model->num_positions);
  }
  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(dim*n_pts,dim*n_pts,mxREAL);
    P = new Map<MatrixXd>(mxGetPrSafe(plhs[2]),dim*n_pts,dim*n_pts);
  }

  model->bodyKin(body_ind,pts,x,J,P);

  if (nlhs>1) delete J;
  if (nlhs>2) delete P;

}
