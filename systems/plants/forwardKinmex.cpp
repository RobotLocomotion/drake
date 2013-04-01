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
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs","Usage forwardKinmex(model_ptr,q_cache,0) for center of mass, or forwardKinmex(model_pts,q_cache,body_ind,pts,include_rotations)");
  }

  RigidBodyManipulator *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));

  double* q = mxGetPr(prhs[1]);
  for (int i = 0; i < model->NB; i++) {
    if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:forwardKinmex:InvalidKinematics","This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }
  
  int body_ind = ((int) mxGetScalar(prhs[2])) - 1;  // note: this is body_ind-1 (so 0 to NB)
  if (body_ind==-1) {  // compute center of mass
    if (nlhs>0) {
      Vector3d x = model->getCOM();
      plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
      memcpy(mxGetPr(plhs[0]), x.data(), sizeof(double)*3);
    }
    if (nlhs>1) {
      MatrixXd J = model->getCOMJac();
      plhs[1] = mxCreateDoubleMatrix(3,model->NB,mxREAL);
      memcpy(mxGetPr(plhs[1]), J.data(), sizeof(double)*3*model->NB);    
    }
  
    if (nlhs>2) {
      MatrixXd dJ = model->getCOMJacDot();
      plhs[2] = mxCreateDoubleMatrix(3,model->NB*model->NB,mxREAL);
      memcpy(mxGetPr(plhs[2]), dJ.data(), sizeof(double)*3*model->NB*model->NB);
    }
    
    return;
  } else if (body_ind<0 || body_ind>model->NB) {
      mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","body_ind must be -1 (for com) or between 0 and NB");
  }

  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs", "Usage forwardKinmex(model_ptr,q_cache,body_index,pts,include_rotations)");
  }

  int n_pts = mxGetN(prhs[3]);
  int dim = mxGetM(prhs[3]);
  
//  if (dim != 2 && dim != 3)
//    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 2 or 3");
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 3");
  
  MatrixXd pts_tmp = MatrixXd::Zero(dim,n_pts);  
  memcpy(pts_tmp.data(), mxGetPr(prhs[3]), sizeof(double)*dim*n_pts);
  
  mxLogical *include_rotations = mxGetLogicals(prhs[4]);
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);

  if (nlhs>0) {
    MatrixXd x = model->forwardKin(body_ind,pts,*include_rotations);
    plhs[0] = mxCreateDoubleMatrix(x.rows(),n_pts,mxREAL);
    memcpy(mxGetPr(plhs[0]), x.data(), sizeof(double)*x.rows()*n_pts);
  }
  if (nlhs>1) {
    MatrixXd J = model->forwardJac(body_ind,pts,*include_rotations);
    plhs[1] = mxCreateDoubleMatrix(J.rows(),model->NB,mxREAL);
    memcpy(mxGetPr(plhs[1]), J.data(), sizeof(double)*J.rows()*model->NB);    
  }
  
  if (nlhs>2) {
    MatrixXd dJ = model->forwardJacDot(body_ind,pts,*include_rotations);
    plhs[2] = mxCreateDoubleMatrix(dJ.rows(),model->NB*model->NB,mxREAL);
    memcpy(mxGetPr(plhs[2]), dJ.data(), sizeof(double)*dJ.rows()*model->NB*model->NB);
  }
}
