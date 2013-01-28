#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Model.h"
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

  if (nrhs < 2) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs","Usage forwardKinmex(model_ptr,0) for center of mass, or forwardKinmex(model_pts,body_ind,pts,include_rotations)");
  }

  Model *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));
  
  int body_ind = (int) mxGetScalar(prhs[1]);  // note: this is body_ind-1 from matlab (so 0 to NB)
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

  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs", "Usage forwardKinmex(model_ptr,body_index,pts,include_rotations)");
  }

  int n_pts = mxGetN(prhs[2]);
  int dim = mxGetM(prhs[2]);
  
//  if (dim != 2 && dim != 3)
//    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 2 or 3");
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 3");
  
  MatrixXd pts_tmp = MatrixXd::Zero(dim,n_pts);  
  memcpy(pts_tmp.data(), mxGetPr(prhs[2]), sizeof(double)*dim*n_pts);
  
  mxLogical *include_rotations = mxGetLogicals(prhs[3]);
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
