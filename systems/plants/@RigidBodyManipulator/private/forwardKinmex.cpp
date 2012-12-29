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
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs", "Usage forwardKinmex(obj,body_index,pts)");
  }

  int n_pts = mxGetN(prhs[2]);
  int dim = mxGetM(prhs[2]);
  
//  if (dim != 2 && dim != 3)
//    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 2 or 3");
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 3");
  
  // first get the model_ptr back from matlab
  mxArray* mex_model_ptr = mxGetProperty(prhs[0],0,"mex_model_ptr");
  if (!mex_model_ptr)  mexErrMsgIdAndTxt("Drake:doKinematicspmex:BadInputs","first argument should be the model class object");
  Model *model = NULL; memcpy(&model,mxGetData(mex_model_ptr),sizeof(model));
  
  int body_ind = (int) mxGetScalar(prhs[1]);
    
  MatrixXd pts_tmp = MatrixXd::Zero(dim,n_pts);  

  memcpy(pts_tmp.data(), mxGetPr(prhs[2]), sizeof(double)*dim*n_pts);
  
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);

  MatrixXd x = model->bodies[body_ind].T.topLeftCorner(dim,dim+1)*pts;
  
  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(dim,n_pts,mxREAL);
    memcpy(mxGetPr(plhs[0]), x.data(), sizeof(double)*dim*n_pts);
  }

  if (nlhs>1) {
    MatrixXd tmp = model->bodies[body_ind].dTdq.topLeftCorner(dim*model->NB,dim+1)*pts;
    MatrixXd Jt = Map<MatrixXd>(tmp.data(),model->NB,dim*n_pts);
    MatrixXd J = Jt.transpose();
       
    plhs[1] = mxCreateDoubleMatrix(dim*n_pts,model->NB,mxREAL);
    memcpy(mxGetPr(plhs[1]), J.data(), sizeof(double)*dim*n_pts*model->NB);
  }

  if (nlhs>2) {
    int i,j;
    MatrixXd dJ_reshaped = MatrixXd(model->NB, dim*n_pts*model->NB);
    for (i = 0; i < model->NB; i++) {
      MatrixXd tmp = model->bodies[body_ind].ddTdqdq.block(i*model->NB*(dim+1),0,dim*model->NB,dim+1)*pts;  //dim*NB x n_pts
      for (j = 0; j < n_pts; j++) {
        dJ_reshaped.block(i,j*dim*model->NB,1,dim*model->NB) = tmp.col(j).transpose();
      }
//       dJ_reshaped.row(i) << tmp.col(0).transpose(), tmp.col(1).transpose();
    }
    MatrixXd dJ_t = Map<MatrixXd>(dJ_reshaped.data(), model->NB*model->NB, dim*n_pts);
    
    MatrixXd dJ = dJ_t.transpose();
  
    plhs[2] = mxCreateDoubleMatrix(dim*n_pts,model->NB*model->NB,mxREAL);
    memcpy(mxGetPr(plhs[2]), dJ.data(), sizeof(double)*dim*n_pts*model->NB*model->NB);
  }
}
