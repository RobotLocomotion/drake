#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Model.h"
#include "PlanarRigidBody.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the doKinematics function
 *
 * Piggybacks on HandCpmex.cpp to properly initialize and destroy models
 * Call with doKinematicsmex(q,b_compute_second_derivatives);
 */

void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[] ) {
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:NotEnoughInputs", "Usage doKinematicsmex(model_ptr,body_index,pts)");
  }
  
  Model *model = NULL;
  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs", "first argument should be the model_ptr");
  memcpy(&model, mxGetData(prhs[0]), sizeof(model));
  
  int n_pts = mxGetN(prhs[2]);
  
  if (mxGetM(prhs[2]) != 2)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 2");
  
  int body_ind = (int) mxGetScalar(prhs[1]);
  
  MatrixXd pts_tmp = MatrixXd::Zero(2,n_pts);  
  
  memcpy(pts_tmp.data(), mxGetPr(prhs[2]), sizeof(double)*2*n_pts);
  
  MatrixXd pts(3,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);
  
  MatrixXd x = model->bodies[body_ind].T.topLeftCorner(2,3)*pts;
  
  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(2,n_pts,mxREAL);
    memcpy(mxGetPr(plhs[0]), x.data(), sizeof(double)*2*n_pts);
  }
  
  if (nlhs>1) {
    MatrixXd tmp = model->bodies[body_ind].dTdq.topLeftCorner(2*model->NB,3)*pts;
    MatrixXd Jt = Map<MatrixXd>(tmp.data(),model->NB,2*n_pts);
    MatrixXd J = Jt.transpose();
        
    plhs[1] = mxCreateDoubleMatrix(2*n_pts,model->NB,mxREAL);
    memcpy(mxGetPr(plhs[1]), J.data(), sizeof(double)*2*n_pts*model->NB);
  }
  
  if (nlhs>2) {
    int i,j;
    MatrixXd dJ_reshaped = MatrixXd(model->NB, 2*n_pts*model->NB);
    for (i = 0; i < model->NB; i++) {
      MatrixXd tmp = model->bodies[body_ind].ddTdqdq.block(i*model->NB*3,0,2*model->NB,3)*pts;  //2*NB x n_pts
      for (j = 0; j < n_pts; j++) {
        dJ_reshaped.block(i,j*2*model->NB,1,2*model->NB) = tmp.col(j).transpose();
      }
//       dJ_reshaped.row(i) << tmp.col(0).transpose(), tmp.col(1).transpose();
    }
    MatrixXd dJ_t = Map<MatrixXd>(dJ_reshaped.data(), model->NB*model->NB, 2*n_pts);
    
    MatrixXd dJ = dJ_t.transpose();
    
    plhs[2] = mxCreateDoubleMatrix(2*n_pts,model->NB*model->NB,mxREAL);
    memcpy(mxGetPr(plhs[2]), dJ.data(), sizeof(double)*2*n_pts*model->NB*model->NB);
  }

}
