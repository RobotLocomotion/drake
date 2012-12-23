#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "PlanarModel.h"
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
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:forwardKinVelpmex:NotEnoughInputs", "Usage forwardKinVel(model_ptr,body_index,pts,qd)");
  }
  
  PlanarModel *model = NULL;
  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:forwardKinVelpmex:BadInputs", "first argument should be the model_ptr");
  memcpy(&model, mxGetData(prhs[0]), sizeof(model));
  
  int n_pts = mxGetN(prhs[2]);
  
  if (mxGetM(prhs[2]) != 2)
    mexErrMsgIdAndTxt("Drake:forwardKinVelpmex:BadInputs", "number of rows in pts must be 2");
  
  int body_ind = (int) mxGetScalar(prhs[1]);
  
  MatrixXd pts_tmp = MatrixXd::Zero(2,n_pts);  
  
  // todo: check that the size is 3x3
  memcpy(pts_tmp.data(), mxGetPr(prhs[2]), sizeof(double)*2*n_pts);
  
  MatrixXd pts(3,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);
  
  
  double *qd;
  if (mxGetNumberOfElements(prhs[3])!=model->NB)
    mexErrMsgIdAndTxt("Drake:forwardKinVelpmex:BadInputs", "qd must be size %d x 1", model->NB);
  qd = mxGetPr(prhs[3]);
  
  MatrixXd v(2,n_pts);
  
  MatrixXd tmp = model->bodies[body_ind].dTdq.topLeftCorner(2*model->NB,3)*pts;
  
  MatrixXd qd_t = Map<MatrixXd>(qd, 1, model->NB);
      
  int i, j;
  for (i=0;i<n_pts;i++) {
    v.col(i) << qd_t*tmp.block(0,i,model->NB,1), qd_t*tmp.block(model->NB,i,model->NB,1);
  }

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(2,n_pts,mxREAL);
    memcpy(mxGetPr(plhs[0]), v.data(), sizeof(double)*2*n_pts);
  }
  
  if (nlhs>1) {
    MatrixXd tmp_reshaped = Map<MatrixXd>(tmp.data(), model->NB, 2*n_pts);    
    
    MatrixXd dv = MatrixXd(2*n_pts,2*model->NB);
    dv.block(0,model->NB,2*n_pts,model->NB) << tmp_reshaped.transpose();
    
    MatrixXd qd_rep = MatrixXd::Zero(2,3*model->NB);
    qd_rep.block(0,0,1,model->NB) = qd_t;
    qd_rep.block(1,model->NB,1,model->NB) = qd_t;
    
    for (i=0;i<model->NB;i++) {
      MatrixXd col_data = qd_rep*model->bodies[body_ind].ddTdqdq.block(3*i*model->NB,0,2*model->NB,3)*pts;
      MatrixXd col = Map<MatrixXd>(col_data.data(), 2*n_pts, 1);
      dv.col(i) = col;
    }
        
    plhs[1] = mxCreateDoubleMatrix(2*n_pts,2*model->NB,mxREAL);
    memcpy(mxGetPr(plhs[1]), dv.data(), sizeof(double)*4*n_pts*model->NB);
  }
}
