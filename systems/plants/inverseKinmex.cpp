#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Model.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the inverseKin function
 *
 */


void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if ((nrhs<4) || (nrhs%2!=0)) {
    mexErrMsgIdAndTxt("Drake:inverseKinmex:NotEnoughInputs","Usage inverseKinmex(model_ptr,q0,q_nom,Q,body1_ind,body1_pos,body2_ind,body2_pos,...) ");
  }

  Model *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:inverseKinmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));
  
  int nq = mxGetM(prhs[1]);

  VectorXd q0 = VectorXd::Zero(nq);
  memcpy(q0.data(),mxGetPr(prhs[1]),sizeof(double)*nq);
  
  VectorXd q_nom = VectorXd::Zero(nq);
  memcpy(q_nom.data(),mxGetPr(prhs[2]),sizeof(double)*nq);
  
  MatrixXd Q = MatrixXd::Zero(nq,nq);
  memcpy(Q.data(),mxGetPr(prhs[3]),sizeof(double)*nq*nq);

  int narg = (nrhs-4)/2;
  int nF = 1, i,j;
  int* body_ind = new int[narg];
  VectorXd* pts = new VectorXd[narg];
  for (i=0; i<narg; i++) {
    body_ind[i] = ((int) mxGetScalar(prhs[4+2*i]))-1;
    pts[i] = VectorXd::Zero(mxGetNumberOfElements(prhs[4+2*i+1]));
    memcpy(pts[i].data(),mxGetPr(prhs[4+2*i+1]),sizeof(double)*pts[i].rows());
    for (j=0; j<pts[i].rows(); j++)
      if (!isnan(pts[i](j))) nF++; 
  }

  VectorXd q = VectorXd::Zero(nq);  // these could be resize instead of zero
  VectorXd f = VectorXd::Zero(nF);
  MatrixXd G = MatrixXd::Zero(nF,nq);
  
  model->snoptIKfun(q, q0, q_nom, Q, narg, body_ind, pts, &f, &G);
  
  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(nF,1,mxREAL);
    memcpy(mxGetPr(plhs[0]), f.data(), sizeof(double)*nF);    
    if (nlhs>1) {
      plhs[1] = mxCreateDoubleMatrix(nF,nq,mxREAL);
      memcpy(mxGetPr(plhs[1]), G.data(), sizeof(double)*nF*nq);
    }
  }  
  
  delete[] pts;
  delete[] body_ind;
}
