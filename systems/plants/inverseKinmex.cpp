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

void ik( double* q, VectorXd q0, VectorXd q_nom, MatrixXd Q, int narg, int* body_ind, VectorXd* pts, double* f, double** G)
{
  /*
  f = zeros(nF,1); G = zeros(nF,obj.num_q);
  f(1) = (q-q_nom)'*Q*(q-q_nom);
  G(1,:) = 2*(q-q_nom)'*Q;
  kinsol = doKinematics(obj,q,false);
  i=1;j=2;
  while i<length(varargin)
    if (varargin{i}==0)
      do_rot = 0;
      [x,J] = getCOM(obj,kinsol);
    else
      do_rot = length(varargin{i+1})==6;
      [x,J] = forwardKin(obj,kinsol,varargin{i},[0;0;0],do_rot);
    end
    ind = ~isnan(varargin{i+1});
    n = sum(ind);
    f([j:j+n-1]) = x(ind) - varargin{i+1}(ind);
    G([j:j+n-1],:) = J(ind,:);
    j=j+n;
    i=i+2;  
   */
}

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
    body_ind[i] = (int) mxGetScalar(prhs[4+2*i]);
    pts[i] = VectorXd::Zero(mxGetNumberOfElements(prhs[4+2*i+1]));
    memcpy(pts[i].data(),mxGetPr(prhs[5+i]),sizeof(double)*pts[i].rows());
    for (j=0; j<pts[i].rows(); j++)
      if (!isnan(pts[i](j))) nF++; 
  }

  mexPrintf("nF = %d\n",nF);
  
  delete[] pts;
  delete[] body_ind;
}
