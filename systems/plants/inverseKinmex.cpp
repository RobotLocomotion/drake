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

  if (nrhs < 4) {
    mexErrMsgIdAndTxt("Drake:inverseKinmex:NotEnoughInputs","Usage inverseKinmex(model_ptr,q0,q_nom,Q,body1_ind,body1_pos,body2_ind,body2_pos) for center of mass, or forwardKinmex(model_pts,body_ind,pts,include_rotations)");
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
  
  
}
