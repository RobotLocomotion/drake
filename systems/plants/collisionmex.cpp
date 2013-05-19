#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * mex interface for bullet collision detection
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 3) {
    mexErrMsgIdAndTxt("Drake:collisionmex:NotEnoughInputs","Usage collisionmex(model_ptr,command,arg1,arg2,...");
  }

  RigidBodyManipulator *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));

  int command = (int) mxGetScalar(prhs[1]);
  switch (command)
  {
  case 1:  // pairwiseCollision()
    {
    	MatrixXd ptsA, ptsB;
    	int body_indA = (int) mxGetScalar(prhs[2])-1, body_indB = (int) mxGetScalar(prhs[3])-1;
      model->getPairwiseCollision(body_indA,body_indB,ptsA,ptsB);
      if (nlhs>0) {
      	plhs[0] = mxCreateDoubleMatrix(3,ptsA.cols(),mxREAL);
      	memcpy(mxGetPr(plhs[0]),ptsA.data(),sizeof(double)*3*ptsA.cols());
      }
      if (nlhs>1) {
      	plhs[1] = mxCreateDoubleMatrix(3,ptsB.cols(),mxREAL);
      	memcpy(mxGetPr(plhs[1]),ptsB.data(),sizeof(double)*3*ptsB.cols());
      }
    }
  	break;
  default:
  	mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","unknown collision command");
  	break;
  
  }

}
