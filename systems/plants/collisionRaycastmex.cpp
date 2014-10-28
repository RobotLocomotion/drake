#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * mex interface for bullet raycast detection
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  
  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:NotEnoughInputs","Usage collisionRaycastmex(model_ptr, origin_vector, ray_endpoint)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  
  Matrix3Xd origins(3, mxGetN(prhs[1])), ray_endpoints(3, mxGetN(prhs[2]));
  
  if (mxIsNumeric(prhs[1]) != true) {
      mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputNotNumeric","Expected a numeric value, got something else.");
  }
  
  if (mxIsNumeric(prhs[2]) != true) {
      mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputNotNumeric","Expected a numeric value, got something else.");
  }
  
  
  if (mxGetM(prhs[1]) != 3) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputSizeWrong","Expected a 3 x N matrix, got %d x %d.", mxGetM(prhs[1]), mxGetN(prhs[1]));
  }
  
  if (mxGetM(prhs[2]) != 3) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputSizeWrong","Expected a 3-element vector for ray_endpoint, got %d elements", mxGetNumberOfElements(prhs[2]));
  }
  
  
  memcpy(origins.data(), mxGetPr(prhs[1]), sizeof(double)*mxGetNumberOfElements(prhs[1]));
  memcpy(ray_endpoints.data(), mxGetPr(prhs[2]), sizeof(double)*mxGetNumberOfElements(prhs[2]));
  bool use_margins = (bool) mxGetScalar(prhs[3]);
  VectorXd distances;
  
  model->collisionRaycast(origins, ray_endpoints, distances, use_margins);
  
  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(distances.size(),1,mxREAL);
    memcpy(mxGetPr(plhs[0]), distances.data(), sizeof(double)*distances.size());
  }
  
}
