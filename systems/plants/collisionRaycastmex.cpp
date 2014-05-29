#include "mex.h"
#include <Eigen/Dense>
#include <vector>
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
  
  
  Vector3d origin, ray_endpoint;
  
  if (mxGetNumberOfElements(prhs[1]) != 3) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputSizeWrong","Expected a 3-element vector for origin, got %d elements", mxGetNumberOfElements(prhs[1]));
  }
  
  if (mxGetNumberOfElements(prhs[2]) != 3) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputSizeWrong","Expected a 3-element vector for ray_endpoint, got %d elements", mxGetNumberOfElements(prhs[2]));
  }
  
  
  memcpy(origin.data(), mxGetPr(prhs[1]), sizeof(double)*3);
  memcpy(ray_endpoint.data(), mxGetPr(prhs[2]), sizeof(double)*3);
  
  
  double distance = model->collisionRaycast(origin, ray_endpoint);
  

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleScalar(distance);
  }
}
