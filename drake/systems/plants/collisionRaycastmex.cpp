#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * mex interface for bullet raycast detection
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  
  if (nrhs < 5) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:NotEnoughInputs","Usage collisionRaycastmex(model_ptr, cache_ptr, origin_vector, ray_endpoint, use_margins)");
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));
  
  const mxArray* origin_vector_mex = prhs[arg_num++];
  const mxArray* ray_endpoint_mex = prhs[arg_num++];
  bool use_margins = (mxGetScalar(prhs[arg_num++])!=0.0);

  Matrix3Xd origins(3, mxGetN(origin_vector_mex)), ray_endpoints(3, mxGetN(ray_endpoint_mex));
  
  if (!mxIsNumeric(origin_vector_mex)) {
      mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputNotNumeric","Expected a numeric value, got something else.");
  }
  
  if (!mxIsNumeric(ray_endpoint_mex)) {
      mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputNotNumeric","Expected a numeric value, got something else.");
  }
  
  
  if (mxGetM(origin_vector_mex) != 3) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputSizeWrong","Expected a 3 x N matrix, got %d x %d.", mxGetM(origin_vector_mex), mxGetN(origin_vector_mex));
  }
  
  if (mxGetM(ray_endpoint_mex) != 3) {
    mexErrMsgIdAndTxt("Drake:collisionRaycastmex:InputSizeWrong","Expected a 3-element vector for ray_endpoint, got %d elements", mxGetNumberOfElements(ray_endpoint_mex));
  }
  
  
  memcpy(origins.data(), mxGetPrSafe(origin_vector_mex), sizeof(double)*mxGetNumberOfElements(origin_vector_mex));
  memcpy(ray_endpoints.data(), mxGetPrSafe(ray_endpoint_mex), sizeof(double)*mxGetNumberOfElements(ray_endpoint_mex));
  VectorXd distances;
  
  model->collisionRaycast(*cache, origins, ray_endpoints, distances, use_margins);
  
  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(static_cast<int>(distances.size()),1,mxREAL);
    memcpy(mxGetPrSafe(plhs[0]), distances.data(), sizeof(double)*distances.size());
  }
  
}
