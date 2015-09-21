#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "drakeGeometryUtil.h"
#include "math.h"

using namespace Eigen;
using namespace std;


void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage x = forwardKinmex(model_ptr, cache_ptr, body_or_frame_ind, points, rotation_type, base_or_frame_ind)";
  if (nrhs != 6) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs != 1) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:WrongNumberOfOutputs", usage.c_str());
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  int body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++]));
  auto points = matlabToEigen<SPACE_DIMENSION, Eigen::Dynamic>(prhs[arg_num++]);
  int rotation_type = (int) mxGetScalar(prhs[arg_num++]);
  int base_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++]));

  auto x = model->forwardKin(*cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type);
  plhs[0] = eigenToMatlab(x);
}
