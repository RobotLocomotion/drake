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

  std::string usage = "Usage [Jdot_times_v, dJdot_times_v] = forwardJacDotTimesVmex(model_ptr, cache_ptr, body_or_frame_ind, points, rotation_type, base_or_frame_ind)";
  if (nrhs != 6) {
    mexErrMsgIdAndTxt("Drake:forwardJacDotTimesVmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:forwardJacDotTimesVmex:WrongNumberOfOutputs", usage.c_str());
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  int body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0
  auto points = matlabToEigen<SPACE_DIMENSION, Eigen::Dynamic>(prhs[arg_num++]);
  int rotation_type = (int) mxGetScalar(prhs[arg_num++]);
  int base_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0

  int gradient_order = nlhs - 1;
  auto Jdot_times_v = model->forwardJacDotTimesV(*cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);
  plhs[0] = eigenToMatlab(Jdot_times_v.value());
  if (gradient_order > 0) {
    plhs[1] = eigenToMatlab(Jdot_times_v.gradient().value());
  }
}
