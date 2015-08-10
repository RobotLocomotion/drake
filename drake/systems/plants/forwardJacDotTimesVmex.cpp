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

  std::string usage = "Usage [Jdot_times_v, dJdot_times_v] = forwardJacDotTimesVmex(model_ptr, body_or_frame_ind, points, rotation_type, base_or_frame_ind)";
  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:forwardJacDotTimesVmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:forwardJacDotTimesVmex:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  int body_or_frame_ind = ((int) mxGetScalar(prhs[1])) - 1; // base 1 to base 0
  auto points = matlabToEigen<SPACE_DIMENSION, Eigen::Dynamic>(prhs[2]);
  int rotation_type = (int) mxGetScalar(prhs[3]);
  int base_or_frame_ind = ((int) mxGetScalar(prhs[4])) - 1; // base 1 to base 0

  int gradient_order = nlhs - 1;
  auto Jdot_times_v = model->forwardJacDotTimesV(points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);
  plhs[0] = eigenToMatlab(Jdot_times_v.value());
  if (gradient_order > 0) {
    plhs[1] = eigenToMatlab(Jdot_times_v.gradient().value());
  }
}
