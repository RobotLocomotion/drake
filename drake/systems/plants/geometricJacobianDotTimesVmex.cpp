#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [Jdot_times_v, dJdot_times_v] = geometricJacobianDotTimesVmex(model_ptr, cache_ptr, base, end_effector, expressed_in)";
  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianDotTimesVmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianDotTimesVmex:WrongNumberOfOutputs", usage.c_str());
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  int base_body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0
  int end_effector_body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0
  int expressed_in_body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0
  int gradient_order = nlhs - 1;
  auto ret = model->geometricJacobianDotTimesV<double>(*cache, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}
