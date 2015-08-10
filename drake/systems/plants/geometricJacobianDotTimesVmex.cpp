#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [Jdot_times_v, dJdot_times_v] = geometricJacobianDotTimesVmex(model_ptr, base, end_effector, expressed_in)";
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianDotTimesVmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianDotTimesVmex:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  int base_body_or_frame_ind = ((int) mxGetScalar(prhs[1])) - 1; // base 1 to base 0
  int end_effector_body_or_frame_ind = ((int) mxGetScalar(prhs[2])) - 1; // base 1 to base 0
  int expressed_in_body_or_frame_ind = ((int) mxGetScalar(prhs[3])) - 1; // base 1 to base 0
  int gradient_order = nlhs - 1;
  auto ret = model->geometricJacobianDotTimesV<double>(base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}
