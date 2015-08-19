#include <mex.h>
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [P, dP] = forwardKinPositionGradientmex(model_ptr, npoints, current_body_or_frame_ind, new_body_or_frame_ind)";
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfOutputs", usage.c_str());
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int npoints =  ((int) mxGetScalar(prhs[1]));
  int current_body_or_frame_ind = ((int) mxGetScalar(prhs[2])) - 1; // base 1 to base 0
  int new_body_or_frame_ind = ((int) mxGetScalar(prhs[3])) - 1; // base 1 to base 0
  int gradient_order = nlhs - 1;
  auto P = model->forwardKinPositionGradient<double>(npoints, current_body_or_frame_ind, new_body_or_frame_ind, gradient_order);
  plhs[0] = eigenToMatlab(P.value());

  if (P.hasGradient()) {
    plhs[1] = eigenToMatlab(P.gradient().value());
  }
}
