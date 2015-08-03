#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [Adot_times_v, dAdot_times_v] = centroidalMomentumMatrixDotTimesV(model_ptr, robotnum)";
  if (nrhs != 2) {
    mexErrMsgIdAndTxt("Drake:centroidalMomentumMatrixDotTimesV:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:centroidalMomentumMatrixDotTimesV:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  set<int> robotnum_set;
  int num_robot = static_cast<int>(mxGetNumberOfElements(prhs[1]));
  double* robotnum = mxGetPrSafe(prhs[1]);
  for (int i = 0; i < num_robot; i++) {
    robotnum_set.insert((int) robotnum[i] - 1);
  }
  int gradient_order = nlhs - 1;
  auto ret = model->centroidalMomentumMatrixDotTimesV<double>(gradient_order, robotnum_set);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}
