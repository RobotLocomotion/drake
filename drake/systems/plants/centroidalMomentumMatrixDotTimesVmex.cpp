#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [Adot_times_v, dAdot_times_v] = centroidalMomentumMatrixDotTimesV(model_ptr, cache_ptr, robotnum)";
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:centroidalMomentumMatrixDotTimesV:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:centroidalMomentumMatrixDotTimesV:WrongNumberOfOutputs", usage.c_str());
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  set<int> robotnum_set;
  int num_robot = static_cast<int>(mxGetNumberOfElements(prhs[arg_num]));
  double* robotnum = mxGetPrSafe(prhs[arg_num]);
  for (int i = 0; i < num_robot; i++) {
    robotnum_set.insert((int) robotnum[i] - 1);
  }
  arg_num++;

  int gradient_order = nlhs - 1;
  auto ret = model->centroidalMomentumMatrixDotTimesV<double>(*cache, gradient_order, robotnum_set);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}
