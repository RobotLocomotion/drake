#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [A, dA] = centroidalMomentumMatrixmex(mex_model_ptr, robotnum, in_terms_of_qdot)";
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:centroidalMomentumMatrixmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:centroidalMomentumMatrixmex:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  set<int> robotnum_set;
  int num_robot = static_cast<int>(mxGetNumberOfElements(prhs[1]));
  double* robotnum = mxGetPrSafe(prhs[1]);
  for (int i = 0; i < num_robot; i++) {
    robotnum_set.insert((int) robotnum[i] - 1);
  }
  bool in_terms_of_qdot = (bool) (mxGetLogicals(prhs[2]))[0];

  int gradient_order = nlhs - 1;
  auto A = model->centroidalMomentumMatrix<double>(gradient_order, robotnum_set, in_terms_of_qdot);

  plhs[0] = eigenToMatlab(A.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(A.gradient().value());
}
