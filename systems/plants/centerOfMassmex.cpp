#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [x, J, dJ] = centerOfMassmex(model_ptr, robotnum, in_terms_of_qdot)";
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:centerOfMassmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:centerOfMassmex:WrongNumberOfOutputs", usage.c_str());
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

  auto com = model->centerOfMass<double>(0, robotnum_set);
  plhs[0] = eigenToMatlab(com.value());
  if (gradient_order > 0) {
    auto J_com = model->centerOfMassJacobian<double>(gradient_order - 1, robotnum_set, in_terms_of_qdot);
    plhs[1] = eigenToMatlab(J_com.value());
    if (gradient_order > 1)
    plhs[2] = eigenToMatlab(J_com.gradient().value());
  }
}
