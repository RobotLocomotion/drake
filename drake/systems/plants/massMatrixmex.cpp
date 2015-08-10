#include <mex.h>
#include <iostream>

#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  string usage = "Usage [M, dM] = massMatrixmex(model_ptr)";
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfOutputs", usage.c_str());
  }

  int gradient_order = nlhs - 1;


  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  auto ret = model->massMatrix<double>(gradient_order);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}
