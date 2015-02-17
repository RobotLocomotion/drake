#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [A, dA] = centroidalMomentumMatrixmex()";
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int gradient_order = nlhs - 1;
  auto A = model->centroidalMomentumMatrix<double>(gradient_order);

  plhs[0] = eigenToMatlab(A.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(A.gradient().value());
}
