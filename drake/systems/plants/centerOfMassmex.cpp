#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [x, J, dJ] = centerOfMassmex(model_ptr, cache_ptr, robotnum, in_terms_of_qdot)";
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:centerOfMassmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:centerOfMassmex:WrongNumberOfOutputs", usage.c_str());
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

  bool in_terms_of_qdot = (bool) (mxGetLogicals(prhs[arg_num++]))[0];
  int gradient_order = nlhs - 1;

  auto com = model->centerOfMass<double>(*cache, 0, robotnum_set);
  plhs[0] = eigenToMatlab(com.value());
  if (gradient_order > 0) {
    auto J_com = model->centerOfMassJacobian<double>(*cache, gradient_order - 1, robotnum_set, in_terms_of_qdot);
    plhs[1] = eigenToMatlab(J_com.value());
    if (gradient_order > 1)
    plhs[2] = eigenToMatlab(J_com.gradient().value());
  }
}
