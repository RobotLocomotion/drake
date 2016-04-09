#include <mex.h>

#include <iostream>
#include <memory>
#include "drake/util/drakeMexUtil.h"
#include "RigidBodyTree.h"
#include "drake/util/drakeGeometryUtil.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  std::string usage =
      "Usage [phi, n, x, body_x, body_idx] = "
      "collisionDetectFromPointsmex(model_ptr, cache_ptr, points, use_margins)";
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:collisionDetectFromPointsmex:WrongNumberOfInputs",
                      usage.c_str());
    printf("Had %d inputs\n", nrhs);
  }

  if (nlhs != 5) {
    mexErrMsgIdAndTxt("Drake:collisionDetectFromPointsmex:WrongNumberOfOutputs",
                      usage.c_str());
    printf("Had %d outputs\n", nlhs);
  }

  int arg_num = 0;
  RigidBodyTree *model =
      static_cast<RigidBodyTree *>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double> *cache = static_cast<KinematicsCache<double> *>(
      getDrakeMexPointer(prhs[arg_num++]));

  auto points = matlabToEigen<SPACE_DIMENSION, Eigen::Dynamic>(prhs[arg_num++]);
  bool use_margins = (bool)(mxGetLogicals(prhs[arg_num++]))[0];

  VectorXd phi;
  Matrix3Xd normal;
  Matrix3Xd x;
  Matrix3Xd body_x;
  vector<int> body_idx;

  model->collisionDetectFromPoints(*cache, points, phi, normal, x, body_x,
                                   body_idx, use_margins);

  plhs[0] = eigenToMatlab(phi);
  plhs[1] = eigenToMatlab(normal);
  plhs[2] = eigenToMatlab(x);
  plhs[3] = eigenToMatlab(body_x);
  plhs[4] = stdVectorToMatlab(body_idx);
}
