#include "mex.h"
#include <iostream>
#include "drake/util/drakeMexUtil.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  // check number of arguments
  if (nrhs < 4) {
    mexErrMsgIdAndTxt("Drake:collidingPointsmex:NotEnoughInputs",
                      "Usage: colliding_points = "
                      "collidingPointsCheckOnly(mex_model_ptr, cache_ptr, "
                      "points, collision_threshold)");
  }

  // check argument types
  if (!mxIsClass(prhs[0], "DrakeMexPointer")) {
    mexErrMsgIdAndTxt(
        "Drake:collidingPointsmex:InvalidInputType",
        "Expected a DrakeMexPointer for mex_model_ptr but got something else.");
  }

  if (!mxIsClass(prhs[1], "DrakeMexPointer")) {
    mexErrMsgIdAndTxt(
        "Drake:collidingPointsmex:InvalidInputType",
        "Expected a DrakeMexPointer for cache_ptr but got something else.");
  }

  if (!mxIsDouble(prhs[2])) {
    mexErrMsgIdAndTxt(
        "Drake:collidingPointsmex:InvalidInputType",
        "Expected a double type for points but got something else.");
  }

  if (!mxIsDouble(prhs[3])) {
    mexErrMsgIdAndTxt("Drake:collidingPointsmex:InvalidInputType",
                      "Expected a double type for collision_threshold but got "
                      "something else.");
  }

  int arg_num = 0;
  RigidBodyTree* model =
      static_cast<RigidBodyTree*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(
      getDrakeMexPointer(prhs[arg_num++]));

  const mxArray* points_mx = prhs[arg_num++];
  const mxArray* collision_threshold_mx = prhs[arg_num++];

  int n_points;
  vector<Vector3d> points;
  double collision_threshold = *(double*)mxGetData(collision_threshold_mx);

  n_points = mxGetN(points_mx);
  double* points_mx_p = (double*)mxGetData(points_mx);

  for (int i = 0; i < n_points; ++i) {
    Vector3d point(points_mx_p[3 * i + 0], points_mx_p[3 * i + 1],
                   points_mx_p[3 * i + 2]);
    points.push_back(point);
  }

  bool is_in_collision =
      model->collidingPointsCheckOnly(*cache, points, collision_threshold);

  if (nlhs > 0) {
    plhs[0] = mxCreateLogicalScalar(false);
    // plhs[0] = mxCreateLogicalScalar(is_in_collision);
    memcpy((bool*)(mxGetPr(plhs[0])), &is_in_collision, sizeof(bool));
  }
}
