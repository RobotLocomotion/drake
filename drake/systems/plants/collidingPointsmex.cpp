#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  //check number of arguments
  if (nrhs < 4) {
    mexErrMsgIdAndTxt("Drake:collidingPointsmex:NotEnoughInputs", "Usage: colliding_points = collidingPoints(mex_model_ptr, cache_ptr, points, collision_threshold)");
  }

  //check argument types
  if (!mxIsClass(prhs[0], "DrakeMexPointer")) {
    mexErrMsgIdAndTxt("Drake:collidingPointsmex:InvalidInputType", "Expected a DrakeMexPointer for mex_model_ptr but got something else.");
  }

  if (!mxIsClass(prhs[1], "DrakeMexPointer")) {
    mexErrMsgIdAndTxt("Drake:collidingPointsmex:InvalidInputType", "Expected a DrakeMexPointer for cache_ptr but got something else.");
  }

  if (!mxIsDouble(prhs[2])) {
    mexErrMsgIdAndTxt("Drake:collidingPointsmex:InvalidInputType", "Expected a double type for points but got something else.");
  }

  if (!mxIsDouble(prhs[3])) {
    mexErrMsgIdAndTxt("Drake:collidingPointsmex:InvalidInputType", "Expected a double type for collision_threshold but got something else.");    
  }
  
  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  const mxArray* points_mx = prhs[arg_num++];
  const mxArray* collision_threshold_mx = prhs[arg_num++];
  
  int n_points;
  Vector3d* points;
  double collision_threshold = *(double*)mxGetData(collision_threshold_mx);
  
  n_points = mxGetN(points_mx);
  //points.resize(3, n_points);
  cout << mxGetDimensions(points_mx)[1] << "\n";
  Vector3d* points_p;
  double* points_mx_p = (double*)mxGetData(points_mx);
  
  //cout << points_mx_p[3];
  *points << points_mx_p[3];
  //memcpy(points, points_mx_p, sizeof(double)*3);
  
  //memcpy(points_p, points_mx_p, sizeof(double)*3*n_points);
  
  /*for (int i=0; i<n_points; ++i){
    //memcpy(points_p,points_mx_p,sizeof(double)*3);
    cout << *points_mx_p << "\n";
    //points_p++;
    points_mx_p = points_mx_p+3;
  }  
  
  /*Vector3d* p = points.data();
  for (int i=0; i<points.size(); ++i){
    cout << *p;
    p++;
  }*/
  
  //vector<size_t> colliding_points = model->collidingPoints(*cache, points, collision_threshold);
  //memcpy(mxGetPrSafe(test[0]),colliding_points.data(),sizeof(size_t)*n_points);
  

  //if (nlhs>0) {
    //plhs[0] = mxCreateDoubleMatrix(1,n_points,mxREAL);
  //  plhs[0] = mxCreateDoubleMatrix(3,static_cast<int>(colliding_points.cols()),mxREAL);
  //  memcpy(mxGetPrSafe(plhs[0]),colliding_points.data(),sizeof(double)*3*colliding_points.cols());
  //}
}