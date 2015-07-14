#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * mex interface for bullet collision detection
 * closest-distance for each body to all other bodies (~(NB^2-NB)/2 points)
 *
 * MATLAB signature:
 *
 * [xA,xB,normal,distance,idxA,idxB] = ...
 *    collisionDetectmex( mex_model_ptr,allow_multiple_contacts,
 *                        active_collision_options);
 */

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  //check number of arguments
  if (nrhs < 3) {
    mexErrMsgIdAndTxt("Drake:collisionDetectmex:NotEnoughInputs", "Usage: [xA,xB,normal,distance,idxA,idxB] = collisionDetectmex(mex_model_ptr, allow_multiple_contacts, active_collision_options)");
  }

  //check argument types
  if (!mxIsClass(prhs[0], "DrakeMexPointer")) {
    mexErrMsgIdAndTxt("Drake:collisionDetectmex:InvalidInputType", "Expected a DrakeMexPointer for mex_model_ptr but got something else.");
  }

  if (!mxIsLogical(prhs[1])) {
    mexErrMsgIdAndTxt("Drake:collisionDetectmex:InvalidInputType", "Expected a boolean logic type for allow_multiple_collisions but got something else.");
  }

  if (!mxIsStruct(prhs[2])) { 
    mexErrMsgIdAndTxt("Drake:collisionDetectmex:InvalidInputType", "Expected a struct type for active_collision_options but got something else.");    
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  // Parse `active_collision_options`
  vector<int> active_bodies_idx;
  set<string> active_group_names;

  const mxArray* active_collision_options = prhs[2];
  const mxArray* allow_multiple_contacts = prhs[1];

  const mxArray* body_idx = mxGetField(active_collision_options, 0, "body_idx");
  if (body_idx != NULL) {
    size_t n_active_bodies = static_cast<size_t>(mxGetNumberOfElements(body_idx));

    active_bodies_idx.resize(n_active_bodies);
    memcpy(active_bodies_idx.data(),(int*) mxGetData(body_idx),
           sizeof(int)*n_active_bodies);
    transform(active_bodies_idx.begin(), active_bodies_idx.end(),
              active_bodies_idx.begin(),
              [](int i){return --i;});
  }

  const mxArray* collision_groups = mxGetField(active_collision_options, 0, "collision_groups");
  
  if (collision_groups != NULL) {
    size_t num_collision_groups = static_cast<size_t>(mxGetNumberOfElements(collision_groups));
    for (size_t i = 0; i < num_collision_groups; i++) {
      const mxArray *ptr = mxGetCell(collision_groups, i);
      size_t buflen = static_cast<size_t>(mxGetN(ptr) * sizeof(mxChar)) + 1;
      char* str = (char*)mxMalloc(buflen);
      mxGetString(ptr, str, buflen);
      active_group_names.insert(str);
      mxFree(str);
    }
  }

  vector<int> bodyA_idx, bodyB_idx;
  Matrix3Xd ptsA, ptsB, normals;
  MatrixXd JA, JB, Jd;
  VectorXd dist;

  if (active_bodies_idx.size() > 0) {
    if (active_group_names.size() > 0) {
      model->collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx,active_group_names);
    } else {
      model->collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx);
    }
  } else {
    const bool multiple_contacts = mxIsLogicalScalarTrue(allow_multiple_contacts);
    if(multiple_contacts) {
      model->potentialCollisions(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx);
    } else if (active_group_names.size() > 0) {
      model->collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                             active_group_names);
    } else {
      model->collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx);
    }
  }

  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(), bodyA_idx.end(), idxA.begin(),
      [](int i){return ++i;});
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(), bodyB_idx.end(), idxB.begin(),
      [](int i){return ++i;});

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(3,static_cast<int>(ptsA.cols()),mxREAL);
    memcpy(mxGetPrSafe(plhs[0]),ptsA.data(),sizeof(double)*3*ptsA.cols());
  }
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(3,static_cast<int>(ptsB.cols()),mxREAL);
    memcpy(mxGetPrSafe(plhs[1]),ptsB.data(),sizeof(double)*3*ptsB.cols());
  }
  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(3,static_cast<int>(normals.cols()),mxREAL);
    memcpy(mxGetPrSafe(plhs[2]),normals.data(),sizeof(double)*3*normals.cols());
  }
  if (nlhs>3) {
    plhs[3] = mxCreateDoubleMatrix(1,static_cast<int>(dist.size()),mxREAL);
    memcpy(mxGetPrSafe(plhs[3]),dist.data(),sizeof(double)*dist.size());
  }
  if (nlhs>4) {
    plhs[4] = mxCreateNumericMatrix(1,static_cast<int>(idxA.size()),mxINT32_CLASS,mxREAL);
    memcpy(mxGetData(plhs[4]),idxA.data(),sizeof(int32_T)*idxA.size());
  }
  if (nlhs>5) {
    plhs[5] = mxCreateNumericMatrix(1,static_cast<int>(idxB.size()),mxINT32_CLASS,mxREAL);
    memcpy(mxGetData(plhs[5]),idxB.data(),sizeof(int32_T)*idxB.size());
  }
}
