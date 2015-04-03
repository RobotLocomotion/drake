#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
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

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  //DEBUG
  //cout << "collisionDetectmex: START" << endl;
  //END_DEBUG

  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:collisionDetectmex:NotEnoughInputs","Usage collisionDetectmex(model_ptr)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  // Parse `active_collision_options`
  vector<int> active_bodies_idx;
  set<string> active_group_names;
  // First get the list of body indices for which to compute distances
  // DEBUG
  //cout << "collisionDetectmex: Parse active_bodies START" << endl;
  // END_DEBUG
  const mxArray* active_collision_options = prhs[2];
  const mxArray* body_idx = mxGetField(active_collision_options,0,"body_idx");
  if (body_idx != NULL) {
    //DEBUG
    //cout << "collisionDetectmex: Received body_idx" << endl;
    //END_DEBUG
    int n_active_bodies = static_cast<int>(mxGetNumberOfElements(body_idx));
    //DEBUG
    //cout << "collisionDetectmex: n_active_bodies = " << n_active_bodies << endl;
    //END_DEBUG
    active_bodies_idx.resize(n_active_bodies);
    memcpy(active_bodies_idx.data(),(int*) mxGetData(body_idx),
           sizeof(int)*n_active_bodies);
    transform(active_bodies_idx.begin(),active_bodies_idx.end(),
              active_bodies_idx.begin(),
              [](int i){return --i;});
  }
  // DEBUG
  //cout << "collisionDetectmex: Parse active_bodies END" << endl;
  // END_DEBUG

  // Then get the group names for which to compute distances
  // DEBUG
  //cout << "collisionDetectmex: Parse collision groups START" << endl;
  // END_DEBUG
  const mxArray* collision_groups = mxGetField(active_collision_options,0,
                                               "collision_groups");
  if (collision_groups != NULL) {
    int num = static_cast<int>(mxGetNumberOfElements(collision_groups));
    for (int i=0; i<num; i++) {
      const mxArray *ptr = mxGetCell(collision_groups,i);
      int buflen = static_cast<int>(mxGetN(ptr)*sizeof(mxChar))+1;
      char* str = (char*)mxMalloc(buflen);
      mxGetString(ptr, str, buflen);
      active_group_names.insert(str);
      mxFree(str);
    }
  }
  // DEBUG
  //cout << "collisionDetectmex: Parse collision groups END" << endl;
  // END_DEBUG

  vector<int> bodyA_idx, bodyB_idx;
  MatrixXd ptsA, ptsB, normals, JA, JB, Jd;
  VectorXd dist;
  // DEBUG
  //cout << "collisionDetectmex: Call collisionDetect START" << endl;
  // END_DEBUG
  if (active_bodies_idx.size() > 0) {
    if (active_group_names.size() > 0) {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx,active_group_names);
    } else {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx);
    }
  } else {
    if (active_group_names.size() > 0) {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                             active_group_names);
    } else {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx);
    }
  }
  // DEBUG
  //cout << "collisionDetectmex: Call collisionDetect END" << endl;
  // END_DEBUG

  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(),bodyA_idx.end(),idxA.begin(),
      [](int i){return ++i;});
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(),bodyB_idx.end(),idxB.begin(),
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
  //DEBUG
  //cout << "collisionDetectmex: END" << endl;
  //END_DEBUG
}
