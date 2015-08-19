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
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:collisionDetectmex:NotEnoughInputs","Usage collisionDetectmex(model_ptr)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  // Now get the list of body indices for which to compute distances
  vector<int> active_bodies_idx;
  const mxArray* active_collision_options = prhs[2];
  //DEBUG
  //cout << "collisionDetectmex: Num fields in active_collision_options" << mxGetNumberOfFields(active_collision_options) << endl;
  //for (int i = 0; i < mxGetNumberOfFields(active_collision_options); ++i) {
    //const char* fieldname;
    //fieldname = mxGetFieldNameByNumber(active_collision_options,i);
    //cout << *fieldname << endl;
  //}
  //END_DEBUG
  const mxArray* body_idx = mxGetField(active_collision_options,0,"body_idx");
  if (body_idx != NULL) {
    //DEBUG
    //cout << "collisionDetectmex: Received body_idx" << endl;
    //END_DEBUG
    size_t n_active_bodies = mxGetNumberOfElements(body_idx);
    //DEBUG
    //cout << "collisionDetectmex: n_active_bodies = " << n_active_bodies << endl;
    //END_DEBUG
    active_bodies_idx.resize(n_active_bodies);
    memcpy(active_bodies_idx.data(),(int*) mxGetData(body_idx),sizeof(int)*n_active_bodies);
    transform(active_bodies_idx.begin(),active_bodies_idx.end(),active_bodies_idx.begin(),
        [](int i){return --i;});
  }

  vector<int> bodyA_idx, bodyB_idx;
  Matrix3Xd ptsA, ptsB;
  model->allCollisions(bodyA_idx,bodyB_idx,ptsA,ptsB);
  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(),bodyA_idx.end(),idxA.begin(),
      [](int i){return ++i;});
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(),bodyB_idx.end(),idxB.begin(),
      [](int i){return ++i;});

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(3,ptsA.cols(),mxREAL);
    memcpy(mxGetPrSafe(plhs[0]),ptsA.data(),sizeof(double)*3*ptsA.cols());
  }
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(3,ptsB.cols(),mxREAL);
    memcpy(mxGetPrSafe(plhs[1]),ptsB.data(),sizeof(double)*3*ptsB.cols());
  }
  if (nlhs>2) {
    plhs[2] = mxCreateNumericMatrix(1,idxA.size(),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPrSafe(plhs[2]),idxA.data(),sizeof(int32_T)*idxA.size());
  }
  if (nlhs>3) {
    plhs[3] = mxCreateNumericMatrix(1,idxB.size(),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPrSafe(plhs[3]),idxB.data(),sizeof(int32_T)*idxB.size());
  }
}
