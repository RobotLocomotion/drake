#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "drakeUtil.h"
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
    int n_active_bodies = mxGetNumberOfElements(body_idx);
    //DEBUG
    //cout << "collisionDetectmex: n_active_bodies = " << n_active_bodies << endl;
    //END_DEBUG
    active_bodies_idx.resize(n_active_bodies);
    memcpy(active_bodies_idx.data(),(int*) mxGetData(body_idx),sizeof(int)*n_active_bodies);
  }

  vector<int> bodyA_idx, bodyB_idx;
  MatrixXd ptsA, ptsB, normals, JA, JB, Jd;
  VectorXd dist;
  model->collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,active_bodies_idx);

  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(),bodyA_idx.end(),idxA.begin(),
      [](int i){return ++i;});
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(),bodyB_idx.end(),idxB.begin(),
      [](int i){return ++i;});

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(3,ptsA.cols(),mxREAL);
    memcpy(mxGetPr(plhs[0]),ptsA.data(),sizeof(double)*3*ptsA.cols());
  }
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(3,ptsB.cols(),mxREAL);
    memcpy(mxGetPr(plhs[1]),ptsB.data(),sizeof(double)*3*ptsB.cols());
  }
  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(3,normals.cols(),mxREAL);
    memcpy(mxGetPr(plhs[2]),normals.data(),sizeof(double)*3*normals.cols());
  }
  if (nlhs>3) {
    plhs[3] = mxCreateDoubleMatrix(1,dist.size(),mxREAL);
    memcpy(mxGetPr(plhs[3]),dist.data(),sizeof(double)*dist.size());
  }
  if (nlhs>4) {
    plhs[4] = mxCreateNumericMatrix(1,idxA.size(),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPr(plhs[4]),idxA.data(),sizeof(int32_T)*idxA.size());
  }
  if (nlhs>5) {
    plhs[5] = mxCreateNumericMatrix(1,idxB.size(),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPr(plhs[5]),idxB.data(),sizeof(int32_T)*idxB.size());
  }
}
