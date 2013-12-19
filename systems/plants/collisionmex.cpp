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
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int command = (int) mxGetScalar(prhs[1]);
  switch (command)
  {
  case 1:  // pairwiseCollision()
    {
      if (nrhs < 3) {
        mexErrMsgIdAndTxt("Drake:collisionmex:NotEnoughInputs","Usage collisionmex(model_ptr,command,arg1,arg2,...");
      }

    	MatrixXd ptsA, ptsB, normals;
    	int bodyA_idx = (int) mxGetScalar(prhs[2])-1, bodyB_idx = (int) mxGetScalar(prhs[3])-1;
    	if (bodyA_idx<0 || bodyA_idx>=model->num_bodies || bodyB_idx<0 || bodyB_idx>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");
      model->getPairwiseCollision(bodyA_idx,bodyB_idx,ptsA,ptsB,normals);
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
    }
    break;
  case 2: // pairwise collisions with specified bodyA points  
    {
      if (nrhs < 3) {
        mexErrMsgIdAndTxt("Drake:collisionmex:NotEnoughInputs","Usage collisionmex(model_ptr,command,arg1,arg2,...");
      }

    	MatrixXd ptsA, ptsB, normals;
    	Vector3d ptA, ptB, normal;
    	int bodyA_idx = (int) mxGetScalar(prhs[2])-1, bodyB_idx = (int) mxGetScalar(prhs[3])-1;
    	if (bodyA_idx<0 || bodyA_idx>=model->num_bodies || bodyB_idx<0 || bodyB_idx>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");

      double* pr = mxGetPr(prhs[4]);
      int num_pts = mxGetNumberOfElements(prhs[4]);
      
      ptsA.resize(3,num_pts);
      ptsB.resize(3,num_pts);
      normals.resize(3,num_pts);
      for (int i=0; i<num_pts; i++) {
        model->getPairwisePointCollision(bodyA_idx,bodyB_idx,(int)pr[i]-1,ptA,ptB,normal);
        ptsA.col(i) = ptA;
        ptsB.col(i) = ptB;
        normals.col(i) = normal;
      }
  
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
    }
    break;
  case 3: // full world collisions with specified bodyA points  
    {
      if (nrhs < 3) {
        mexErrMsgIdAndTxt("Drake:collisionmex:NotEnoughInputs","Usage collisionmex(model_ptr,command,arg1,arg2,...");
      }

    	MatrixXd ptsA, ptsB, normals;
    	Vector3d ptA, ptB, normal;
    	int bodyA_idx = (int) mxGetScalar(prhs[2])-1;
    	if (bodyA_idx<0 || bodyA_idx>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");

      double* pr = mxGetPr(prhs[3]);
      int num_pts = mxGetNumberOfElements(prhs[3]);
      
      ptsA.resize(3,num_pts);
      ptsB.resize(3,num_pts);
      normals.resize(3,num_pts);
      for (int i=0; i<num_pts; i++) {
        model->getPointCollision(bodyA_idx,(int)pr[i]-1,ptA,ptB,normal);
        ptsA.col(i) = ptA;
        ptsB.col(i) = ptB;
        normals.col(i) = normal;
      }
  
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
    }
    break;
  case 4: // pairwise closest-distance
    {
      if (nrhs < 3) {
        mexErrMsgIdAndTxt("Drake:collisionmex:NotEnoughInputs","Usage collisionmex(model_ptr,command,arg1,arg2,...");
      }

    	Vector3d ptA, ptB, normal;
      double dist;
    	int bodyA_idx = (int) mxGetScalar(prhs[2])-1;
      int bodyB_idx = (int) mxGetScalar(prhs[3])-1;
    	if (bodyA_idx<0 || bodyA_idx>=model->num_bodies || bodyB_idx<0 || bodyB_idx>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");
      model->getPairwiseClosestPoint(bodyA_idx,bodyB_idx,ptA,ptB,normal,dist);
      if (nlhs>0) {
        plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
        memcpy(mxGetPr(plhs[0]),ptA.data(),sizeof(double)*3);
      }
      if (nlhs>1) {
        plhs[1] = mxCreateDoubleMatrix(3,1,mxREAL);
        memcpy(mxGetPr(plhs[1]),ptB.data(),sizeof(double)*3);
      }
      if (nlhs>2) {
        plhs[2] = mxCreateDoubleMatrix(3,1,mxREAL);
        memcpy(mxGetPr(plhs[2]),normal.data(),sizeof(double)*3);
      }
      if (nlhs>3) {
        plhs[3] = mxCreateDoubleScalar(dist);
      }
    }
    break;
  case 5: // closest-distance for each body to all other bodies (~(NB^2-NB)/2 points)
    {
      if (nrhs < 2) {
        mexErrMsgIdAndTxt("Drake:collisionmex:NotEnoughInputs","Usage collisionmex(model_ptr,command,arg1,arg2,...");
      }

      vector<int> bodyA_idx, bodyB_idx;
       MatrixXd ptsA, ptsB, normals, JA, JB, Jd;
      VectorXd dist;
      model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB,normals,
                                         dist, JA, JB,Jd);
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
      if (nlhs>6) {
      	plhs[6] = mxCreateDoubleMatrix(JA.rows(),JA.cols(),mxREAL);
      	memcpy(mxGetPr(plhs[6]),JA.data(),sizeof(double)*JA.rows()*JA.cols());
      }
      if (nlhs>7) {
      	plhs[7] = mxCreateDoubleMatrix(JB.rows(),JB.cols(),mxREAL);
      	memcpy(mxGetPr(plhs[7]),JB.data(),sizeof(double)*JB.rows()*JB.cols());
      }
      if (nlhs>8) {
      	plhs[8] = mxCreateDoubleMatrix(Jd.rows(),Jd.cols(),mxREAL);
      	memcpy(mxGetPr(plhs[8]),Jd.data(),sizeof(double)*Jd.rows()*Jd.cols());
      }
    }
    break;
  default:
  	mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","unknown collision command");
  	break;
  
  }

}
