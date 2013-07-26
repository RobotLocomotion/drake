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

  if (nrhs < 3) {
    mexErrMsgIdAndTxt("Drake:collisionmex:NotEnoughInputs","Usage collisionmex(model_ptr,command,arg1,arg2,...");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int command = (int) mxGetScalar(prhs[1]);
  switch (command)
  {
  case 1:  // pairwiseCollision()
    {
    	MatrixXd ptsA, ptsB, normals;
    	int body_indA = (int) mxGetScalar(prhs[2])-1, body_indB = (int) mxGetScalar(prhs[3])-1;
    	if (body_indA<0 || body_indA>=model->num_bodies || body_indB<0 || body_indB>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");
      model->getPairwiseCollision(body_indA,body_indB,ptsA,ptsB,normals);
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
    	MatrixXd ptsA, ptsB, normals;
    	Vector3d ptA, ptB, normal;
    	int body_indA = (int) mxGetScalar(prhs[2])-1, body_indB = (int) mxGetScalar(prhs[3])-1;
    	if (body_indA<0 || body_indA>=model->num_bodies || body_indB<0 || body_indB>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");

      double* pr = mxGetPr(prhs[4]);
      int num_pts = mxGetNumberOfElements(prhs[4]);
      
      ptsA.resize(3,num_pts);
      ptsB.resize(3,num_pts);
      normals.resize(3,num_pts);
      for (int i=0; i<num_pts; i++) {
        model->getPairwisePointCollision(body_indA,body_indB,(int)pr[i]-1,ptA,ptB,normal);
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
    	MatrixXd ptsA, ptsB, normals;
    	Vector3d ptA, ptB, normal;
    	int body_indA = (int) mxGetScalar(prhs[2])-1;
    	if (body_indA<0 || body_indA>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");

      double* pr = mxGetPr(prhs[3]);
      int num_pts = mxGetNumberOfElements(prhs[3]);
      
      ptsA.resize(3,num_pts);
      ptsB.resize(3,num_pts);
      normals.resize(3,num_pts);
      for (int i=0; i<num_pts; i++) {
        model->getPointCollision(body_indA,(int)pr[i]-1,ptA,ptB,normal);
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
    	Vector3d ptA, ptB, normal;
        double dist;
    	int body_indA = (int) mxGetScalar(prhs[2])-1;
        int body_indB = (int) mxGetScalar(prhs[3])-1;
    	if (body_indA<0 || body_indA>=model->num_bodies || body_indB<0 || body_indB>=model->num_bodies)
    		mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","body indices must be between 1 and num_bodies");
        model->getPairwiseClosestPoint(body_indA,body_indB,ptA,ptB,normal,dist);
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
  default:
  	mexErrMsgIdAndTxt("Drake:collisionmex:BadInputs","unknown collision command");
  	break;
  
  }

}
