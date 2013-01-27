#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Model.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the forwardKin function
 *
 * Piggybacks on HandCpmex.cpp to properly initialize and destroy models
 * Call with doKinematicsmex(q,b_compute_second_derivatives);
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotEnoughInputs", "Usage forwardKinmex(model_ptr,body_index,pts,include_rotations)");
  }

  int n_pts = mxGetN(prhs[2]);
  int dim = mxGetM(prhs[2]);
  
//  if (dim != 2 && dim != 3)
//    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 2 or 3");
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs", "number of rows in pts must be 3");
  
  Model *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:forwardKinmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));
  
  int body_ind = (int) mxGetScalar(prhs[1]);
    
  MatrixXd pts_tmp = MatrixXd::Zero(dim,n_pts);  

  memcpy(pts_tmp.data(), mxGetPr(prhs[2]), sizeof(double)*dim*n_pts);
  
  mxLogical *include_rotations = mxGetLogicals(prhs[3]);
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);

	// NOTE: we're assuming an X-Y-Z convention was used to construct R
  MatrixXd R = model->bodies[body_ind].T.topLeftCorner(dim,dim);
  if (nlhs>0) {
	  MatrixXd x = model->bodies[body_ind].T.topLeftCorner(dim,dim+1)*pts;
  	if (*include_rotations) {
	    plhs[0] = mxCreateDoubleMatrix(2*dim,n_pts,mxREAL);
			Vector3d rpy;
    	rpy << atan2(R(2,1),R(2,2)), atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))), atan2(R(1,0),R(0,0));

		  MatrixXd X = MatrixXd::Zero(2*dim,n_pts);
		  X.block(0,0,3,n_pts) = x;
		  X.block(3,0,3,n_pts) = rpy.replicate(1,n_pts);
						
    	memcpy(mxGetPr(plhs[0]), X.data(), sizeof(double)*2*dim*n_pts);
		}
		else {
    	plhs[0] = mxCreateDoubleMatrix(dim,n_pts,mxREAL);
    	memcpy(mxGetPr(plhs[0]), x.data(), sizeof(double)*dim*n_pts);
		}
  }

  if (nlhs>1) {
    MatrixXd tmp = model->bodies[body_ind].dTdq.topLeftCorner(dim*model->NB,dim+1)*pts;
    MatrixXd Jt = Map<MatrixXd>(tmp.data(),model->NB,dim*n_pts);
    MatrixXd J = Jt.transpose();

  	if (*include_rotations) {
  		
  		/* 
       * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
       * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
       */

      VectorXd dR21_dq(model->NB),dR22_dq(model->NB),dR20_dq(model->NB),dR00_dq(model->NB),dR10_dq(model->NB);
    	for (int i=0; i<model->NB; i++) {
				dR21_dq(i) = model->bodies[body_ind].dTdq(2*model->NB+i,1);
				dR22_dq(i) = model->bodies[body_ind].dTdq(2*model->NB+i,2);
				dR20_dq(i) = model->bodies[body_ind].dTdq(2*model->NB+i,0);
				dR00_dq(i) = model->bodies[body_ind].dTdq(i,0);
				dR10_dq(i) = model->bodies[body_ind].dTdq(model->NB+i,0);
			}
    	double sqterm1 = R(2,1)*R(2,1) + R(2,2)*R(2,2);
      double sqterm2 = R(0,0)*R(0,0) + R(1,0)*R(1,0);

    	MatrixXd Jr = MatrixXd::Zero(3,model->NB);

    	Jr.block(0,0,1,model->NB) = ((R(2,2)*dR21_dq - R(2,1)*dR22_dq)/sqterm1).transpose(); 
			Jr.block(1,0,1,model->NB) = ((-sqrt(sqterm1)*dR20_dq + R(2,0)/sqrt(sqterm1)*(R(2,1)*dR21_dq + R(2,2)*dR22_dq) )/(R(2,0)*R(2,0) + R(2,1)*R(2,1) + R(2,2)*R(2,2))).transpose();
	  	Jr.block(2,0,1,model->NB)= ((R(0,0)*dR10_dq - R(1,0)*dR00_dq)/sqterm2).transpose();
			  	
		  MatrixXd Jfull = MatrixXd::Zero(2*dim*n_pts,model->NB);
		  for (int i=0; i<n_pts; i++) {
				Jfull.block(i*6,0,3,model->NB) = J.block(i*3,0,3,model->NB);
			 	Jfull.block(i*6+3,0,3,model->NB) = Jr;
			}

    	plhs[1] = mxCreateDoubleMatrix(2*dim*n_pts,model->NB,mxREAL);
    	memcpy(mxGetPr(plhs[1]), Jfull.data(), sizeof(double)*2*dim*n_pts*model->NB);
	}
	else {       
    	plhs[1] = mxCreateDoubleMatrix(dim*n_pts,model->NB,mxREAL);
    	memcpy(mxGetPr(plhs[1]), J.data(), sizeof(double)*dim*n_pts*model->NB);
	}
  }

  if (nlhs>2) {
  	if (*include_rotations)
  	    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotImplemented","Second derivatives of rotations are not implemented yet");
  	
    int i,j;
    MatrixXd dJ_reshaped = MatrixXd(model->NB, dim*n_pts*model->NB);
    for (i = 0; i < model->NB; i++) {
      MatrixXd tmp = model->bodies[body_ind].ddTdqdq.block(i*model->NB*(dim+1),0,dim*model->NB,dim+1)*pts;  //dim*NB x n_pts
      for (j = 0; j < n_pts; j++) {
        dJ_reshaped.block(i,j*dim*model->NB,1,dim*model->NB) = tmp.col(j).transpose();
      }
//       dJ_reshaped.row(i) << tmp.col(0).transpose(), tmp.col(1).transpose();
    }
    MatrixXd dJ_t = Map<MatrixXd>(dJ_reshaped.data(), model->NB*model->NB, dim*n_pts);
    
    MatrixXd dJ = dJ_t.transpose();
  
    plhs[2] = mxCreateDoubleMatrix(dim*n_pts,model->NB*model->NB,mxREAL);
    memcpy(mxGetPr(plhs[2]), dJ.data(), sizeof(double)*dim*n_pts*model->NB*model->NB);
  }
}
