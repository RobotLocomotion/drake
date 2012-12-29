#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Model.h"
#include "math.h"

#define INF -2147483648

using namespace Eigen;
using namespace std;

/*
 * A C version of the doKinematics function
 *
 * Piggybacks on HandCmex.cpp to properly initialize and destroy models
 * Call with doKinematicsmex(q,b_compute_second_derivatives);
 */

 
Matrix3d rotz(double theta) {
	// returns 3D rotation matrix (about the z axis)
	Matrix3d M;
	double c=cos(theta); 
	double s=sin(theta);
	M << c,-s, 0,
		 s, c, 0,
		 0, 0, 1;
	return M;
} 

void Tjcalc(int pitch, double q, Matrix4d* TJ)
{
	*TJ = Matrix4d::Identity();
	if (pitch==0) { // revolute joint
  		(*TJ).topLeftCorner(3,3) = rotz(q);
  	}
	else if (pitch == INF) { // prismatic joint
  		(*TJ)(2,3) = q;
	}
	else { // helical joint
  		(*TJ).topLeftCorner(3,3) = rotz(q);
  		(*TJ)(2,3) = q*pitch;
  	}
}

void dTjcalc(int pitch, double q, Matrix4d* dTJ)
{
	double s=sin(q); 
	double c=cos(q);
  	if (pitch==0) { // revolute joint
  		*dTJ << -s,-c, 0, 0, 
  				 c,-s, 0, 0,
  				 0, 0, 0, 0,
  				 0, 0, 0, 0;
  	}
	else if (pitch == INF) { // prismatic joint
  		*dTJ <<  0, 0, 0, 0,
  				 0, 0, 0, 0,
  				 0, 0, 0, 1,
  				 0, 0, 0, 0;
	}
	else { // helical joint
  		*dTJ << -s,-c, 0, 0,
  				 c,-s, 0, 0,
  				 0, 0, 0, pitch,
  				 0, 0, 0, 0;
  	}
}

void ddTjcalc(int pitch, double q, Matrix4d* ddTJ)
{
  	double c = cos(q);
  	double s = sin(q);

  	if (pitch==0) { // revolute joint  	
  		*ddTJ << -c, s, 0, 0,
  				 -s,-c, 0, 0,
  				  0, 0, 0, 0,
  				  0, 0, 0, 0;
  	}
	else if (pitch == INF) { // prismatic joint
  		*ddTJ = Matrix4d::Zero();
	}
	else { // helical joint
  		*ddTJ << -c, s, 0, 0,
  				 -s,-c, 0, 0,
  				  0, 0, 0, 0,
  				  0, 0, 0, 0;
	}
}


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:NotEnoughInputs", "Usage doKinematicsmex(q,b_compute_second_derivatives)");
  }
  
  // first get the model_ptr back from matlab
  if (mxGetNumberOfElements(prhs[0])!=1)  
    mexErrMsgIdAndTxt("Drake:doKinematicsMex:BadInputs","first argument should be the model");

  mxArray* mex_model_ptr = mxGetProperty(prhs[0],0,"mex_model_ptr");
  if (!mex_model_ptr)  mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs","first argument should be the model class object");
  Model *model = NULL; memcpy(&model,mxGetData(mex_model_ptr),sizeof(model));
  
  double *q;
  if (mxGetNumberOfElements(prhs[1])!=model->NB)
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs", "q must be size %d x 1", model->NB);
  q = mxGetPr(prhs[1]);
  int b_compute_second_derivatives = (int) mxGetScalar(prhs[2]);
    
  int i,j,k;
  //Check against cached values for bodies[1];
  
  if (model->kinematicsInit) {
    bool skip = true;
    if (b_compute_second_derivatives && !model->secondDerivativesCached)
    	skip = false;
    for (i = 0; i < model->NB; i++) {
      if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
        skip = false;
        break;
      }
    }
    if (skip) {
      return;
    }
  }
  for (i = 0; i < model->NB + 1; i++) {
    int parent = model->bodies[i].parent;
    if (parent < 0) {
      model->bodies[i].T = model->bodies[i].Ttree;
      //dTdq, ddTdqdq initialized as all zeros
      
    } 
    else {
      double qi = q[model->bodies[i].dofnum];
      Matrix4d TJ, dTJ, ddTJ;
      Tjcalc(model->bodies[i].pitch,qi,&TJ);
      dTjcalc(model->bodies[i].pitch,qi,&dTJ);
      
      Matrix4d Tbinv, Tb;
      Tb = model->bodies[i].T_body_to_joint;
      Tbinv = Tb.inverse();

      model->bodies[i].T = model->bodies[parent].T * model->bodies[i].Ttree * Tbinv * TJ * Tb;
  
      /* 
       * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
       * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
       */
      
      model->bodies[i].dTdq = model->bodies[parent].dTdq * model->bodies[i].Ttree * Tbinv * TJ * Tb;

      MatrixXd tmp = model->bodies[parent].T * model->bodies[i].Ttree * Tbinv * dTJ * Tb;
      model->bodies[i].dTdq.row(model->bodies[i].dofnum) += tmp.row(0);
      model->bodies[i].dTdq.row(model->bodies[i].dofnum + model->NB) += tmp.row(1);
      model->bodies[i].dTdq.row(model->bodies[i].dofnum + 2*model->NB) += tmp.row(2);
      model->bodies[i].dTdq.row(model->bodies[i].dofnum + 3*model->NB) += tmp.row(3);
 
      if (b_compute_second_derivatives) {
        //ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        model->bodies[i].ddTdqdq = model->bodies[parent].ddTdqdq * model->bodies[i].Ttree * Tbinv * TJ * Tb;
        tmp = model->bodies[parent].dTdq * model->bodies[i].Ttree * Tbinv * dTJ * Tb;
        for (j = 0; j < 4*model->NB; j++) {
          model->bodies[i].ddTdqdq.row(4*model->NB*(model->bodies[i].dofnum) + j) += tmp.row(j);
        }
        
        for (j = 0; j < 4; j++) {
          for (k = 0; k < model->NB; k++) {
            model->bodies[i].ddTdqdq.row(model->bodies[i].dofnum + (4*k+j)*model->NB) += tmp.row(j*model->NB+k);
          }
        }
        
        ddTjcalc(model->bodies[i].pitch,qi,&ddTJ);
        tmp = model->bodies[parent].T*model->bodies[i].Ttree * Tbinv * ddTJ * Tb;      
        
        model->bodies[i].ddTdqdq.row(4*model->NB*(model->bodies[i].dofnum) + model->bodies[i].dofnum) += tmp.row(0);
        model->bodies[i].ddTdqdq.row(4*model->NB*(model->bodies[i].dofnum) + model->bodies[i].dofnum + model->NB) += tmp.row(1);
        model->bodies[i].ddTdqdq.row(4*model->NB*(model->bodies[i].dofnum) + model->bodies[i].dofnum + 2*model->NB) += tmp.row(2);
        model->bodies[i].ddTdqdq.row(4*model->NB*(model->bodies[i].dofnum) + model->bodies[i].dofnum + 3*model->NB) += tmp.row(3);
      }
    }
  }
  
  model->kinematicsInit = true;
  for (i = 0; i < model->NB; i++) {
    model->cached_q[i] = q[i];
  }
  model->secondDerivativesCached = b_compute_second_derivatives;
}
