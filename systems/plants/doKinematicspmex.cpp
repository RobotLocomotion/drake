#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "PlanarModel.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the doKinematics function
 *
 * Piggybacks on HandCpmex.cpp to properly initialize and destroy models
 * Call with doKinematicsmex(q,b_compute_second_derivatives);
 */

void Tjcalcp(int code, double q, Matrix3d* TJ)
{
  double c = cos(q);
  double s = sin(q);
  switch(code) {
    case 1:
      *TJ << c, -s, 0, s, c, 0, 0, 0, 1;
      break;
    case 2:
      *TJ << 1,0,q,0,1,0,0,0,1;
      break;
    case 3:
      *TJ << 1,0,0,0,1,q,0,0,1;
      break;
    default:      
      mexErrMsgIdAndTxt("Drake:doKinematicspmex:BadJointCode","unrecognised joint code");
  }
}

void dTjcalcp(int code, double q, Matrix3d* dTJ)
{
  double c = cos(q);
  double s = sin(q);
  switch(code) {
    case 1:
      *dTJ << -s, -c, 0, c, -s, 0, 0, 0, 0;
      break;
    case 2:
      *dTJ << 0, 0, 1, 0, 0, 0, 0, 0, 0;
      break;
    case 3:
      *dTJ << 0, 0, 0, 0, 0, 1, 0, 0, 0;
      break;
    default:      
      mexErrMsgIdAndTxt("Drake:doKinematicspmex:BadJointCode","unrecognised joint code");
  }
}

void ddTjcalcp(int code, double q, Matrix3d* dTJ)
{
  double c = cos(q);
  double s = sin(q);
  switch(code) {
    case 1:
      *dTJ << -c, s, 0, -s, -c, 0, 0, 0, 0;
      break;
    case 2:
    case 3:
      *dTJ = Matrix3d::Zero();
      break;
    default:      
      mexErrMsgIdAndTxt("Drake:doKinematicspmex:BadJointCode","unrecognised joint code");
  }
}

void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[] ) {
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:doKinematicspmex:NotEnoughInputs", "Usage doKinematicspmex(model_ptr,q,b_compute_second_derivatives)");
  }

  PlanarModel *model = NULL;
  
  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:doKinematicspmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));

  double *q;
  if (mxGetNumberOfElements(prhs[1])!=model->NB)
    mexErrMsgIdAndTxt("Drake:doKinematicspmex:BadInputs", "q must be size %d x 1", model->NB);
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
  
  Matrix3d TJ, dTJ, ddTJ, tmp;
  MatrixXd tmp2, tmp3;
  
  for (i = 0; i < model->NB + 1; i++) {
    int parent = model->bodies[i].parent;
    if (parent < 0) {
      model->bodies[i].T = model->bodies[i].Ttree;
      //dTdq, ddTdqdq initialized as all zeros
      
    } else {
      double qi = model->bodies[i].jsign*q[model->bodies[i].dofnum];
      
      Tjcalcp(model->bodies[i].jcode,qi,&TJ);
      dTjcalcp(model->bodies[i].jcode,qi,&dTJ);
      dTJ = model->bodies[i].jsign*dTJ;
      model->bodies[i].T = model->bodies[parent].T*model->bodies[i].Ttree*TJ;
                
      /* 
       * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
       * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
       */
      
      model->bodies[i].dTdq = model->bodies[parent].dTdq*model->bodies[i].Ttree*TJ;
      tmp = model->bodies[parent].T*model->bodies[i].Ttree*dTJ;
      model->bodies[i].dTdq.row(model->bodies[i].dofnum) += tmp.row(0);
      model->bodies[i].dTdq.row(model->bodies[i].dofnum + model->NB) += tmp.row(1);
      model->bodies[i].dTdq.row(model->bodies[i].dofnum + 2*model->NB) += tmp.row(2);
      if (b_compute_second_derivatives) {
        //ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        model->bodies[i].ddTdqdq = model->bodies[parent].ddTdqdq*model->bodies[i].Ttree*TJ;
        tmp2 = model->bodies[parent].dTdq*model->bodies[i].Ttree*dTJ;
        for (j = 0; j < 3*model->NB; j++) {
          model->bodies[i].ddTdqdq.row(3*model->NB*(model->bodies[i].dofnum) + j) += tmp2.row(j);
        }
        
        for (j = 0; j < 3; j++) {
          for (k = 0; k < model->NB; k++) {
            model->bodies[i].ddTdqdq.row(model->bodies[i].dofnum + (3*k+j)*model->NB) += tmp2.row(j*model->NB+k);
          }
        }
        
        ddTjcalcp(model->bodies[i].jcode,qi,&ddTJ);
        tmp3 = model->bodies[parent].T*model->bodies[i].Ttree*ddTJ;      
        
        model->bodies[i].ddTdqdq.row(3*model->NB*(model->bodies[i].dofnum) + model->bodies[i].dofnum) += tmp3.row(0);
        model->bodies[i].ddTdqdq.row(3*model->NB*(model->bodies[i].dofnum) + model->bodies[i].dofnum + model->NB) += tmp3.row(1);
        model->bodies[i].ddTdqdq.row(3*model->NB*(model->bodies[i].dofnum) + model->bodies[i].dofnum + 2*model->NB) += tmp3.row(2);
      }
    }
  }
  
  model->kinematicsInit = true;
  for (i = 0; i < model->NB; i++) {
    model->cached_q[i] = q[i];
  }
  model->secondDerivativesCached = b_compute_second_derivatives;
}
