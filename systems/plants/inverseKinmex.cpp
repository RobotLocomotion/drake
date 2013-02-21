#include <mex.h>
#include <math.h>
#include <float.h>

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <f2c.h>
#include "snfilewrapper.hh"
#include "snopt.hh"
#include "snoptProblem.hh"
#undef max
#undef min

#include "Model.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the inverseKin function
 *
 */

Model* model=NULL;
VectorXd q0; 
VectorXd q_nom;
MatrixXd Q;
int narg;
int* body_ind;
VectorXd* pts;
VectorXd q;

MatrixXd zero;
VectorXd xpt;
        
int snoptIKfun( integer    *Status, integer *n,    doublereal x[],
        integer    *needF,  integer *neF,  doublereal F[],
        integer    *needG,  integer *neG,  doublereal G[],
        char       *cu,     integer *lencu,
        integer    iu[],    integer *leniu,
        doublereal ru[],    integer *lenru )
{
  memcpy(q.data(),x,sizeof(double)*q.rows());
  F[0] = ((q-q_nom).transpose()*Q*(q-q_nom)).coeff(0);
  MatrixXd tmp = 2*(q-q_nom).transpose()*Q;
  VectorXd xpt;
  MatrixXd J;
  memcpy(G,tmp.data(),sizeof(double)*q.rows());

  if (narg<1) return 0;        
  
  model->doKinematics(x,0);
  
  int i=0,j=1,k;
  bool do_rot;
  while (i<narg) {
    if (body_ind[i]==-1){
      do_rot = false;
      xpt = model->getCOM();
      J = model->getCOMJac();
    } else {
      do_rot = (pts[i].rows()==6);
      xpt = model->forwardKin(body_ind[i],zero,do_rot);
      J = model->forwardJac(body_ind[i],zero,do_rot);
    }

    for (k=0;k<xpt.rows();k++) {
      if (!isnan(pts[i](k))) {
        F[j] = xpt.coeff(k) - pts[i].coeff(k);
        memcpy(&G[j*q.rows()],J.row(k).data(),sizeof(double)*q.rows());
        j++;
      }
    }
    i++;
  }
  return 0;
}



void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if ((nrhs<4) || (nrhs%2!=0)) {
    mexErrMsgIdAndTxt("Drake:inverseKinmex:NotEnoughInputs","Usage [q,info]=inverseKinmex(model_ptr,q0,q_nom,Q,body1_ind,body1_pos,body2_ind,body2_pos,...) ");
  }

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:inverseKinmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));
  
  int nq = mxGetM(prhs[1]);

  plhs[0] = mxCreateDoubleMatrix(nq,1,mxREAL);
  memcpy(mxGetPr(plhs[0]),mxGetPr(prhs[1]),sizeof(double)*nq);
  
  q_nom.resize(nq);
  memcpy(q_nom.data(),mxGetPr(prhs[2]),sizeof(double)*nq);
  
  Q.resize(nq,nq);
  memcpy(Q.data(),mxGetPr(prhs[3]),sizeof(double)*nq*nq);

  int nF = 1, i,j,k;
  narg = (nrhs-4)/2;
  body_ind = new int[narg];
  pts = new VectorXd[narg];
  for (i=0; i<narg; i++) {
    body_ind[i] = ((int) mxGetScalar(prhs[4+2*i]))-1;
    pts[i] = VectorXd::Zero(mxGetNumberOfElements(prhs[4+2*i+1]));
    memcpy(pts[i].data(),mxGetPr(prhs[4+2*i+1]),sizeof(double)*pts[i].rows());
    for (j=0; j<pts[i].rows(); j++)
      if (!isnan(pts[i](j))) nF++; 
  }

  zero = MatrixXd::Zero(4,1); zero(3)=1;
  q.resize(nq);  
  
//  [q,F,info] = snopt(q0,obj.joint_limit_min,obj.joint_limit_max,zeros(nF,1),[inf;zeros(nF-1,1)],'snoptUserfun',[],[],[],iGfun',jGvar');
  
  snoptProblem snoptIKprob;
  
  snoptIKprob.setPrintFile("ik.out");
  snoptIKprob.setProblemSize(nq,nF);
  snoptIKprob.setObjective(0,0.0);
  snoptIKprob.setA(0,NULL,NULL,NULL);

  integer lenG   = nq*nF;
  integer *iGfun = new integer[lenG];
  integer *jGvar = new integer[lenG];
  k=0;
  for (i=0;i<nF;i++) {
    for (j=0;j<nq;j++) {
      iGfun[k]=i;
      jGvar[k++]=j;
    }
  }
  snoptIKprob.setG(lenG,iGfun,jGvar);
  
  doublereal *x      = mxGetPr(plhs[0]);
  doublereal *xlow   = model->joint_limit_min;
  doublereal *xupp   = model->joint_limit_max;
  doublereal *xmul   = new doublereal[nq];
  integer    *xstate = new    integer[nq];
  memset(xstate,0,sizeof(int)*nq);
  snoptIKprob.setX( x, xlow, xupp, xmul, xstate );

  doublereal *F      = new doublereal[nF];
  doublereal *Flow   = new doublereal[nF];
  memset(Flow,0,sizeof(double)*nF);
  doublereal *Fupp   = new doublereal[nF];
  memset(Fupp,0,sizeof(double)*nF);
  Fupp[0] = 1e20;
  doublereal *Fmul   = new doublereal[nF];
  integer    *Fstate = new integer[nF];
  memset(Fstate,0,sizeof(int)*nF);
  snoptIKprob.setF( F, Flow, Fupp, Fmul, Fstate );

  char names[8];
  snoptIKprob.setXNames(names, 1);
  snoptIKprob.setFNames(names, 1);
  snoptIKprob.setNeA(0);
  snoptIKprob.setNeG(lenG);
  snoptIKprob.setIntParameter( "Derivative option", 3 );
  
  snoptIKprob.setProbName("ik");
  snoptIKprob.setUserFun( snoptIKfun );

  snoptIKprob.solve(0);
  
/*
  if (nlhs>0) {
    VectorXd qzero = VectorXd::Zero(nq);
    plhs[0] = mxCreateDoubleMatrix(nF,1,mxREAL);
    plhs[1] = mxCreateDoubleMatrix(nq*nF,1,mxREAL);
    snoptusrfun( NULL, NULL, qzero.data(),
        NULL, NULL,  mxGetPr(plhs[0]),
        NULL, NULL,  mxGetPr(plhs[1]),
        NULL, NULL, NULL, NULL, NULL, NULL);
  }
*/  
  
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleScalar((double) 1);
  }  
  
  delete[] xmul; delete[] xstate;
  delete[] F; delete[] Flow; delete[] Fupp; delete[] Fmul; delete[] Fstate;
  delete[] iGfun;  delete[] jGvar;
  
  delete[] pts;
  delete[] body_ind;
}
