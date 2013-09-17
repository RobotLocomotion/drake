#include <mex.h>
#include <math.h>
#include <float.h>

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <f2c.h>
#include "snopt.hh"
#undef max
#undef min

#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

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

MatrixXd zero;
VectorXd xpt;
        
int snoptIKfun( integer    *Status, integer *n,    doublereal x[],
        integer    *needF,  integer *neF,  doublereal F[],
        integer    *needG,  integer *neG,  doublereal G[],
        char       *cu,     integer *lencu,
        integer    iu[],    integer *leniu,
        doublereal ru[],    integer *lenru )
{
  Map<VectorXd> q(x,(*n));
  F[0] = ((q-q_nom).transpose()*Q*(q-q_nom)).coeff(0);
  MatrixXd tmp = 2*(q-q_nom).transpose()*Q;
  VectorXd xpt;
  MatrixXd J;
  memcpy(G,tmp.data(),sizeof(double)*(*n));
  
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
    J.transposeInPlace();
    
    for (k=0;k<xpt.rows();k++) {
      if (!isnan(pts[i](k))) {
        F[j] = xpt(k) - pts[i](k);
        memcpy(&G[j*(*n)],(J.col(k)).data(),sizeof(double)*(*n));
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
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  integer nq = mxGetM(prhs[1]);

  plhs[0] = mxCreateDoubleMatrix(nq,1,mxREAL);
  memcpy(mxGetPr(plhs[0]),mxGetPr(prhs[1]),sizeof(double)*nq);
  
  q_nom.resize(nq);
  memcpy(q_nom.data(),mxGetPr(prhs[2]),sizeof(double)*nq);
  
  Q.resize(nq,nq);
  memcpy(Q.data(),mxGetPr(prhs[3]),sizeof(double)*nq*nq);

  zero = MatrixXd::Zero(4,1); zero(3)=1;

  integer nF = 1;
  int i,j,k;
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

  
//  [q,F,info] = snopt(q0,obj.joint_limit_min,obj.joint_limit_max,zeros(nF,1),[inf;zeros(nF-1,1)],'snoptUserfun',[],[],[],iGfun',jGvar');

  integer    minrw, miniw, mincw;
  integer    lenrw = 20000, leniw = 10000, lencw = 500;
  doublereal rw[20000];
  integer    iw[10000];
  char       cw[8*500];

  integer    Cold = 0, Basis = 1, Warm = 2;

  doublereal *x      = mxGetPr(plhs[0]);
  doublereal *xlow   = model->joint_limit_min;
  doublereal *xupp   = model->joint_limit_max;
  doublereal *xmul   = new doublereal[nq];
  integer    *xstate = new    integer[nq];
  memset(xstate,0,sizeof(int)*nq);

  doublereal *F      = new doublereal[nF];
  doublereal *Flow   = new doublereal[nF];
  memset(Flow,0,sizeof(double)*nF);
  doublereal *Fupp   = new doublereal[nF];
  memset(Fupp,0,sizeof(double)*nF);
  Fupp[0] = 1e20;
  doublereal *Fmul   = new doublereal[nF];
  integer    *Fstate = new integer[nF];
  memset(Fstate,0,sizeof(int)*nF);

  doublereal ObjAdd=0;

  integer    INFO, ObjRow=1;

  integer    lenA = 0;
  integer lenG   = nq*nF;
  integer *iGfun = new integer[lenG];
  integer *jGvar = new integer[lenG];
  k=0;
  for (i=1;i<=nF;i++) {
    for (j=1;j<=nq;j++) {
      iGfun[k]=i;
      jGvar[k++]=j;
    }
  }

  integer    nxname = 1, nFname = 1, npname;
  char       xnames[1*8], Fnames[1*8];
  char       Prob[200];

  integer    iSpecs = -1,  spec_len;
  integer    iSumm  = -1;
  integer    iPrint = -1,  prnt_len;

  integer    nS, nInf;
  doublereal sInf;

  sninit_(&iPrint,&iSumm,cw,&lencw,iw,&leniw,rw,&lenrw,8*500);

  char strOpt[200] = "Derivative option";
  integer DerOpt=3, strOpt_len=strlen(strOpt);
  snseti_(strOpt,&DerOpt,&iPrint,&iSumm,&INFO,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);

  sprintf(Prob,"ik");

  snopta_
    ( &Cold, &nF, &nq, &nxname, &nFname,
      &ObjAdd, &ObjRow, Prob, snoptIKfun,
      NULL, NULL, &lenA, &lenA, NULL,
      iGfun, jGvar, &lenG, &lenG,
      xlow, xupp, xnames, Flow, Fupp, Fnames,
      x, xstate, xmul, F, Fstate, Fmul,
      &INFO, &mincw, &miniw, &minrw,
      &nS, &nInf, &sInf,
      cw, &lencw, iw, &leniw, rw, &lenrw,
      cw, &lencw, iw, &leniw, rw, &lenrw,
      npname, 8*nxname, 8*nFname,
      8*500, 8*500);

  if (nlhs>1) {
    plhs[1] = mxCreateDoubleScalar((double) INFO);
  }  
  
// for debugging:
/*  
  plhs[1] = mxCreateDoubleMatrix(nF,1,mxREAL);
  plhs[2] = mxCreateDoubleMatrix(lenG,1,mxREAL);
  snoptIKfun( NULL, &nq, x,
          NULL, &nF,  mxGetPr(plhs[1]),
          NULL, &lenG,  mxGetPr(plhs[2]),
          NULL, NULL, NULL, NULL, NULL, NULL);
*/  

  delete[] xmul; delete[] xstate;
  delete[] F; delete[] Flow; delete[] Fupp; delete[] Fmul; delete[] Fstate;
  delete[] iGfun;  delete[] jGvar;
  
  delete[] pts;
  delete[] body_ind;
}
