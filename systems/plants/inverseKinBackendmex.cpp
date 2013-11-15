#include <mex.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>

#include <cstdio>
#include <cstring>
#include <iostream>

namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
#include "snoptProblem.hh"
}
#undef abs
#undef max
#undef min

#include "RigidBodyManipulator.h"
#include "constraint/Constraint.h"
#include "drakeUtil.h"
#include <Eigen/LU>

using namespace Eigen;
using namespace std;


RigidBodyManipulator* model = NULL;
SingleTimeKinematicConstraint** st_kc_array = NULL;
MultipleTimeKinematicConstraint** mt_kc_array = NULL;
QuasiStaticConstraint* qsc_ptr = NULL;
MatrixXd q_seed;
MatrixXd q_nom;
VectorXd q_nom_i;
MatrixXd q_sol;
MatrixXd Q;
MatrixXd Qa;
MatrixXd Qv;
bool qscActiveFlag;
double shrinkFactor;
snopt::integer nx;
snopt::integer nF;
int nC;
snopt::integer nG;
snopt::integer* nc_array;
snopt::integer* nG_array;
snopt::integer* nA_array;
int nq;
double *t = NULL;
double *t_samples = NULL; 
double* ti = NULL;
int num_st_kc;
int num_mt_kc;
int* mtkc_nc;
int nT;
int num_qsc_pts;
bool fixInitialState;
// The following variables are used in inverseKinSequence only
snopt::integer* qfree_idx;
snopt::integer* qdotf_idx;
snopt::integer* qdot0_idx;

VectorXd q0_fixed;
VectorXd qdot0_fixed;
snopt::integer qstart_idx;
snopt::integer num_qfree;
snopt::integer num_qdotfree;

MatrixXd velocity_mat;
MatrixXd velocity_mat_qd0;
MatrixXd velocity_mat_qdf;
MatrixXd accel_mat;
MatrixXd accel_mat_qd0;
MatrixXd accel_mat_qdf;

VectorXd* t_inbetween;
snopt::integer num_inbetween_tSamples;
MatrixXd* dqInbetweendqknot;
MatrixXd* dqInbetweendqd0;
MatrixXd* dqInbetweendqdf;
snopt::integer* qknot_qsamples_idx;

/* Remeber to delete this*/
snopt::integer nF_tmp;
snopt::integer nG_tmp;
snopt::integer nx_tmp;

void gevalNumerical(void (*func_ptr)(const VectorXd &, VectorXd &),const VectorXd &x, VectorXd &c, MatrixXd &dc,int order = 2)
{
  int nx = x.rows();
  (*func_ptr)(x,c);
  int nc = c.rows();
  dc.resize(nc,nx);
  double err = 1e-10;
  for(int i = 0;i<nx;i++)
  {
    VectorXd dx = VectorXd::Zero(nx);
    dx(i) = err;
    VectorXd c1(nc);
    (*func_ptr)(x+dx,c1);
    if(order == 1)
    {
      for(int j = 0;j<nc;j++)
      {
        dc(j,i) = (c1(j)-c(j))/err;
      }
    }
    else if(order == 2)
    {
      VectorXd c2(nc);
      (*func_ptr)(x-dx,c2);
      for(int j = 0;j<nc;j++)
      {
        dc(j,i) = (c1(j)-c2(j))/(2*err);
      }
    }
  }
}


void IK_constraint_fun(double* x,double* c, double* G)
{
  double* qsc_weights;
  if(qscActiveFlag)
  {
    qsc_weights = x+nq;
  }
  int nc_accum = 0;
  int ng_accum = 0;
  int nc;
  for(int i = 0;i<num_st_kc;i++)
  {
    nc = st_kc_array[i]->getNumConstraint(ti);
    VectorXd cnst(nc);
    MatrixXd dcnst(nc,nq);
    st_kc_array[i]->eval(ti,cnst,dcnst);
    memcpy(&c[nc_accum],cnst.data(),sizeof(double)*nc);
    memcpy(&G[ng_accum],dcnst.data(),sizeof(double)*nc*nq);
    nc_accum += nc;
    ng_accum += nc*nq;
  }
  if(qscActiveFlag)
  {
    int num_qsc_cnst = qsc_ptr->getNumConstraint(ti);
    VectorXd cnst(num_qsc_cnst-1);
    MatrixXd dcnst(num_qsc_cnst-1,nq+num_qsc_pts);
    qsc_ptr->eval(ti,qsc_weights,cnst,dcnst);
    memcpy(c+nc_accum,cnst.data(),sizeof(double)*(num_qsc_cnst-1));
    c[nc_accum+num_qsc_cnst-1] = 0.0;
    memcpy(G+ng_accum,dcnst.data(),sizeof(double)*dcnst.size());
    nc_accum += num_qsc_cnst;
    ng_accum += dcnst.size();
  }
} 

void IK_cost_fun(double* x, double &J, double* dJ)
{
  VectorXd q(nq);
  memcpy(q.data(),x,sizeof(double)*nq);
  VectorXd q_err = q-q_nom_i;
  J = q_err.transpose()*Q*q_err;
  VectorXd dJ_vec = 2*q_err.transpose()*Q;
  memcpy(dJ, dJ_vec.data(),sizeof(double)*nq);
}

int snoptIKfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *neF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *neG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru)
{
  double* q = x;
  model->doKinematics(q);
  IK_cost_fun(x,F[0],G);
  IK_constraint_fun(x,&F[1],&G[nq]);
  return 0;
}

void IKtraj_cost_fun(MatrixXd q,const VectorXd &qdot0,const VectorXd &qdotf,double &J,double* dJ)
{
  MatrixXd dJ_vec = MatrixXd::Zero(1,nq*(num_qfree+num_qdotfree));
  MatrixXd qdot(nq*nT,1);
  MatrixXd qddot(nq*nT,1);
  q.resize(nq*nT,1);
  qdot.block(0,0,nq,1) = qdot0;
  qdot.block(nq,0,nq*(nT-2),1) = velocity_mat*q+velocity_mat_qd0*qdot0+velocity_mat_qdf*qdotf;
  qdot.block(nq*(nT-1),0,nq,1) = qdotf;
  qddot = accel_mat*q+accel_mat_qd0*qdot0+accel_mat_qdf*qdotf;
  q.resize(nq,nT);
  qdot.resize(nq,nT);
  qddot.resize(nq,nT);
  MatrixXd q_diff = q.block(0,qstart_idx,nq,num_qfree)-q_nom.block(0,qstart_idx,nq,num_qfree);
  MatrixXd tmp1 = 0.5*Qa*qddot;
  MatrixXd tmp2 = tmp1.cwiseProduct(qddot);
  J = tmp2.sum();
  MatrixXd tmp3 = 0.5*Qv*qdot;
  MatrixXd tmp4 = tmp3.cwiseProduct(qdot);
  J += tmp4.sum();
  MatrixXd tmp5 = 0.5*Q*q_diff;
  MatrixXd tmp6 = tmp5.cwiseProduct(q_diff);
  J += tmp6.sum();
  MatrixXd dJdqd = 2*tmp3.block(0,1,nq,nT-2);//[dJdqd(2) dJdqd(3) dJdqd(nT-1)]
  dJdqd.resize(1,nq*(nT-2));
  dJ_vec.block(0,0,1,nq*num_qfree) = dJdqd*velocity_mat.block(0,nq*qstart_idx,(nT-2)*nq,nq*num_qfree);
  MatrixXd dJdqdiff = 2*tmp5;
  dJdqdiff.resize(1,nq*num_qfree);
  dJ_vec.block(0,0,1,nq*num_qfree) += dJdqdiff;
  MatrixXd dJdqdd = 2.0*tmp1;
  dJdqdd.resize(1,nq*nT);
  dJ_vec.block(0,0,1,nq*num_qfree) += dJdqdd*accel_mat.block(0,nq*qstart_idx,nq*nT,nq*num_qfree);
  MatrixXd dJdqdotf;
  dJdqdotf = dJdqdd*accel_mat_qdf+Qv*qdotf+dJdqd*velocity_mat_qdf;
  dJdqdotf.resize(1,nq);
  if(fixInitialState)
  {
    dJ_vec.block(0,nq*num_qfree,1,nq) = dJdqdotf;
  }
  else
  {
    MatrixXd dJdqdot0;
    dJdqdot0 = dJdqdd*accel_mat_qd0+Qv*qdot0+dJdqd*velocity_mat_qd0;
    dJdqdot0.resize(1,nq);
    dJ_vec.block(0,nq*num_qfree,1,nq) = dJdqdot0;
    dJ_vec.block(0,nq*num_qfree+nq,1,nq) = dJdqdotf;
  }
  memcpy(dJ,dJ_vec.data(),sizeof(double)*nq*(num_qfree+num_qdotfree));
}

int snoptIKtrajfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *neF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *neG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru)
{
  VectorXd qdotf = VectorXd::Zero(nq);
  VectorXd qdot0 = VectorXd::Zero(nq);
  MatrixXd q(nq,nT);
  for(int i = 0;i<nq;i++)
  {
    qdotf(i) = x[qdotf_idx[i]];
  }
  if(fixInitialState)
  {
    qdot0 = qdot0_fixed;
    q.col(0) = q0_fixed;
  }
  else
  {
    for(int i = 0;i<nq;i++)
    {
      qdot0(i) = x[qdot0_idx[i]];
    }
  }
  for(int i = 0;i<nq*num_qfree;i++)
  {
    q(nq*qstart_idx+i) = x[qfree_idx[i]];
  }

  IKtraj_cost_fun(q,qdot0,qdotf,F[0],G);
  int nf_cum = 1;
  int nG_cum = nq*(num_qfree+num_qdotfree);
  for(int i = qstart_idx;i<nT;i++)
  {
    double* qi;
    if(qscActiveFlag)
    {
      qi = x+(i-qstart_idx)*(nq+num_qsc_pts);
    }
    else
    {
      qi = x+(i-qstart_idx)*nq;
    }
    model->doKinematics(qi);
    ti = &t[i];
    IK_constraint_fun(qi,F+nf_cum,G+nG_cum);
    nf_cum += nc_array[i];
    nG_cum += nG_array[i];
  }
  MatrixXd q_inbetween = MatrixXd::Zero(nq,num_inbetween_tSamples);
  MatrixXd q_samples = MatrixXd::Zero(nq,num_inbetween_tSamples+nT);
  int inbetween_idx = 0;
  
  for(int i = 0;i<nT-1;i++)
  {
    q.resize(nq*nT,1);
    MatrixXd q_inbetween_block_tmp = dqInbetweendqknot[i]*q+dqInbetweendqd0[i]*qdot0+dqInbetweendqdf[i]*qdotf;
    q_inbetween_block_tmp.resize(nq,t_inbetween[i].size());  
    q_inbetween.block(0,inbetween_idx,nq,t_inbetween[i].size()) = q_inbetween_block_tmp; 
    q.resize(nq,nT);
    for(int j = 0;j<t_inbetween[i].size();j++)
    {
      double t_j = t_inbetween[i](j)+t[i];
      double* qi = q_inbetween.data()+(inbetween_idx+j)*nq;
      model->doKinematics(qi);
      for(int k = 0;k<num_st_kc;k++)
      {
        if(st_kc_array[k]->isTimeValid(&t_j))
        {
          int nc = st_kc_array[k]->getNumConstraint(&t_j);
          VectorXd c_k(nc);
          MatrixXd dc_k(nc,nq);
          st_kc_array[k]->eval(&t_j,c_k,dc_k);
          memcpy(F+nf_cum,c_k.data(),sizeof(double)*nc);
          MatrixXd dc_kdx = MatrixXd::Zero(nc,nq*(num_qfree+num_qdotfree));
          dc_kdx.block(0,0,nc,nq*num_qfree) = dc_k*dqInbetweendqknot[i].block(nq*j,nq*qstart_idx,nq,nq*num_qfree);
          if(!fixInitialState)
          {
            dc_kdx.block(0,nq*num_qfree,nc,nq) = dc_k*dqInbetweendqd0[i].block(nq*j,0,nq,nq);
            dc_kdx.block(0,nq*num_qfree+nq,nc,nq) = dc_k*dqInbetweendqdf[i].block(nq*j,0,nq,nq);
          }
          else
          {
            dc_kdx.block(0,nq*num_qfree,nc,nq) = dc_k*dqInbetweendqdf[i].block(nq*j,0,nq,nq);
          }
          memcpy(G+nG_cum,dc_kdx.data(),sizeof(double)*dc_kdx.size());
          nf_cum += nc;
          nG_cum += nc*nq*(num_qfree+num_qdotfree);
        }
      }
    }
    q_samples.col(inbetween_idx+i) = q.col(i);
    q_samples.block(0,inbetween_idx+i+1,nq,t_inbetween[i].size()) = q_inbetween.block(0,inbetween_idx,nq,t_inbetween[i].size());
    inbetween_idx += t_inbetween[i].size();
  }
  q_samples.col(nT+num_inbetween_tSamples-1) = q.col(nT-1);

  for(int i = 0;i<num_mt_kc;i++)
  {
    VectorXd mtkc_c(mtkc_nc[i]);
    MatrixXd mtkc_dc(mtkc_nc[i],nq*(num_qfree+num_inbetween_tSamples));
    mt_kc_array[i]->eval(t_samples+qstart_idx,num_qfree+num_inbetween_tSamples,q_samples.block(0,qstart_idx,nq,num_qfree+num_inbetween_tSamples),mtkc_c,mtkc_dc);
    memcpy(F+nf_cum,mtkc_c.data(),sizeof(double)*mtkc_nc[i]);
    MatrixXd mtkc_dc_dx = MatrixXd::Zero(mtkc_nc[i],nq*(num_qfree+num_qdotfree));
    for(int j = qstart_idx;j<nT;j++)
    {
      mtkc_dc_dx.block(0,(j-qstart_idx)*nq,mtkc_nc[i],nq) = mtkc_dc.block(0,(qknot_qsamples_idx[j]-qstart_idx)*nq,mtkc_nc[i],nq);
    }
    for(int j = 0;j<nT-1;j++)
    {
      MatrixXd dc_ij = mtkc_dc.block(0,nq*(qknot_qsamples_idx[j]+1-qstart_idx),mtkc_nc[i],nq*t_inbetween[j].size());
      mtkc_dc_dx.block(0,0,mtkc_nc[i],nq*num_qfree) += dc_ij*dqInbetweendqknot[j].block(0,qstart_idx*nq,nq*t_inbetween[j].size(),nq*num_qfree);
      if(fixInitialState)
      {
        mtkc_dc_dx.block(0,nq*num_qfree,mtkc_nc[i],nq) += dc_ij*dqInbetweendqdf[j];
      }
      else
      {
        mtkc_dc_dx.block(0,nq*num_qfree,mtkc_nc[i],nq) += dc_ij*dqInbetweendqd0[j];
        mtkc_dc_dx.block(0,nq*num_qfree+nq,mtkc_nc[i],nq) += dc_ij*dqInbetweendqdf[j];
      }
    }
    memcpy(G+nG_cum,mtkc_dc_dx.data(),sizeof(double)*mtkc_dc_dx.size());
    nf_cum += mtkc_nc[i];
    nG_cum += mtkc_nc[i]*nq*(num_qfree+num_qdotfree);
  }
  return 0;
}


void snoptIKtraj_userfun(const VectorXd &x_vec, VectorXd &c_vec, VectorXd &G_vec)
{
  snopt::doublereal* x = new snopt::doublereal[nx_tmp];
  for(int i = 0;i<nx_tmp;i++)
  {
    x[i] = x_vec(i);
  }
  snopt::doublereal* F = new snopt::doublereal[nF_tmp];
  snopt::doublereal* G = new snopt::doublereal[nG_tmp];
  snopt::integer Status = 0;
  snopt::integer n = nx_tmp;
  snopt::integer needF = 1;
  snopt::integer neF = nF_tmp;
  snopt::integer needG = 1;
  snopt::integer neG = nG_tmp;
  char *cu;
  snopt::integer lencu = 0;
  snopt::integer *iu;
  snopt::integer leniu = 0;
  snopt::doublereal *ru;
  snopt::integer lenru = 0;
  snoptIKtrajfun(&Status,&n,x,&needF, &neF, F, &needG, &neG, G, cu, &lencu, iu, &leniu, ru, &lenru);
  c_vec.resize(nF_tmp,1);
  for(int i = 0;i<nF_tmp;i++)
  {
    c_vec(i) = F[i];
  }
  G_vec.resize(nG_tmp,1);
  for(int i = 0;i<nG_tmp;i++)
  {
    G_vec(i) = G[i];
  }
  delete[] x;
  delete[] F;
  delete[] G;
}

void snoptIKtraj_fevalfun(const VectorXd &x, VectorXd &c)
{
  VectorXd G;
  snoptIKtraj_userfun(x,c,G);
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{
  if(nrhs<7)
  {
    mexErrMsgIdAndTxt("Drake:inverseKinBackendmex:NotEnoughInputs", "Usage varargout = inverseKinBackendmex(model_ptr,mode,t,q_seed,q_nom,constraint_ptr1,constraint_ptr2,...,IKoptions)");
  }
  model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  nq = model->num_dof;
  int mode = (int) mxGetScalar(prhs[1]);
  nT = mxGetNumberOfElements(prhs[2]);
  if(nT > 0)
  {
    t = new double[nT];
    memcpy(t,mxGetPr(prhs[2]),sizeof(double)*nT);
  }
  else if(nT == 0)
  {
    nT = 1;
    t = NULL;
  }
  q_seed.resize(nq,nT);
  q_nom.resize(nq,nT);
  if(mxGetM(prhs[3])!=nq ||mxGetN(prhs[3]) != nT)
  {
    mexErrMsgIdAndTxt("Drake:inverseKinBackendmex:BadInput","q_seed should be of size nq x nT");
  }
  if(mxGetM(prhs[4])!=nq || mxGetN(prhs[4]) != nT)
  {
    mexErrMsgIdAndTxt("Drake:inverseKinBackendmex:BadInput","q_nom should be of size nq x nT");
  }
  memcpy(q_seed.data(),mxGetPr(prhs[3]),sizeof(double)*nq*nT);
  memcpy(q_nom.data(),mxGetPr(prhs[4]),sizeof(double)*nq*nT);
  int num_constraint = nrhs-6;
  num_st_kc = 0;
  num_mt_kc = 0;
  st_kc_array = new SingleTimeKinematicConstraint*[num_constraint];
  mt_kc_array = new MultipleTimeKinematicConstraint*[num_constraint];
  qsc_ptr = NULL;
  MatrixXd joint_limit_min(nq,nT);
  MatrixXd joint_limit_max(nq,nT);
  for(int i = 0;i<nT;i++)
  {
    memcpy(joint_limit_min.data()+i*nq,model->joint_limit_min,sizeof(double)*nq);
    memcpy(joint_limit_max.data()+i*nq,model->joint_limit_max,sizeof(double)*nq);
  }
  for(int i = 0;i<num_constraint;i++)
  {
    Constraint* constraint = (Constraint*) getDrakeMexPointer(prhs[i+5]);
    DrakeConstraintType constraint_type = constraint->getType();
    if(constraint_type == DrakeConstraintType::SingleTimeKinematicConstraintType)
    {
      st_kc_array[num_st_kc] = (SingleTimeKinematicConstraint*) constraint;
      num_st_kc++;
    }
    else if(constraint_type == DrakeConstraintType::MultipleTimeKinematicConstraintType)
    {
      mt_kc_array[num_mt_kc] = (MultipleTimeKinematicConstraint*) constraint;
      num_mt_kc++;
    }
    else if(constraint_type == DrakeConstraintType::QuasiStaticConstraintType)
    {
      qsc_ptr = (QuasiStaticConstraint*) constraint;
    }
    else if(constraint_type == DrakeConstraintType::PostureConstraintType)
    {
      double* joint_min = new double[nq];
      double* joint_max = new double[nq];
      PostureConstraint* pc = (PostureConstraint*) constraint;
      for(int j = 0;j<nT;j++)
      {
        pc->bounds(&t[j],joint_min,joint_max);
        for(int k = 0;k<nq;k++)
        {
          joint_limit_min(k,j) = (joint_limit_min(k,j)>joint_min[k]? joint_limit_min(k,j):joint_min[k]);
          joint_limit_max(k,j) = (joint_limit_max(k,j)<joint_max[k]? joint_limit_max(k,j):joint_max[k]);
          if(joint_limit_min(k,j)>joint_limit_max(k,j))
          {
            mexErrMsgIdAndTxt("Drake:inverseKinBackendmex:BadInputs","Some posture constraint has lower bound larger than the upper bound of other posture constraint for joint %d at time %4.2f\n",k+1,&t[j]==NULL?0.0:t[j]);
          }
        }
      }
      delete[] joint_min;
      delete[] joint_max;
    }
  }
  if(qsc_ptr == NULL)
  {
    qscActiveFlag = false;
    num_qsc_pts = 0;
  }
  else
  {
    qscActiveFlag = qsc_ptr->isActive(); 
    num_qsc_pts = qsc_ptr->getNumWeights();
  }
  mxArray* pm;
  int ikoptions_idx = nrhs-1;
  pm = mxGetProperty(prhs[ikoptions_idx],0,"Q");
  Q.resize(nq,nq);
  memcpy(Q.data(),mxGetPr(pm),sizeof(double)*nq*nq);
  pm = mxGetProperty(prhs[ikoptions_idx],0,"SNOPT_MajorIterationsLimit");
  snopt::integer SNOPT_MajorIterationsLimit = (snopt::integer) mxGetScalar(pm);
  pm = mxGetProperty(prhs[ikoptions_idx],0,"SNOPT_IterationsLimit");
  snopt::integer SNOPT_IterationsLimit = (snopt::integer) mxGetScalar(pm);
  pm = mxGetProperty(prhs[ikoptions_idx],0,"SNOPT_MajorFeasibilityTolerance");
  double SNOPT_MajorFeasibilityTolerance = mxGetScalar(pm);
  pm = mxGetProperty(prhs[ikoptions_idx],0,"SNOPT_MajorOptimalityTolerance");
  double SNOPT_MajorOptimalityTolerance = mxGetScalar(pm);
  pm = mxGetProperty(prhs[ikoptions_idx],0,"SNOPT_SuperbasicsLimit");
  snopt::integer SNOPT_SuperbasicsLimit = (snopt::integer) mxGetScalar(pm);
  pm = mxGetProperty(prhs[ikoptions_idx],0,"debug_mode");
  bool* debug_mode_ptr = mxGetLogicals(pm);
  bool debug_mode = *debug_mode_ptr;
  pm = mxGetProperty(prhs[ikoptions_idx],0,"sequentialSeedFlag");
  bool* sequentialSeedFlag_ptr = mxGetLogicals(pm);
  bool sequentialSeedFlag = *sequentialSeedFlag_ptr;
  snopt::integer* INFO;
  double* INFO_tmp;
  mxArray* infeasible_constraint_cell;
  vector<string> infeasible_constraint_vec;
  if(mode == 1)
  {
    mwSize ret_dim[1] = {3};
    plhs[0] = mxCreateCellArray(1,ret_dim);
    INFO = new snopt::integer[nT]; 
    INFO_tmp = new double[nT];
    mwSize time_dim[1];  time_dim[0] = nT;
    for(int j = 0;j<nT;j++)
    {
      INFO[j] = 0;
    }
  }
  else if(mode == 2)
  {
    mwSize ret_dim[1] = {5};
    plhs[0] = mxCreateCellArray(1,ret_dim);
    INFO = new snopt::integer[1];
    INFO_tmp = new double[1];
    INFO[0] = 0;
  }
  mxArray* q_sol_ptr = mxCreateDoubleMatrix(nq,nT,mxREAL);
  mxArray* info_ptr;
  if(mode == 1)
  { 
    info_ptr = mxCreateDoubleMatrix(1,nT,mxREAL);
  }
  else
  {
    info_ptr = mxCreateDoubleMatrix(1,1,mxREAL);
  }
  q_sol.resize(nq,nT);
  memcpy(q_sol.data(),mxGetPr(prhs[3]),sizeof(double)*nq*nT);
  VectorXi* iCfun_array = new VectorXi[nT];
  VectorXi* jCvar_array = new VectorXi[nT];
  nc_array = new snopt::integer[nT];
  nG_array = new snopt::integer[nT];
  nA_array = new snopt::integer[nT];
  for(int i = 0;i<nT;i++)
  {
    nc_array[i] = 0;
    nG_array[i] = 0;
    nA_array[i] = 0;
  }
  VectorXd* A_array = new VectorXd[nT];
  VectorXi* iAfun_array = new VectorXi[nT];
  VectorXi* jAvar_array = new VectorXi[nT];
  VectorXd* Cmin_array = new VectorXd[nT];
  VectorXd* Cmax_array = new VectorXd[nT]; 
  vector<string>* Cname_array = new vector<string>[nT];
  for(int i =0;i<nT;i++)
  {
    Cmin_array[i].resize(0);
    Cmax_array[i].resize(0);
    iCfun_array[i].resize(0);
    jCvar_array[i].resize(0);
    A_array[i].resize(0);
    iAfun_array[i].resize(0);
    jAvar_array[i].resize(0);
  }
  for(int i = 0;i<nT;i++)
  {
    for(int j = 0;j<num_st_kc;j++)
    {
      if(st_kc_array[j]->isTimeValid(&t[i]))
      {
        int nc = st_kc_array[j]->getNumConstraint(&t[i]);
        VectorXd lb,ub;
        lb.resize(nc);
        ub.resize(nc);
        st_kc_array[j]->bounds(&t[i],lb,ub);
        Cmin_array[i].conservativeResize(Cmin_array[i].size()+nc);
        Cmin_array[i].tail(nc) = lb;
        Cmax_array[i].conservativeResize(Cmax_array[i].size()+nc);
        Cmax_array[i].tail(nc) = ub;
        iCfun_array[i].conservativeResize(iCfun_array[i].size()+nc*nq);
        jCvar_array[i].conservativeResize(jCvar_array[i].size()+nc*nq);
        VectorXi iCfun_append(nc);
        VectorXi jCvar_append(nc);
        for(int k = 0;k<nc;k++)
        {
          iCfun_append(k) = nc_array[i]+k+1;
        }
        for(int k = 0;k<nq;k++)
        {
          iCfun_array[i].segment(nG_array[i]+k*nc,nc) = iCfun_append;
          jCvar_append = VectorXi::Constant(nc,k+1);
          jCvar_array[i].segment(nG_array[i]+k*nc,nc) = jCvar_append;
        }
        nc_array[i] = nc_array[i]+nc;
        nG_array[i] = nG_array[i]+nq*nc;
        if(debug_mode)
        {
          vector<string> constraint_name;
          st_kc_array[j]->name(&t[i],constraint_name);
          Cname_array[i].insert(Cname_array[i].end(),constraint_name.begin(),constraint_name.end());
        }
      }
    }
    if(qscActiveFlag)
    {
      int num_qsc_cnst = qsc_ptr->getNumConstraint(&t[i]);
      iCfun_array[i].conservativeResize(iCfun_array[i].size()+(num_qsc_cnst-1)*(nq+num_qsc_pts));
      jCvar_array[i].conservativeResize(jCvar_array[i].size()+(num_qsc_cnst-1)*(nq+num_qsc_pts));
      iAfun_array[i].conservativeResize(iAfun_array[i].size()+num_qsc_pts);
      jAvar_array[i].conservativeResize(jAvar_array[i].size()+num_qsc_pts);
      A_array[i].conservativeResize(A_array[i].size()+num_qsc_pts);
      for(int k=0;k<nq+num_qsc_pts;k++)
      {
        for(int l = 0;l<num_qsc_cnst-1;l++)
        {
        iCfun_array[i](nG_array[i]+(num_qsc_cnst-1)*k+l) = nc_array[i]+l+1;
        jCvar_array[i](nG_array[i]+(num_qsc_cnst-1)*k+l) = k+1;
        }
      }
      iAfun_array[i].tail(num_qsc_pts) = VectorXi::Constant(num_qsc_pts,nc_array[i]+num_qsc_cnst);
      for(int k = 0;k<num_qsc_pts;k++)
      {
        jAvar_array[i](nA_array[i]+k) = nq+k+1;
      }
      A_array[i].tail(num_qsc_pts) = VectorXd::Ones(num_qsc_pts);
      Cmin_array[i].conservativeResize(Cmin_array[i].size()+num_qsc_cnst);
      Cmax_array[i].conservativeResize(Cmax_array[i].size()+num_qsc_cnst);
      VectorXd qsc_lb(num_qsc_cnst);
      VectorXd qsc_ub(num_qsc_cnst);
      VectorXd qsc_lb_tmp(num_qsc_cnst-1);
      VectorXd qsc_ub_tmp(num_qsc_cnst-1);
      qsc_ptr->bounds(&t[i],qsc_lb_tmp,qsc_ub_tmp);
      qsc_lb.head(num_qsc_cnst-1) = qsc_lb_tmp;
      qsc_lb(num_qsc_cnst-1) = 1.0;
      qsc_ub.head(num_qsc_cnst-1) = qsc_ub_tmp;
      qsc_ub(num_qsc_cnst-1) = 1.0;
      Cmin_array[i].tail(num_qsc_cnst) = qsc_lb;
      Cmax_array[i].tail(num_qsc_cnst) = qsc_ub;
      nc_array[i] += num_qsc_cnst;
      nG_array[i] += (num_qsc_cnst-1)*(nq+num_qsc_pts);
      nA_array[i] += num_qsc_pts;
      if(debug_mode)
      {
        vector<string> constraint_name;
        qsc_ptr->name(&t[i],constraint_name);
        for(int k = 0;k<num_qsc_cnst-1;k++)
        {
          Cname_array[i].push_back(constraint_name[k]);
        }
        string qsc_weights_cnst_name;
        if(&t[i]!= NULL)
        {
          char qsc_name_buffer[200];
          sprintf(qsc_name_buffer,"quasi static constraint weights at time %7.3f", t[i]);
          qsc_weights_cnst_name = string(qsc_name_buffer);
        }
        else
        {
          char qsc_name_buffer[200];
          sprintf(qsc_name_buffer,"quasi static constraint weights");
          qsc_weights_cnst_name = string(qsc_name_buffer);
        }
        Cname_array[i].push_back(qsc_weights_cnst_name);
      }
    }
    if(mode == 1)
    {
      if(!qscActiveFlag)
      {
        nx = nq;
      }
      else
      {
        nx = nq+num_qsc_pts;
      }
      nG = nq + nG_array[i];
      snopt::integer* iGfun = new snopt::integer[nG];
      snopt::integer* jGvar = new snopt::integer[nG];
      for(int k = 0;k<nq;k++)
      {
        iGfun[k] = 1;
        jGvar[k] = (snopt::integer) k+1;
      }
      for(int k = nq;k<nG;k++)
      {
        iGfun[k] = iCfun_array[i][k-nq]+1;
        jGvar[k] = jCvar_array[i][k-nq];
      }
      nF = nc_array[i]+1;

      snopt::integer lenA = A_array[i].size();
      snopt::integer* iAfun;
      snopt::integer* jAvar;
      snopt::doublereal* A;
      if(lenA == 0)
      {
        iAfun = NULL;
        jAvar = NULL;
        A = NULL;
      }
      else
      {
        A = new snopt::doublereal[lenA];
        iAfun = new snopt::integer[lenA];
        jAvar = new snopt::integer[lenA];
        for(int k = 0;k<lenA;k++)
        {
          A[k] = A_array[i](k);
          iAfun[k] = iAfun_array[i](k)+1;
          jAvar[k] = jAvar_array[i](k);
        }
      }
      snopt::doublereal* xlow = new snopt::doublereal[nx];
      snopt::doublereal* xupp = new snopt::doublereal[nx];
      for(int k = 0;k<nq;k++)
      {
        xlow[k] = joint_limit_min(k,i);
        xupp[k] = joint_limit_max(k,i);
      }
      //memcpy(xlow,joint_limit_min.col(i).data(),sizeof(double)*nq);
      //memcpy(xupp,joint_limit_max.col(i).data(),sizeof(double)*nq);
      if(qscActiveFlag)
      {
        for(int k = 0;k<num_qsc_pts;k++)
        {
          xlow[nq+k] = 0.0;
          xupp[nq+k] = 1.0;
        }
      }
      snopt::doublereal* Flow = new snopt::doublereal[nF];
      snopt::doublereal* Fupp = new snopt::doublereal[nF];
      Flow[0] = -mxGetInf();
      Fupp[0] = mxGetInf();
      for(int k = 0;k<nc_array[i];k++)
      {
        Flow[1+k] = Cmin_array[i](k);
        Fupp[1+k] = Cmax_array[i](k);
      }
      //memcpy(&Flow[1],Cmin_array[i].data(),sizeof(double)*nc_array[i]);
      //memcpy(&Fupp[1],Cmax_array[i].data(),sizeof(double)*nc_array[i]);
      ti = &t[i];
      q_nom_i = q_nom.col(i);
      snopt::doublereal* x = new snopt::doublereal[nx];
      if(!sequentialSeedFlag)
      {
        for(int k=0;k<nq;k++)
        {
          x[k] = q_seed(k,i);
        }
        //memcpy(x,q_seed.col(i).data(),sizeof(double)*nq);
      }
      else
      {
        if(i == 0)
        {
          for(int k = 0;k<nq;k++)
          {
            x[k] = q_seed(k,i);
          }
          //memcpy(x,q_seed.col(i).data(),sizeof(double)*nq);
        }
        else 
        {
          if(INFO[i-1]>10)
          {
            for(int k = 0;k<nq;k++)
            {
              x[k] = q_seed(k,i);
            }
            //memcpy(x,q_seed.col(i).data(),sizeof(double)*nq);
          }
          else
          {
            for(int k = 0;k<nq;k++)
            {
              x[k] = q_sol(k,i-1);
            }
            //memcpy(x,q_sol.col(i-1).data(),sizeof(double)*nq);
          }
        }
      }
      if(qscActiveFlag)
      {
        for(int j = 0;j<num_qsc_pts;j++)
        {
          x[nq+j] = 1.0/num_qsc_pts;
        }
      }

      snopt::integer minrw,miniw,mincw;
      snopt::integer lenrw = 10000000, leniw = 500000, lencw = 500;
      snopt::doublereal* rw;
      rw = (snopt::doublereal*) std::calloc(lenrw,sizeof(snopt::doublereal));
      //doublereal rw[lenrw];
      snopt::integer* iw;
      iw = (snopt::integer*) std::calloc(lenrw,sizeof(snopt::integer));
      char cw[8*lencw];

      snopt::integer Cold = 0, Basis = 1, Warm = 2;
      snopt::doublereal *xmul = new snopt::doublereal[nx];
      snopt::integer    *xstate = new snopt::integer[nx];
      for(int j = 0;j<nx;j++)
      {
        xstate[j] = 0;
      }
      snopt::doublereal *F      = new snopt::doublereal[nF];
      snopt::doublereal *Fmul   = new snopt::doublereal[nF];
      snopt::integer    *Fstate = new snopt::integer[nF];
      for(int j = 0;j<nF;j++)
      {
        Fstate[j] = 0;
      }
      snopt::doublereal ObjAdd = 0.0;

      snopt::integer ObjRow = 1;
      
      snopt::integer   nxname = 1, nFname = 1, npname;
      char* xnames = new char[nxname*8];
      char* Fnames = new char[nFname*8];
      char Prob[200];
      char printname[200];
      char specname[200];

      snopt::integer iSpecs = -1, spec_len;
      snopt::integer iSumm  = -1;
      snopt::integer iPrint = -1, prnt_len;

      snopt::integer nS, nInf;
      snopt::doublereal sInf;

      /*sprintf(specname, "%s","ik.spc");
      sprintf(printname, "%s","ik.out");
      sprintf(Prob,"%s","ik");
      prnt_len = strlen(printname);
      spec_len = strlen(specname);
      npname = strlen(Prob);
      snopenappend_(&iPrint,printname, &INFO[i], prnt_len);*/

      snopt::sninit_(&iPrint,&iSumm,cw,&lencw,iw,&leniw,rw,&lenrw,8*500);
      //snopt::snfilewrapper_(specname,&iSpecs,&INFO[i],cw,&lencw,iw,&leniw,rw,&lenrw,spec_len,8*lencw);
      char strOpt1[200] = "Derivative option";
      snopt::integer DerOpt = 1, strOpt_len = strlen(strOpt1);
      snopt::snseti_(strOpt1,&DerOpt,&iPrint,&iSumm,&INFO[i],cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
      char strOpt2[200] = "Major optimality tolerance";
      strOpt_len = strlen(strOpt2);
      snopt::snsetr_(strOpt2,&SNOPT_MajorOptimalityTolerance,&iPrint,&iSumm,&INFO[i],cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
      char strOpt3[200] = "Major feasibility tolerance";
      strOpt_len = strlen(strOpt3);
      snopt::snsetr_(strOpt3,&SNOPT_MajorFeasibilityTolerance,&iPrint,&iSumm,&INFO[i],cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
      char strOpt4[200] = "Superbasics limit";
      strOpt_len = strlen(strOpt4);
      snopt::snseti_(strOpt4,&SNOPT_SuperbasicsLimit,&iPrint,&iSumm,&INFO[i],cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
      char strOpt5[200] = "Major iterations limit";
      strOpt_len = strlen(strOpt5);
      snopt::snseti_(strOpt5,&SNOPT_MajorIterationsLimit,&iPrint,&iSumm,&INFO[i],cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
      char strOpt6[200] = "Iterations limit";
      strOpt_len = strlen(strOpt6);
      snopt::snseti_(strOpt6,&SNOPT_IterationsLimit,&iPrint,&iSumm,&INFO[i],cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
     

      //debug only
      /* 
      double* f = new double[nF];
      double* G = new double[nG];
      model->doKinematics(x);
      IK_cost_fun(x,f[0],G);
      IK_constraint_fun(x,&f[1],&G[nq]);
      mxArray* f_ptr = mxCreateDoubleMatrix(nF,1,mxREAL);
      mxArray* G_ptr = mxCreateDoubleMatrix(nG,1,mxREAL); 
      memcpy(mxGetPr(f_ptr),f,sizeof(double)*(nF));
      memcpy(mxGetPr(G_ptr),G,sizeof(double)*(nG));
      mxSetCell(plhs[0],0,f_ptr);
      mxSetCell(plhs[0],1,G_ptr);

      double* iGfun_tmp = new double[nG];
      double* jGvar_tmp = new double[nG];
      mxArray* iGfun_ptr = mxCreateDoubleMatrix(nG,1,mxREAL);
      mxArray* jGvar_ptr = mxCreateDoubleMatrix(nG,1,mxREAL);
      for(int k = 0;k<nG;k++)
      {
        iGfun_tmp[k] = (double) iGfun[k];
        jGvar_tmp[k] = (double) jGvar[k];
      } 
      memcpy(mxGetPr(iGfun_ptr),iGfun_tmp,sizeof(double)*nG);
      memcpy(mxGetPr(jGvar_ptr),jGvar_tmp,sizeof(double)*nG);
      mxSetCell(plhs[0],2,iGfun_ptr);
      mxSetCell(plhs[0],3,jGvar_ptr);

      mxArray* Fupp_ptr = mxCreateDoubleMatrix(nF,1,mxREAL);
      mxArray* Flow_ptr = mxCreateDoubleMatrix(nF,1,mxREAL);
      memcpy(mxGetPr(Fupp_ptr),Fupp,sizeof(double)*nF);
      memcpy(mxGetPr(Flow_ptr),Flow,sizeof(double)*nF);
      mxSetCell(plhs[0],4,Fupp_ptr);
      mxSetCell(plhs[0],5,Flow_ptr);

      mxArray* xupp_ptr = mxCreateDoubleMatrix(nx,1,mxREAL);
      mxArray* xlow_ptr = mxCreateDoubleMatrix(nx,1,mxREAL);
      memcpy(mxGetPr(xupp_ptr),xupp,sizeof(double)*nx);
      memcpy(mxGetPr(xlow_ptr),xlow,sizeof(double)*nx);
      mxSetCell(plhs[0],6,xupp_ptr);
      mxSetCell(plhs[0],7,xlow_ptr);

      mxArray* iAfun_ptr = mxCreateDoubleMatrix(lenA,1,mxREAL);
      mxArray* jAvar_ptr = mxCreateDoubleMatrix(lenA,1,mxREAL);
      mxArray* A_ptr = mxCreateDoubleMatrix(lenA,1,mxREAL);
      double* iAfun_tmp = new double[lenA];
      double* jAvar_tmp = new double[lenA];
      for(int k = 0;k<lenA;k++)
      {
        iAfun_tmp[k] = (double) iAfun[k];
        jAvar_tmp[k] = (double) jAvar[k];
      }
      memcpy(mxGetPr(iAfun_ptr),iAfun_tmp,sizeof(double)*lenA);
      memcpy(mxGetPr(jAvar_ptr),jAvar_tmp,sizeof(double)*lenA);
      memcpy(mxGetPr(A_ptr),A,sizeof(double)*lenA);
      mxSetCell(plhs[0],8,iAfun_ptr);
      mxSetCell(plhs[0],9,jAvar_ptr);
      mxSetCell(plhs[0],10,A_ptr);
      
      mxArray* nF_ptr = mxCreateDoubleScalar((double) nF);
      mxSetCell(plhs[0],11,nF_ptr);*/
      snopt::snopta_
        ( &Cold, &nF, &nx, &nxname, &nFname,
          &ObjAdd, &ObjRow, Prob, snoptIKfun,
          iAfun, jAvar, &lenA, &lenA, A,
          iGfun, jGvar, &nG, &nG,
          xlow, xupp, xnames, Flow, Fupp, Fnames,
          x, xstate, xmul, F, Fstate, Fmul,
          &INFO[i], &mincw, &miniw, &minrw,
          &nS, &nInf, &sInf,
          cw, &lencw, iw, &leniw, rw, &lenrw,
          cw, &lencw, iw, &leniw, rw, &lenrw,
          npname, 8*nxname, 8*nFname,
          8*500, 8*500);
      //snclose_(&iPrint);
      //snclose_(&iSpecs);
      vector<string> Fname(Cname_array[i]);
      if(debug_mode)
      {
        string objective_name("Objective");
        vector<string>::iterator it = Fname.begin();
        Fname.insert(it,objective_name);
      }
      if(INFO[i] == 13)
      {
        double *ub_err = new double[nF];
        double *lb_err = new double[nF];
        double max_lb_err = -mxGetInf();
        double max_ub_err = -mxGetInf();
        bool *infeasible_constraint_idx = new bool[nF];
        ub_err[0] = -mxGetInf();
        lb_err[0] = -mxGetInf();
        infeasible_constraint_idx[0] = false;
        for(int j = 1;j<nF;j++)
        {
          ub_err[j] = F[j]-Fupp[j];
          lb_err[j] = Flow[j]-F[j];
          if(ub_err[j]>max_ub_err)
            max_ub_err = ub_err[j];
          if(lb_err[j]>max_lb_err)
            max_lb_err = lb_err[j];
          infeasible_constraint_idx[j] = ub_err[j]>5e-5 | lb_err[j] > 5e-5;
        }
        max_ub_err = (max_ub_err>0.0? max_ub_err: 0.0);
        max_lb_err = (max_lb_err>0.0? max_lb_err: 0.0);
        if(max_ub_err+max_lb_err>1e-4)
        {
          INFO[i] = 13;
          if(debug_mode)
          {
            for(int j = 1;j<nF;j++)
            {
              if(infeasible_constraint_idx[j])
              {
                infeasible_constraint_vec.push_back(Fname[j]);
              }
            }
          }
        }
        else
        {
          INFO[i] = 4;
        }
        delete[] ub_err;
        delete[] lb_err;
        delete[] infeasible_constraint_idx;
      }
      memcpy(q_sol.col(i).data(),x,sizeof(double)*nq);
      INFO_tmp[i] = (double) INFO[i];
       
      
      delete[] xmul; delete[] xstate; delete[] xnames;
      delete[] F; delete[] Fmul; delete[] Fstate; delete[] Fnames;
      delete[] iGfun;  delete[] jGvar;
      if(lenA>0)
      {
        delete[] iAfun;  delete[] jAvar;  delete[] A;
      }
      delete[] x; delete[] xlow; delete[] xupp; delete[] Flow; delete[] Fupp;

    }
  }
  if(mode == 1)
  {
    memcpy(mxGetPr(q_sol_ptr),q_sol.data(),sizeof(double)*nq*nT);
    memcpy(mxGetPr(info_ptr),INFO_tmp,sizeof(double)*nT);
    mwSize name_dims[1] = {infeasible_constraint_vec.size()};
    infeasible_constraint_cell = mxCreateCellArray(1,name_dims);
    mxArray *name_ptr;
    for(int j = 0;j<infeasible_constraint_vec.size();j++)
    {
      name_ptr = mxCreateString(infeasible_constraint_vec[j].c_str());
      mxSetCell(infeasible_constraint_cell,j,name_ptr);
    }
    mxSetCell(plhs[0],0,q_sol_ptr);
    mxSetCell(plhs[0],1,info_ptr);
    mxSetCell(plhs[0],2,infeasible_constraint_cell);
  }
  if(mode == 2)
  {
    double* dt = new double[nT-1];
    for(int j = 0;j<nT-1;j++)
    {
      dt[j] = t[j+1]-t[j];
    }
    double* dt_ratio = new double[nT-2];
    for(int j = 0;j<nT-2;j++)
    {
      dt_ratio[j] = dt[j]/dt[j+1];
    }
    pm = mxGetProperty(prhs[ikoptions_idx],0,"Qa");
    Qa.resize(nq,nq);
    memcpy(Qa.data(),mxGetPr(pm),sizeof(double)*nq*nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"Qv");
    Qv.resize(nq,nq);
    memcpy(Qv.data(),mxGetPr(pm),sizeof(double)*nq*nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"q0_lb");
    VectorXd q0_lb(nq);
    memcpy(q0_lb.data(),mxGetPr(pm),sizeof(double)*nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"q0_ub");
    VectorXd q0_ub(nq);
    memcpy(q0_ub.data(),mxGetPr(pm),sizeof(double)*nq);
    VectorXd qd0_lb(nq);
    VectorXd qd0_ub(nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"qd0_lb");
    memcpy(qd0_lb.data(),mxGetPr(pm),sizeof(double)*nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"qd0_ub");
    memcpy(qd0_ub.data(),mxGetPr(pm),sizeof(double)*nq);
    VectorXd qd0_seed = (qd0_lb+qd0_ub)/2;
    VectorXd qdf_lb(nq);
    VectorXd qdf_ub(nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"qdf_lb");
    memcpy(qdf_lb.data(),mxGetPr(pm),sizeof(double)*nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"qdf_ub");
    memcpy(qdf_ub.data(),mxGetPr(pm),sizeof(double)*nq);
    VectorXd qdf_seed = (qdf_lb+qdf_ub)/2;
    pm = mxGetProperty(prhs[ikoptions_idx],0,"fixInitialState");
    bool* fixInitialState_ptr = mxGetLogicals(pm);
    fixInitialState = *fixInitialState_ptr;
    if(fixInitialState)
    {
      q0_fixed.resize(nq);
      q0_fixed = q_seed.col(0); 
      qdot0_fixed.resize(nq);
      qdot0_fixed = (qd0_lb+qd0_ub)/2;
      qstart_idx = 1;
      num_qfree = nT-1;
      num_qdotfree = 1;
    }
    else
    {
      q0_fixed.resize(0);
      qdot0_fixed.resize(0);
      qstart_idx = 0;
      num_qfree = nT;
      num_qdotfree = 2;
    }
    // This part can be rewritten using the sparse matrix if efficiency becomes a concern
    MatrixXd velocity_mat1 = MatrixXd::Zero(nq*nT,nq*nT);
    MatrixXd velocity_mat2 = MatrixXd::Zero(nq*nT,nq*nT);
    velocity_mat1.block(0,0,nq,nq) = MatrixXd::Identity(nq,nq);
    velocity_mat1.block(nq*(nT-1),nq*(nT-1),nq,nq) = MatrixXd::Identity(nq,nq);
    for(int j = 1;j<nT-1;j++)
    {
      double val_tmp1 = dt[j-1];
      double val_tmp2 = dt[j-1]*(2.0+2.0*dt_ratio[j-1]);
      double val_tmp3 = dt[j-1]*dt_ratio[j-1];
      double val_tmp4 = 3.0-3.0*dt_ratio[j-1]*dt_ratio[j-1];
      double val_tmp5 = 3.0-val_tmp4;
      for(int k = 0;k<nq;k++)
      {
        velocity_mat1(j*nq+k,(j-1)*nq+k) = val_tmp1;
        velocity_mat1(j*nq+k,j*nq+k) = val_tmp2;
        velocity_mat1(j*nq+k,(j+1)*nq+k) = val_tmp3;
        velocity_mat2(j*nq+k,(j-1)*nq+k) = -3.0;
        velocity_mat2(j*nq+k,j*nq+k) = val_tmp4;
        velocity_mat2(j*nq+k,(j+1)*nq+k) = val_tmp5;
      }
    }
    velocity_mat.resize(nq*(nT-2),nq*nT);
    velocity_mat = velocity_mat1.inverse().block(nq,0,nq*(nT-2),nq*nT)*velocity_mat2;

    MatrixXd velocity_mat1_middle_inv = (velocity_mat1.block(nq,nq,nq*(nT-2),nq*(nT-2))).inverse();
    velocity_mat_qd0 = -velocity_mat1_middle_inv*velocity_mat1.block(nq,0,nq*(nT-2),nq);
    velocity_mat_qdf = -velocity_mat1_middle_inv*velocity_mat1.block(nq,nq*(nT-1),nq*(nT-2),nq);

    MatrixXd accel_mat1 = MatrixXd::Zero(nq*nT,nq*nT);
    MatrixXd accel_mat2 = MatrixXd::Zero(nq*nT,nq*nT);
    for(int j = 0;j<nT-1;j++)
    {
      double val_tmp1 = -6.0/(dt[j]*dt[j]);
      double val_tmp2 = -val_tmp1;
      double val_tmp3 = -4.0/dt[j];
      double val_tmp4 = 0.5*val_tmp3;
      for(int k = 0;k<nq;k++)
      {
        accel_mat1(j*nq+k,j*nq+k) = val_tmp1;
        accel_mat1(j*nq+k,(j+1)*nq+k) = val_tmp2;
        accel_mat2(j*nq+k,j*nq+k) = val_tmp3;
        accel_mat2(j*nq+k,(j+1)*nq+k) = val_tmp4;
      }
    }
    for(int k = 0;k<nq;k++)
    {
      double val_tmp1 = 6.0/(dt[nT-2]*dt[nT-2]);
      double val_tmp2 = -val_tmp1;
      double val_tmp3 = 2.0/dt[nT-2];
      double val_tmp4 = 4.0/dt[nT-2];
      accel_mat1((nT-1)*nq+k,(nT-2)*nq+k) = val_tmp1;
      accel_mat1((nT-1)*nq+k,(nT-1)*nq+k) = val_tmp2;
      accel_mat2((nT-1)*nq+k,(nT-2)*nq+k) = val_tmp3;
      accel_mat2((nT-1)*nq+k,(nT-1)*nq+k) = val_tmp4;
    }
    accel_mat.resize(nq*nT,nq*nT);
    accel_mat = accel_mat1+accel_mat2.block(0,nq,nq*nT,nq*(nT-2))*velocity_mat;
    accel_mat_qd0.resize(nq*nT,nq);
    accel_mat_qd0 = accel_mat2.block(0,0,nq*nT,nq)+accel_mat2.block(0,nq,nq*nT,nq*(nT-2))*velocity_mat_qd0;
    accel_mat_qdf.resize(nq*nT,nq);
    accel_mat_qdf = accel_mat2.block(0,(nT-1)*nq,nT*nq,nq)+accel_mat2.block(0,nq,nq*nT,nq*(nT-2))*velocity_mat_qdf;

    qfree_idx = new snopt::integer[nq*num_qfree];
    qdotf_idx = new snopt::integer[nq];
    if(!fixInitialState)
    {
      qdot0_idx = new snopt::integer [nq];
    }
    else
    {
      qdot0_idx = NULL; 
    }
    int qdot_idx_start = 0;
    if(qscActiveFlag)
    {
      nx= nq*(num_qfree+num_qdotfree)+num_qsc_pts*num_qfree;
      for(int j = 0;j<num_qfree;j++)
      {
        for(int k = 0;k<nq;k++)
        {
          qfree_idx[j*nq+k] = j*(nq+num_qsc_pts)+k;
        }
      }
      qdot_idx_start = (nq+num_qsc_pts)*num_qfree;
    }
    else
    {
      nx = nq*(num_qfree+num_qdotfree);
      for(int j = 0;j<num_qfree*nq;j++)
      {
        qfree_idx[j] = j;
      }
      qdot_idx_start = nq*num_qfree;
    }
    if(!fixInitialState)
    {
      for(int j = 0;j<nq;j++)
      {
        qdot0_idx[j] = qdot_idx_start+j;
        qdotf_idx[j] = qdot_idx_start+nq+j;
      }
    }
    else
    {
      for(int j = 0;j<nq;j++)
      {
        qdotf_idx[j] = qdot_idx_start+j;
      }
    }
    snopt::doublereal* xlow = new snopt::doublereal[nx];
    snopt::doublereal* xupp = new snopt::doublereal[nx];
    for(int j = 0;j<num_qfree;j++)
    {
      if(qscActiveFlag)
      {
        for(int k = 0;k<nq;k++)
        {
          xlow[j*(nq+num_qsc_pts)+k] = joint_limit_min((j+qstart_idx)*nq+k);
          xupp[j*(nq+num_qsc_pts)+k] = joint_limit_max((j+qstart_idx)*nq+k);
        }
        //memcpy(xlow+j*(nq+num_qsc_pts),joint_limit_min.data()+(j+qstart_idx)*nq,sizeof(double)*nq);
        //memcpy(xupp+j*(nq+num_qsc_pts),joint_limit_max.data()+(j+qstart_idx)*nq,sizeof(double)*nq);
        for(int k = 0;k<num_qsc_pts;k++)
        {
          xlow[j*(nq+num_qsc_pts)+nq+k] = 0.0;
          xupp[j*(nq+num_qsc_pts)+nq+k] = 1.0;
        }
      }
      else
      {
        for(int k = 0;k<nq;k++)
        {
          xlow[j*nq+k] = joint_limit_min(k,j+qstart_idx);
          xupp[j*nq+k] = joint_limit_max(k,j+qstart_idx);
        }
        //memcpy(xlow+j*nq,joint_limit_min.col(j+qstart_idx).data(),sizeof(double)*nq);
        //memcpy(xupp+j*nq,joint_limit_max.col(j+qstart_idx).data(),sizeof(double)*nq);
      }
    }
    if(fixInitialState)
    {
      for(int k = 0; k<nq; k++)
      {
        xlow[qdotf_idx[0]+k] = qdf_lb(k);
        xupp[qdotf_idx[0]+k] = qdf_ub(k);
      }
      //memcpy(xlow+qdotf_idx[0],qdf_lb.data(),sizeof(double)*nq);
      //memcpy(xupp+qdotf_idx[0],qdf_ub.data(),sizeof(double)*nq);
    }
    else
    {
      for(int k = 0;k<nq;k++)
      {
        xlow[qdot0_idx[0]+k] = qd0_lb(k);
        xupp[qdot0_idx[0]+k] = qd0_ub(k);
        xlow[qdotf_idx[0]+k] = qdf_lb(k);
        xupp[qdotf_idx[0]+k] = qdf_ub(k);
      }
      //memcpy(xlow+qdot0_idx[0], qd0_lb.data(),sizeof(double)*nq);
      //memcpy(xupp+qdot0_idx[0], qd0_ub.data(),sizeof(double)*nq);
      //memcpy(xlow+qdotf_idx[0], qdf_lb.data(),sizeof(double)*nq);
      //memcpy(xupp+qdotf_idx[0], qdf_ub.data(),sizeof(double)*nq);
    }
    pm = mxGetProperty(prhs[ikoptions_idx],0,"additional_tSamples");
    set<double> t_set(t,t+nT);
    VectorXd inbetween_tSamples;
    inbetween_tSamples.resize(0);
    t_inbetween = new VectorXd[nT-1];
    for(int i = 0;i<nT-1;i++)
    {
      t_inbetween[i].resize(0);
    }
    {
      int num_inbetween_tSamples_tmp = mxGetNumberOfElements(pm);
      double* inbetween_tSamples_tmp = new double[num_inbetween_tSamples_tmp];
      memcpy(inbetween_tSamples_tmp,mxGetPr(pm),sizeof(double)*num_inbetween_tSamples_tmp);
      int knot_idx = 0;
      for(int i = 0;i<num_inbetween_tSamples_tmp;i++)
      {
        if(inbetween_tSamples_tmp[i]>t[0] && inbetween_tSamples_tmp[i]<t[nT-1] && t_set.find(inbetween_tSamples_tmp[i])==t_set.end())
        {
          inbetween_tSamples.conservativeResize(inbetween_tSamples.size()+1);
          inbetween_tSamples(inbetween_tSamples.size()-1) = (inbetween_tSamples_tmp[i]);
          {
            while(t[knot_idx+1]<inbetween_tSamples_tmp[i])
            {
              knot_idx++;
            }
            t_inbetween[knot_idx].conservativeResize(t_inbetween[knot_idx].size()+1);
            t_inbetween[knot_idx](t_inbetween[knot_idx].size()-1) = (inbetween_tSamples_tmp[i]-t[knot_idx]);
          }
        }
      }
      delete[] inbetween_tSamples_tmp;
    }
    num_inbetween_tSamples = inbetween_tSamples.size();
    qknot_qsamples_idx = new snopt::integer[nT];
    t_samples = new double[nT+num_inbetween_tSamples];
    {
      int t_samples_idx = 0;
      for(int i = 0;i<nT-1;i++)
      {
        qknot_qsamples_idx[i] = t_samples_idx;
        t_samples[t_samples_idx] = t[i];
        for(int j = 0;j<t_inbetween[i].size();j++)
        {
          t_samples[t_samples_idx+1+j] = t_inbetween[i](j)+t[i];
        }
        t_samples_idx += 1+t_inbetween[i].size();
      }
      qknot_qsamples_idx[nT-1] = nT+num_inbetween_tSamples-1;
      t_samples[nT+num_inbetween_tSamples-1] = t[nT-1];
    }

    dqInbetweendqknot = new MatrixXd[nT-1];
    dqInbetweendqd0 = new MatrixXd[nT-1];
    dqInbetweendqdf = new MatrixXd[nT-1];

    for(int i = 0;i<nT-1;i++)
    {
      VectorXd dt_ratio_inbetween_i = t_inbetween[i]/dt[i];
      int nt_sample_inbetween_i = t_inbetween[i].size();
      dqInbetweendqknot[i] = MatrixXd::Zero(nt_sample_inbetween_i*nq,nq*nT);
      dqInbetweendqdf[i] = MatrixXd::Zero(nt_sample_inbetween_i*nq,nq);
      dqInbetweendqdf[i] = MatrixXd::Zero(nt_sample_inbetween_i*nq,nq);
      MatrixXd dq_idqdknot_i = MatrixXd::Zero(nt_sample_inbetween_i*nq,nq);
      MatrixXd dq_idqdknot_ip1 = MatrixXd::Zero(nt_sample_inbetween_i*nq,nq);
      for(int j = 0;j<nt_sample_inbetween_i;j++)
      {
        double val1 = 1.0-3.0*pow(dt_ratio_inbetween_i[j],2)+2.0*pow(dt_ratio_inbetween_i[j],3);
        double val2 = 3.0*pow(dt_ratio_inbetween_i[j],2)-2.0*pow(dt_ratio_inbetween_i[j],3);
        double val3 = (1.0-2.0*dt_ratio_inbetween_i[j]+pow(dt_ratio_inbetween_i[j],2))*t_inbetween[i](j);
        double val4 = (pow(dt_ratio_inbetween_i[j],2)-dt_ratio_inbetween_i[j])*t_inbetween[i](j);
        for(int k = 0;k<nq;k++)
        {
          dqInbetweendqknot[i](j*nq+k,i*nq+k) = val1;
          dqInbetweendqknot[i](j*nq+k,(i+1)*nq+k) = val2;
          dq_idqdknot_i(j*nq+k,k) = val3;
          dq_idqdknot_ip1(j*nq+k,k) = val4;
        }
      }
      if(i>=1 && i<=nT-3)
      {
        dqInbetweendqknot[i] += dq_idqdknot_i*velocity_mat.block((i-1)*nq,0,nq,nq*nT)+dq_idqdknot_ip1*velocity_mat.block(i*nq,0,nq,nq*nT);
        dqInbetweendqd0[i] = dq_idqdknot_i*velocity_mat_qd0.block((i-1)*nq,0,nq,nq)+dq_idqdknot_ip1*velocity_mat_qd0.block(i*nq,0,nq,nq);
        dqInbetweendqdf[i] = dq_idqdknot_i*velocity_mat_qdf.block((i-1)*nq,0,nq,nq)+dq_idqdknot_ip1*velocity_mat_qdf.block(i*nq,0,nq,nq);
      }
      else if(i == 0 && i != nT-2)
      {
        dqInbetweendqknot[i] += dq_idqdknot_ip1*velocity_mat.block(i*nq,0,nq,nq*nT);
        dqInbetweendqd0[i] = dq_idqdknot_i+dq_idqdknot_ip1*velocity_mat_qd0.block(i*nq,0,nq,nq);
        dqInbetweendqdf[i] = dq_idqdknot_ip1*velocity_mat_qdf.block(i*nq,0,nq,nq);
      }
      else if(i == nT-2&& i != 0)
      {
        dqInbetweendqknot[i] += dq_idqdknot_i*velocity_mat.block((i-1)*nq,0,nq,nq*nT);
        dqInbetweendqd0[i] = dq_idqdknot_i*velocity_mat_qd0.block((i-1)*nq,0,nq,nq);
        dqInbetweendqdf[i] = dq_idqdknot_i*velocity_mat_qdf.block((i-1)*nq,0,nq,nq)+dq_idqdknot_ip1;
      }
      else if(i == 0 && i == nT-2)
      {
        dqInbetweendqd0[i] = dq_idqdknot_i;
        dqInbetweendqdf[i] = dq_idqdknot_ip1;
      }
    }

    snopt::integer* nc_inbetween_array = new snopt::integer[num_inbetween_tSamples];
    snopt::integer* nG_inbetween_array = new snopt::integer[num_inbetween_tSamples];
    VectorXd* Cmin_inbetween_array = new VectorXd[num_inbetween_tSamples];
    VectorXd* Cmax_inbetween_array = new VectorXd[num_inbetween_tSamples];
    VectorXi* iCfun_inbetween_array = new VectorXi[num_inbetween_tSamples];
    VectorXi* jCvar_inbetween_array = new VectorXi[num_inbetween_tSamples];
    vector<string>* Cname_inbetween_array = new vector<string>[num_inbetween_tSamples];
    for(int i = 0;i<num_inbetween_tSamples;i++)
    {
      nc_inbetween_array[i] = 0;
      nG_inbetween_array[i] = 0;
      Cmin_inbetween_array[i].resize(0);
      Cmax_inbetween_array[i].resize(0);
      iCfun_inbetween_array[i].resize(0);
      jCvar_inbetween_array[i].resize(0);
      for(int j = 0;j<num_st_kc;j++)
      {
        double* t_inbetween_ptr = inbetween_tSamples.data()+i;
        int nc = st_kc_array[j]->getNumConstraint(t_inbetween_ptr);
        VectorXd lb(nc,1);
        VectorXd ub(nc,1);
        st_kc_array[j]->bounds(t_inbetween_ptr,lb,ub);
        Cmin_inbetween_array[i].conservativeResize(Cmin_inbetween_array[i].size()+nc);
        Cmin_inbetween_array[i].tail(nc) = lb;
        Cmax_inbetween_array[i].conservativeResize(Cmax_inbetween_array[i].size()+nc);
        Cmax_inbetween_array[i].tail(nc) = ub;
        iCfun_inbetween_array[i].conservativeResize(iCfun_inbetween_array[i].size()+nc*nq*(num_qfree+num_qdotfree));
        jCvar_inbetween_array[i].conservativeResize(jCvar_inbetween_array[i].size()+nc*nq*(num_qfree+num_qdotfree));
        VectorXi iCfun_append = VectorXi::Zero(nc*nq*(num_qfree+num_qdotfree));
        VectorXi jCvar_append = VectorXi::Zero(nc*nq*(num_qfree+num_qdotfree));
        for(int k = 0;k<nq*(num_qfree+num_qdotfree);k++)
        {
          for(int l = 0;l<nc;l++)
          {
            iCfun_append(k*nc+l) = nc_inbetween_array[i]+l;            
          }
        }
        iCfun_inbetween_array[i].tail(nc*nq*(num_qfree+num_qdotfree)) = iCfun_append;
        for(int k = 0;k<nq*num_qfree;k++)
        {
          for(int l = 0;l<nc;l++)
          {
            jCvar_append(k*nc+l) = qfree_idx[k];
          }
        }
        if(fixInitialState)
        {
          for(int k = nq*num_qfree;k<nq*num_qfree+nq;k++)
          {
            for(int l = 0;l<nc;l++)
            {
              jCvar_append(k*nc+l) = qdotf_idx[k-nq*num_qfree];
            }
          }
        }
        else
        {
          for(int k = nq*num_qfree;k<nq*num_qfree+nq;k++)
          {
            for(int l = 0;l<nc;l++)
            {
              jCvar_append(k*nc+l) = qdot0_idx[k-nq*num_qfree];
            }
          }
          for(int k = nq*num_qfree+nq;k<nq*num_qfree+2*nq;k++)
          {
            for(int l = 0;l<nc;l++)
            {
              jCvar_append(k*nc+l) = qdotf_idx[k-nq*num_qfree-nq];
            }
          }
        }
        jCvar_inbetween_array[i].tail(nc*nq*(num_qfree+num_qdotfree)) = jCvar_append;
        nc_inbetween_array[i] += nc;
        nG_inbetween_array[i] += nc*nq*(num_qfree+num_qdotfree);
        if(debug_mode)
        {
          vector<string> constraint_name;
          st_kc_array[j]->name(t_inbetween_ptr,constraint_name);
          Cname_inbetween_array[i].insert(Cname_inbetween_array[i].end(),constraint_name.begin(),constraint_name.end());
        }
      }
    }

    nF = 1;
    nG = nq*(num_qfree+num_qdotfree);
    snopt::integer lenA = 0;
    for(int j = qstart_idx;j<nT;j++)
    {
      nF += nc_array[j]; 
      nG += nG_array[j];
      lenA += nA_array[j];
    } 
    for(int j = 0;j<num_inbetween_tSamples;j++)
    {
      nF += nc_inbetween_array[j];
      nG += nG_inbetween_array[j];
    }
    mtkc_nc = new int[num_mt_kc];
    for(int j = 0;j<num_mt_kc;j++)
    {
      mtkc_nc[j] = mt_kc_array[j]->getNumConstraint(t_samples+qstart_idx,num_qfree+num_inbetween_tSamples);
      nF += mtkc_nc[j];
      nG += mtkc_nc[j]*nq*(num_qfree+num_qdotfree);
    }
    snopt::doublereal* Flow = new snopt::doublereal[nF];
    snopt::doublereal* Fupp = new snopt::doublereal[nF];
    string* Fname = new string[nF];
    snopt::integer* iGfun = new snopt::integer[nG];
    snopt::integer* jGvar = new snopt::integer[nG];
    snopt::integer* iAfun;
    snopt::integer* jAvar;
    snopt::doublereal* A;
    if(lenA>0)
    {
      iAfun = new snopt::integer[lenA];
      jAvar = new snopt::integer[lenA];
      A = new snopt::doublereal[lenA];
    }
    else
    {
      snopt::integer* iAfun = NULL;
      jAvar = NULL;
      A = NULL;
    }
    Flow[0] = -mxGetInf();
    Fupp[0] = mxGetInf();
    if(debug_mode)
    {
      Fname[0] = string("Objective");
    }
    for(int j = 0;j<nq*num_qfree;j++)
    {
      iGfun[j] = 1;
      jGvar[j] = qfree_idx[j]+1;//C interface uses 1 index
    }
    for(int j = 0;j<nq;j++)
    {
      if(fixInitialState)
      {
        iGfun[j+nq*num_qfree] = 1;
        jGvar[j+nq*num_qfree] = qdotf_idx[j]+1;//C interface uses 1 index
      }
      else
      {
        iGfun[j+nq*num_qfree] = 1;
        jGvar[j+nq*num_qfree] = qdot0_idx[j]+1;
        iGfun[j+nq+nq*num_qfree] = 1;
        jGvar[j+nq+nq*num_qfree] = qdotf_idx[j]+1;
      }
    }

    snopt::integer nf_cum = 1;
    snopt::integer nG_cum = nq*(num_qfree+num_qdotfree);
    snopt::integer nA_cum = 0;
    int x_start_idx = 0;
    for(int j = qstart_idx;j<nT;j++)
    {
      for(int k = 0;k<nc_array[j];k++)
      {
        Flow[nf_cum+k] = Cmin_array[j](k);
        Fupp[nf_cum+k] = Cmax_array[j](k);
      }
      //memcpy(Flow+nf_cum,Cmin_array[j].data(),sizeof(double)*nc_array[j]);
      //memcpy(Fupp+nf_cum,Cmax_array[j].data(),sizeof(double)*nc_array[j]);
      if(debug_mode)
      {
        for(int k = 0;k<Cname_array[j].size();k++)
        {
          Fname[nf_cum+k] = Cname_array[j][k];
        }
      }
      for(int k = 0;k<nG_array[j];k++)
      {
        iGfun[nG_cum+k] = nf_cum+iCfun_array[j][k];
        jGvar[nG_cum+k] = x_start_idx+jCvar_array[j][k];
      }
      for(int k = 0;k<nA_array[j];k++)
      {
        iAfun[nA_cum+k] = nf_cum+iAfun_array[j][k];
        jAvar[nA_cum+k] = x_start_idx+jAvar_array[j][k];
        A[nA_cum+k] = A_array[j][k];
      }
      nf_cum += nc_array[j];
      nG_cum += nG_array[j];
      nA_cum += nA_array[j];
      if(qscActiveFlag)
      {
        x_start_idx += nq+num_qsc_pts;
      }
      else
      {
        x_start_idx += nq;
      }
    }
    // parse the inbetween samples constraints
    //
    for(int j = 0;j<num_inbetween_tSamples;j++)
    {
      for(int k = 0;k<nc_inbetween_array[j];k++)
      {
        Flow[nf_cum+k] = Cmin_inbetween_array[j](k);
        Fupp[nf_cum+k] = Cmax_inbetween_array[j](k);
      }
      //memcpy(Flow+nf_cum,Cmin_inbetween_array[j].data(),sizeof(double)*nc_inbetween_array[j]);
      //memcpy(Fupp+nf_cum,Cmax_inbetween_array[j].data(),sizeof(double)*nc_inbetween_array[j]);
      if(debug_mode)
      {
        for(int k = 0;k<nc_inbetween_array[j];k++)
        {
          Fname[nf_cum+k] = Cname_inbetween_array[j][k];
        }
      }
      for(int k = 0;k<nG_inbetween_array[j];k++)
      {
        iGfun[nG_cum+k] = nf_cum+iCfun_inbetween_array[j][k]+1;
        jGvar[nG_cum+k] = jCvar_inbetween_array[j][k]+1;
      }
      nf_cum += nc_inbetween_array[j];
      nG_cum += nG_inbetween_array[j];
    }
    //
    // Parse MultipleTimeKinematicConstraint.
    for(int j = 0;j<num_mt_kc;j++)
    {
      VectorXd mtkc_lb(mtkc_nc[j]);
      VectorXd mtkc_ub(mtkc_nc[j]);
      mt_kc_array[j]->bounds(t_samples+qstart_idx,num_qfree+num_inbetween_tSamples,mtkc_lb,mtkc_ub);
      for(int k = 0;k<mtkc_nc[j];k++)
      {
        Flow[nf_cum+k] = mtkc_lb(k);
        Fupp[nf_cum+k] = mtkc_ub(k);
      }
      //memcpy(Flow+nf_cum,mtkc_lb.data(),sizeof(double)*mtkc_nc[j]);
      //memcpy(Fupp+nf_cum,mtkc_ub.data(),sizeof(double)*mtkc_nc[j]);
      vector<string> mtkc_name;
      mt_kc_array[j]->name(t_samples+qstart_idx,num_qfree+num_inbetween_tSamples,mtkc_name);
      if(debug_mode)
      {
        for(int k = 0;k<mtkc_nc[j];k++)
        {
          Fname[nf_cum+k] = mtkc_name[k];
        }
      }
      for(int l = 0;l<nq*(num_qfree+num_qdotfree);l++)
      {
        for(int k = 0;k<mtkc_nc[j];k++)
        {
          iGfun[nG_cum+l*mtkc_nc[j]+k] = nf_cum+k+1;
        }
      }
      for(int l = 0;l<nq*num_qfree;l++)
      {
        for(int k = 0;k<mtkc_nc[j];k++)
        {
          jGvar[nG_cum+l*mtkc_nc[j]+k] = qfree_idx[l]+1;
        }
      }
      if(fixInitialState)
      {
        for(int l = nq*num_qfree;l<nq*num_qfree+nq;l++)
        {
          for(int k = 0;k<mtkc_nc[j];k++)
          {
            jGvar[nG_cum+l*mtkc_nc[j]+k] = qdotf_idx[l-nq*num_qfree]+1;
          }
        }
      }
      else
      {
        for(int l=nq*num_qfree;l<nq*num_qfree+nq;l++)
        {
          for(int k = 0;k<mtkc_nc[j];k++)
          {
            jGvar[nG_cum+l*mtkc_nc[j]+k] = qdot0_idx[l-nq*num_qfree]+1;
          }
        }
        for(int l = nq*num_qfree+nq;l<nq*num_qfree+2*nq;l++)
        {
          for(int k = 0;k<mtkc_nc[j];k++)
          {
            jGvar[nG_cum+l*mtkc_nc[j]+k] = qdotf_idx[l-nq*num_qfree-nq]+1;
          }
        }
      }
      nf_cum += mtkc_nc[j];
      nG_cum += mtkc_nc[j]*nq*(num_qfree+num_qdotfree);
    }
    snopt::doublereal* x = new snopt::doublereal[nx];
    if(qscActiveFlag)
    {
      for(int j = 0;j<num_qfree;j++)
      {
        for(int k = 0;k<nq;k++)
        {
          x[j*(nq+num_qsc_pts)+k] = q_seed((j+qstart_idx)*nq+k);
        }
        //memcpy(x+j*(nq+num_qsc_pts),q_seed.data()+(j+qstart_idx)*nq,sizeof(double)*nq);
        for(int k = 0;k<num_qsc_pts;k++)
        {
          x[j*(nq+num_qsc_pts)+nq+k] = 1.0/num_qsc_pts;
        }
      }
    }
    else
    {
      for(int j = 0;j<nq*num_qfree;j++)
      {
        x[j] = q_seed(nq*qstart_idx+j);
      }
      //memcpy(x,q_seed.data()+nq*qstart_idx,sizeof(double)*nq*num_qfree);
    }
    if(fixInitialState)
    {
      for(int j = 0;j<nq;j++)
      {
        x[qdotf_idx[0]+j] = qdf_seed(j);
      }
      //memcpy(x+qdotf_idx[0],qdf_seed.data(),sizeof(double)*nq);
    }
    else
    {
      for(int j = 0;j<nq;j++)
      {
        x[qdot0_idx[0]+j] = qd0_seed(j);
        x[qdotf_idx[0]+j] = qdf_seed(j);
      }
      //memcpy(x+qdot0_idx[0],qd0_seed.data(),sizeof(double)*nq);
      //memcpy(x+qdotf_idx[0],qdf_seed.data(),sizeof(double)*nq);
    }
    snopt::integer minrw,miniw,mincw;
    snopt::integer lenrw = 20000000, leniw = 2000000, lencw = 5000;
    snopt::doublereal* rw;
    rw = (snopt::doublereal*) std::calloc(lenrw,sizeof(snopt::doublereal));
    snopt::integer* iw;
    iw = (snopt::integer*) std::calloc(leniw,sizeof(snopt::integer));
    char cw[8*lencw];

    snopt::integer Cold = 0, Basis = 1, Warm = 2;
    snopt::doublereal *xmul = new snopt::doublereal[nx];
    snopt::integer    *xstate = new snopt::integer[nx];
    for(int j = 0;j<nx;j++)
    {
      xstate[j] = 0;
    }

    snopt::doublereal *F      = new snopt::doublereal[nF];
    snopt::doublereal *Fmul   = new snopt::doublereal[nF];
    snopt::integer    *Fstate = new snopt::integer[nF];
    for(int j = 0;j<nF;j++)
    {
      Fstate[j] = 0;
    }

    snopt::doublereal ObjAdd = 0.0;

    snopt::integer ObjRow = 1;
    
    snopt::integer   nxname = 1, nFname = 1, npname;
    char* xnames = new char[nxname*8];
    char* Fnames = new char[nFname*8];
    char Prob[200];

    snopt::integer iSpecs = -1, spec_len;
    snopt::integer iSumm = -1;
    snopt::integer iPrint = -1, prnt_len;


    snopt::integer nS, nInf;
    snopt::doublereal sInf;

    snopt::sninit_(&iPrint,&iSumm,cw,&lencw,iw,&leniw,rw,&lenrw,8*500);
    char strOpt1[200] = "Derivative option";
    snopt::integer DerOpt = 1, strOpt_len = strlen(strOpt1);
    snopt::snseti_(strOpt1,&DerOpt,&iPrint,&iSumm,INFO,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
    char strOpt2[200] = "Major optimality tolerance";
    strOpt_len = strlen(strOpt2);
    snopt::snsetr_(strOpt2,&SNOPT_MajorOptimalityTolerance,&iPrint,&iSumm,INFO,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
    char strOpt3[200] = "Major feasibility tolerance";
    strOpt_len = strlen(strOpt3);
    snopt::snsetr_(strOpt3,&SNOPT_MajorFeasibilityTolerance,&iPrint,&iSumm,INFO,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
    char strOpt4[200] = "Superbasics limit";
    strOpt_len = strlen(strOpt4);
    snopt::snseti_(strOpt4,&SNOPT_SuperbasicsLimit,&iPrint,&iSumm,INFO,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
    char strOpt5[200] = "Major iterations limit";
    strOpt_len = strlen(strOpt5);
    snopt::snseti_(strOpt5,&SNOPT_MajorIterationsLimit,&iPrint,&iSumm,INFO,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
    char strOpt6[200] = "Iterations limit";
    strOpt_len = strlen(strOpt6);
    snopt::snseti_(strOpt6,&SNOPT_IterationsLimit,&iPrint,&iSumm,INFO,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*500);
    //debug only
    /*nx_tmp = nx;
    nG_tmp = nG;
    nF_tmp = nF;
    mexPrintf("start to debug\n"); 
    VectorXd f_vec(nF,1);
    VectorXd G_vec(nG,1);
    VectorXd x_vec(nx,1);
    for(int i = 0;i<nx;i++)
    {
      x_vec(i) = x[i];
    }
    snoptIKtraj_userfun(x_vec,f_vec,G_vec);
    mxArray* f_ptr = mxCreateDoubleMatrix(nF,1,mxREAL);
    mxArray* G_ptr = mxCreateDoubleMatrix(nG,1,mxREAL); 
    memcpy(mxGetPr(f_ptr),f_vec.data(),sizeof(double)*(nF));
    memcpy(mxGetPr(G_ptr),G_vec.data(),sizeof(double)*(nG));
    mxSetCell(plhs[0],0,f_ptr);
    mxSetCell(plhs[0],1,G_ptr);
    mexPrintf("got f,G\n");
    
    double* iGfun_tmp = new double[nG];
    double* jGvar_tmp = new double[nG];
    mxArray* iGfun_ptr = mxCreateDoubleMatrix(nG,1,mxREAL);
    mxArray* jGvar_ptr = mxCreateDoubleMatrix(nG,1,mxREAL);
    for(int k = 0;k<nG;k++)
    {
      iGfun_tmp[k] = (double) iGfun[k];
      jGvar_tmp[k] = (double) jGvar[k];
    } 
    memcpy(mxGetPr(iGfun_ptr),iGfun_tmp,sizeof(double)*nG);
    memcpy(mxGetPr(jGvar_ptr),jGvar_tmp,sizeof(double)*nG);
    mxSetCell(plhs[0],2,iGfun_ptr);
    mxSetCell(plhs[0],3,jGvar_ptr);
    mexPrintf("got iGfun jGar\n");
    mxArray* Fupp_ptr = mxCreateDoubleMatrix(nF,1,mxREAL);
    memcpy(mxGetPr(Fupp_ptr),Fupp,sizeof(double)*nF);
    mxSetCell(plhs[0],4,Fupp_ptr);
    mxArray* Flow_ptr = mxCreateDoubleMatrix(nF,1,mxREAL);
    memcpy(mxGetPr(Flow_ptr),Flow,sizeof(double)*nF);
    mxSetCell(plhs[0],5,Flow_ptr);
    mexPrintf("got Fupp Flow\n");
    mxArray* xupp_ptr = mxCreateDoubleMatrix(nx,1,mxREAL);
    mxArray* xlow_ptr = mxCreateDoubleMatrix(nx,1,mxREAL);
    memcpy(mxGetPr(xupp_ptr),xupp,sizeof(double)*nx);
    memcpy(mxGetPr(xlow_ptr),xlow,sizeof(double)*nx);
    mxSetCell(plhs[0],6,xupp_ptr);
    mxSetCell(plhs[0],7,xlow_ptr);
    mexPrintf("got xupp xlow\n");
    mxArray* iAfun_ptr = mxCreateDoubleMatrix(lenA,1,mxREAL);
    mxArray* jAvar_ptr = mxCreateDoubleMatrix(lenA,1,mxREAL);
    mxArray* A_ptr = mxCreateDoubleMatrix(lenA,1,mxREAL);
    double* iAfun_tmp = new double[lenA];
    double* jAvar_tmp = new double[lenA];
    for(int k = 0;k<lenA;k++)
    {
      iAfun_tmp[k] = (double) iAfun[k];
      jAvar_tmp[k] = (double) jAvar[k];
    }
    memcpy(mxGetPr(iAfun_ptr),iAfun_tmp,sizeof(double)*lenA);
    memcpy(mxGetPr(jAvar_ptr),jAvar_tmp,sizeof(double)*lenA);
    memcpy(mxGetPr(A_ptr),A,sizeof(double)*lenA);
    mxSetCell(plhs[0],8,iAfun_ptr);
    mxSetCell(plhs[0],9,jAvar_ptr);
    mxSetCell(plhs[0],10,A_ptr);
    mexPrintf("got iAfun jAvar A\n"); 
    mxArray* velocity_mat_ptr = mxCreateDoubleMatrix(nq*(nT-2),nq*nT,mxREAL);
    memcpy(mxGetPr(velocity_mat_ptr),velocity_mat.data(),sizeof(double)*nq*(nT-2)*nq*nT);
    mxSetCell(plhs[0],11,velocity_mat_ptr);
    mxArray* velocity_mat_qd0_ptr = mxCreateDoubleMatrix(nq*(nT-2),nq,mxREAL);
    memcpy(mxGetPr(velocity_mat_qd0_ptr),velocity_mat_qd0.data(),sizeof(double)*velocity_mat_qd0.size());
    mxSetCell(plhs[0],12,velocity_mat_qd0_ptr);
    mxArray* velocity_mat_qdf_ptr = mxCreateDoubleMatrix(nq*(nT-2),nq,mxREAL);
    memcpy(mxGetPr(velocity_mat_qdf_ptr),velocity_mat_qdf.data(),sizeof(double)*velocity_mat_qdf.size());
    mxSetCell(plhs[0],13,velocity_mat_qdf_ptr);
    mxArray* accel_mat_ptr = mxCreateDoubleMatrix(nq*nT,nq*nT,mxREAL);
    memcpy(mxGetPr(accel_mat_ptr),accel_mat.data(),sizeof(double)*accel_mat.size());
    mxSetCell(plhs[0],14,accel_mat_ptr);
    mxArray* accel_mat_qd0_ptr = mxCreateDoubleMatrix(nq*nT,nq,mxREAL);
    memcpy(mxGetPr(accel_mat_qd0_ptr),accel_mat_qd0.data(),sizeof(double)*accel_mat_qd0.size());
    mxSetCell(plhs[0],15,accel_mat_qd0_ptr);
    mxArray* accel_mat_qdf_ptr = mxCreateDoubleMatrix(nq*nT,nq,mxREAL);
    memcpy(mxGetPr(accel_mat_qdf_ptr),accel_mat_qdf.data(),sizeof(double)*accel_mat_qdf.size());
    mxSetCell(plhs[0],16,accel_mat_qdf_ptr);
    mxArray** dqInbetweendqknot_ptr = new mxArray*[nT-1];
    mxArray** dqInbetweendqd0_ptr = new mxArray*[nT-1];
    mxArray** dqInbetweendqdf_ptr = new mxArray*[nT-1];
    mwSize cell_dim[1] = {nT-1};
    mxArray* dqInbetweendqknot_cell = mxCreateCellArray(1,cell_dim);
    mxArray* dqInbetweendqd0_cell = mxCreateCellArray(1,cell_dim);
    mxArray* dqInbetweendqdf_cell = mxCreateCellArray(1,cell_dim);
    for(int i = 0;i<nT-1;i++)
    {
      dqInbetweendqknot_ptr[i] = mxCreateDoubleMatrix(nq*t_inbetween[i].size(),nq*nT,mxREAL); 
      dqInbetweendqd0_ptr[i] = mxCreateDoubleMatrix(nq*t_inbetween[i].size(),nq,mxREAL); 
      dqInbetweendqdf_ptr[i] = mxCreateDoubleMatrix(nq*t_inbetween[i].size(),nq,mxREAL); 
      memcpy(mxGetPr(dqInbetweendqknot_ptr[i]),dqInbetweendqknot[i].data(),sizeof(double)*dqInbetweendqknot[i].size());
      memcpy(mxGetPr(dqInbetweendqd0_ptr[i]),dqInbetweendqd0[i].data(),sizeof(double)*dqInbetweendqd0[i].size());
      memcpy(mxGetPr(dqInbetweendqdf_ptr[i]),dqInbetweendqdf[i].data(),sizeof(double)*dqInbetweendqdf[i].size());
      mxSetCell(dqInbetweendqknot_cell,i,dqInbetweendqknot_ptr[i]);
      mxSetCell(dqInbetweendqd0_cell,i,dqInbetweendqd0_ptr[i]);
      mxSetCell(dqInbetweendqdf_cell,i,dqInbetweendqdf_ptr[i]);
    }
    mxSetCell(plhs[0],17,dqInbetweendqknot_cell);
    mxSetCell(plhs[0],18,dqInbetweendqd0_cell);
    mxSetCell(plhs[0],19,dqInbetweendqdf_cell);
    delete[] dqInbetweendqknot_ptr; delete[] dqInbetweendqd0_ptr; delete[] dqInbetweendqdf_ptr;
    mexPrintf("get dqInbetweendqknot\n");
    mxArray** iCfun_inbetween_ptr = new mxArray*[num_inbetween_tSamples];
    mxArray** jCvar_inbetween_ptr = new mxArray*[num_inbetween_tSamples];
    cell_dim[0] = num_inbetween_tSamples;
    mxArray* iCfun_inbetween_cell = mxCreateCellArray(1,cell_dim);
    mxArray* jCvar_inbetween_cell = mxCreateCellArray(1,cell_dim);
    for(int i = 0;i<num_inbetween_tSamples;i++)
    {
      iCfun_inbetween_ptr[i] = mxCreateDoubleMatrix(nG_inbetween_array[i],1,mxREAL); 
      jCvar_inbetween_ptr[i] = mxCreateDoubleMatrix(nG_inbetween_array[i],1,mxREAL); 
      double* iCfun_inbetween_i_double = new double[nG_inbetween_array[i]];
      double* jCvar_inbetween_i_double = new double[nG_inbetween_array[i]];
      for(int j = 0;j<nG_inbetween_array[i];j++)
      {
        iCfun_inbetween_i_double[j] = (double) iCfun_inbetween_array[i](j)+1;
        jCvar_inbetween_i_double[j] = (double) jCvar_inbetween_array[i](j)+1;
      }
      memcpy(mxGetPr(iCfun_inbetween_ptr[i]),iCfun_inbetween_i_double,sizeof(double)*nG_inbetween_array[i]);
      memcpy(mxGetPr(jCvar_inbetween_ptr[i]),jCvar_inbetween_i_double,sizeof(double)*nG_inbetween_array[i]);
      mxSetCell(iCfun_inbetween_cell,i,iCfun_inbetween_ptr[i]);
      mxSetCell(jCvar_inbetween_cell,i,jCvar_inbetween_ptr[i]);
      delete[] iCfun_inbetween_i_double; delete[] jCvar_inbetween_i_double;
    }
    mxSetCell(plhs[0],20,iCfun_inbetween_cell);
    mxSetCell(plhs[0],21,jCvar_inbetween_cell);
    delete[] iCfun_inbetween_ptr; delete[] jCvar_inbetween_ptr; 
    mexPrintf("get iCfun_inbetween\n");*/

   
    snopt::snopta_
      ( &Cold, &nF, &nx, &nxname, &nFname,
        &ObjAdd, &ObjRow, Prob, snoptIKtrajfun,
        iAfun, jAvar, &lenA, &lenA, A,
        iGfun, jGvar, &nG, &nG,
        xlow, xupp, xnames, Flow, Fupp, Fnames,
        x, xstate, xmul, F, Fstate, Fmul,
        INFO, &mincw, &miniw, &minrw,
        &nS, &nInf, &sInf,
        cw, &lencw, iw, &leniw, rw, &lenrw,
        cw, &lencw, iw, &leniw, rw, &lenrw,
        npname, 8*nxname, 8*nFname,
        8*500, 8*500);
    if(*INFO == 41)
    {
      nx_tmp = nx;
      nG_tmp = nG;
      nF_tmp = nF;
      MatrixXd df_numerical(nF,nx);
      VectorXd x_vec(nx_tmp);
      memcpy(x_vec.data(),x,sizeof(double)*nx);
      VectorXd c_vec(nF_tmp);
      VectorXd G_vec(nG_tmp);
      snoptIKtraj_userfun(x_vec,c_vec,G_vec);
      MatrixXd df_userfun = MatrixXd::Zero(nF,nx);
      for(int i = 0;i<nG;i++)
      {
        df_userfun(iGfun[i]-1,jGvar[i]-1) = G_vec(i);
      }
      gevalNumerical(&snoptIKtraj_fevalfun,x_vec,c_vec,df_numerical);
      MatrixXd df_err = df_userfun-df_numerical;
      df_err = df_err.cwiseAbs();
      int max_err_row,max_err_col;
      double max_err;
      max_err = df_err.maxCoeff(&max_err_row,&max_err_col);
      mexPrintf("The maximum gradient numerical error is %e, in row %d, col %d\nuser gradient is %e\n2nd order numerical gradient is %e\n",max_err,max_err_row+1,max_err_col+1,df_userfun(max_err_row,max_err_col),df_numerical(max_err_row,max_err_col));
      MatrixXd df_numerical2;
      gevalNumerical(&snoptIKtraj_fevalfun,x_vec,c_vec,df_numerical2,1);
      mexPrintf("1st order numerical gradient is %e\n",df_numerical2(max_err_row,max_err_col));
      double err = 1e-10;
      VectorXd x_vec_err(nx_tmp);
      x_vec_err = VectorXd::Zero(nx_tmp);
      x_vec_err(max_err_col) = 1e-10;
      VectorXd G_vec1(nG_tmp);
      VectorXd G_vec2(nG_tmp);
      MatrixXd df_err2 = df_err.block(1,0,nF-1,nx);
      mexPrintf("The maximum gradient numerical error, except in the cost function, is %e\n",df_err2.maxCoeff());
    }
    MatrixXd q(nq*nT,1);
    VectorXd qdot0(nq);
    VectorXd qdotf(nq);
    if(fixInitialState)
    {
      q.block(0,0,nq,1) = q0_fixed;
    }
    for(int j = 0;j<nq*num_qfree;j++)
    {
      q(j+nq*qstart_idx) = x[qfree_idx[j]];
    }
    for(int j = 0;j<nq;j++)
    {
      qdotf(j) = x[qdotf_idx[j]];
    } 
    if(!fixInitialState)
    {
      for(int j = 0;j<nq;j++)
      {
        qdot0(j) = x[qdot0_idx[j]];
      }
    }
    else
    {
      qdot0 = qdot0_fixed;
    }
    MatrixXd qdot(nq*nT,1);
    MatrixXd qddot(nq*nT,1);
    qdot.block(0,0,nq,1) = qdot0;
    qdot.block(nq*(nT-1),0,nq,1) = qdotf;
    qdot.block(nq,0,nq*(nT-2),1) = velocity_mat*q;
    qddot = accel_mat*q+accel_mat_qd0*qdot0+accel_mat_qdf*qdotf;
    q.resize(nq,nT);
    qdot.resize(nq,nT);
    qddot.resize(nq,nT);

    if(*INFO == 13)
    {
      double *ub_err = new double[nF];
      double *lb_err = new double[nF];
      double max_lb_err = -mxGetInf();
      double max_ub_err = -mxGetInf();
      bool *infeasible_constraint_idx = new bool[nF];
      ub_err[0] = -mxGetInf();
      lb_err[0] = -mxGetInf();
      infeasible_constraint_idx[0] = false;
      for(int j = 1;j<nF;j++)
      {
        ub_err[j] = F[j]-Fupp[j];
        lb_err[j] = Flow[j]-F[j];
        if(ub_err[j]>max_ub_err)
          max_ub_err = ub_err[j];
        if(lb_err[j]>max_lb_err)
          max_lb_err = lb_err[j];
        infeasible_constraint_idx[j] = ub_err[j]>5e-5 | lb_err[j] > 5e-5;
      }
      max_ub_err = (max_ub_err>0.0? max_ub_err: 0.0);
      max_lb_err = (max_lb_err>0.0? max_lb_err: 0.0);
      if(max_ub_err+max_lb_err>1e-4)
      {
        *INFO = 13;
        if(debug_mode)
        {
          for(int j = 1;j<nF;j++)
          {
            if(infeasible_constraint_idx[j])
            {
              infeasible_constraint_vec.push_back(Fname[j]);
            }
          }
        }
      }
      else
      {
        *INFO = 4;
      }
      delete[] ub_err;
      delete[] lb_err;
      delete[] infeasible_constraint_idx;
    }

    mwSize name_dim[1] = {infeasible_constraint_vec.size()};
    mxArray* infeasible_constraint_cell = mxCreateCellArray(1,name_dim);
    for(int j = 0;j<infeasible_constraint_vec.size();j++)
    {
      mxArray* name_ptr = mxCreateString(infeasible_constraint_vec[j].c_str());
      mxSetCell(infeasible_constraint_cell,j,name_ptr);
    }

    memcpy(mxGetPr(q_sol_ptr),q.data(),sizeof(double)*nq*nT);
    mxSetCell(plhs[0],0,q_sol_ptr);
    mxArray* qdot_ptr = mxCreateDoubleMatrix(nq,nT,mxREAL);
    memcpy(mxGetPr(qdot_ptr),qdot.data(),sizeof(double)*nq*nT);
    mxSetCell(plhs[0],1,qdot_ptr);
    mxArray* qddot_ptr = mxCreateDoubleMatrix(nq,nT,mxREAL);
    memcpy(mxGetPr(qddot_ptr),qddot.data(),sizeof(double)*nq*nT);
    mxSetCell(plhs[0],2,qddot_ptr);
    info_ptr = mxCreateDoubleMatrix(1,1,mxREAL);
    *INFO_tmp = (double) *INFO;
    memcpy(mxGetPr(info_ptr),INFO_tmp,sizeof(double));
    mxSetCell(plhs[0],3,info_ptr);
    mxSetCell(plhs[0],4,infeasible_constraint_cell);
    delete rw;
    delete[] xmul; delete[] xstate; delete[] xnames; 
    delete[] F; delete[] Fmul; delete[] Fstate; delete[] Fnames;
    delete[] iGfun;  delete[] jGvar;
    if(!fixInitialState)
    {
      delete[] qdot0_idx;
    }
    if(lenA>0)
    {
      delete[] iAfun;  delete[] jAvar;  delete[] A;
    }
    delete[] x; delete[] xlow; delete[] xupp; delete[] Flow; delete[] Fupp; delete[] Fname;
    delete[] dt; delete[] dt_ratio;  
    delete[] t_inbetween; delete[] t_samples; delete[] dqInbetweendqknot; delete[] dqInbetweendqd0; delete[] dqInbetweendqdf;
    delete[] nc_inbetween_array; delete[] nG_inbetween_array; delete[] Cmin_inbetween_array; delete[] Cmax_inbetween_array;
    delete[] iCfun_inbetween_array; delete[] jCvar_inbetween_array;
    delete[] qknot_qsamples_idx;
  } 
  delete[] INFO; delete[] INFO_tmp;
  delete[] iAfun_array; delete[] jAvar_array; delete[] A_array;
  if(mode == 2)
  {
    delete[] qfree_idx; delete[] qdotf_idx;
    delete[] mtkc_nc;
  }
  delete[] iCfun_array; delete[] jCvar_array; 
  delete[] Cmin_array; delete[] Cmax_array; delete[] Cname_array;
  delete[] nc_array; delete[] nG_array; delete[] nA_array;
  if(t!=NULL)
  {
    delete[] t;
  }
  delete[] st_kc_array;
  delete[] mt_kc_array;
}


