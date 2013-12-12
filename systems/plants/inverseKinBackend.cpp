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

#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "constraint/RigidBodyConstraint.h"
#include "IKoptions.h"
#include "inverseKinBackend.h"

#include <Eigen/LU>

using namespace Eigen;
using namespace std;

static RigidBodyManipulator* model = NULL;
static SingleTimeKinematicConstraint** st_kc_array = NULL;
static MultipleTimeKinematicConstraint** mt_kc_array = NULL;
static QuasiStaticConstraint* qsc_ptr = NULL;
static MatrixXd q_seed;
static MatrixXd q_nom;
static VectorXd q_nom_i;
static MatrixXd q_sol;
static MatrixXd Q;
static MatrixXd Qa;
static MatrixXd Qv;
static bool qscActiveFlag;
static double shrinkFactor;
static snopt::integer nx;
static snopt::integer nF;
static int nC;
static snopt::integer nG;
static snopt::integer* nc_array;
static snopt::integer* nG_array;
static snopt::integer* nA_array;
static int nq;
static double *t = NULL;
static double *t_samples = NULL; 
static double* ti = NULL;
static int num_st_kc;
static int num_mt_kc;
static int num_st_lpc;
static int num_mt_lpc;
static int* mt_kc_nc;
static int* st_lpc_nc;
static int* mt_lpc_nc;
static int nT;
static int num_qsc_pts;
static bool fixInitialState;
static // The following variables are used in inverseKinTraj only
static snopt::integer* qfree_idx;
static snopt::integer* qdotf_idx;
static snopt::integer* qdot0_idx;

static VectorXd q0_fixed;
static VectorXd qdot0_fixed;
static snopt::integer qstart_idx;
static snopt::integer num_qfree;
static snopt::integer num_qdotfree;

static MatrixXd velocity_mat;
static MatrixXd velocity_mat_qd0;
static MatrixXd velocity_mat_qdf;
static MatrixXd accel_mat;
static MatrixXd accel_mat_qd0;
static MatrixXd accel_mat_qdf;

static VectorXd* t_inbetween;
static snopt::integer num_inbetween_tSamples;
static MatrixXd* dqInbetweendqknot;
static MatrixXd* dqInbetweendqd0;
static MatrixXd* dqInbetweendqdf;
static snopt::integer* qknot_qsamples_idx;

/* Remeber to delete this*/
static snopt::integer nF_tmp;
static snopt::integer nG_tmp;
static snopt::integer nx_tmp;

static void gevalNumerical(void (*func_ptr)(const VectorXd &, VectorXd &),const VectorXd &x, VectorXd &c, MatrixXd &dc,int order = 2)
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


static void IK_constraint_fun(double* x,double* c, double* G)
{
  double* qsc_weights=NULL;
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
  for(int i = 0;i<num_st_lpc;i++)
  {
    nc_accum += st_lpc_nc[i];
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

static void IK_cost_fun(double* x, double &J, double* dJ)
{
  VectorXd q(nq);
  memcpy(q.data(),x,sizeof(double)*nq);
  VectorXd q_err = q-q_nom_i;
  J = q_err.transpose()*Q*q_err;
  VectorXd dJ_vec = 2*q_err.transpose()*Q;
  memcpy(dJ, dJ_vec.data(),sizeof(double)*nq);
}

static int snoptIKfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
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

static void IKtraj_cost_fun(MatrixXd q,const VectorXd &qdot0,const VectorXd &qdotf,double &J,double* dJ)
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

static int snoptIKtrajfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
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
    VectorXd mtkc_c(mt_kc_nc[i]);
    MatrixXd mtkc_dc(mt_kc_nc[i],nq*(num_qfree+num_inbetween_tSamples));
    mt_kc_array[i]->eval(t_samples+qstart_idx,num_qfree+num_inbetween_tSamples,q_samples.block(0,qstart_idx,nq,num_qfree+num_inbetween_tSamples),mtkc_c,mtkc_dc);
    memcpy(F+nf_cum,mtkc_c.data(),sizeof(double)*mt_kc_nc[i]);
    MatrixXd mtkc_dc_dx = MatrixXd::Zero(mt_kc_nc[i],nq*(num_qfree+num_qdotfree));
    for(int j = qstart_idx;j<nT;j++)
    {
      mtkc_dc_dx.block(0,(j-qstart_idx)*nq,mt_kc_nc[i],nq) = mtkc_dc.block(0,(qknot_qsamples_idx[j]-qstart_idx)*nq,mt_kc_nc[i],nq);
    }
    for(int j = 0;j<nT-1;j++)
    {
      MatrixXd dc_ij = mtkc_dc.block(0,nq*(qknot_qsamples_idx[j]+1-qstart_idx),mt_kc_nc[i],nq*t_inbetween[j].size());
      mtkc_dc_dx.block(0,0,mt_kc_nc[i],nq*num_qfree) += dc_ij*dqInbetweendqknot[j].block(0,qstart_idx*nq,nq*t_inbetween[j].size(),nq*num_qfree);
      if(fixInitialState)
      {
        mtkc_dc_dx.block(0,nq*num_qfree,mt_kc_nc[i],nq) += dc_ij*dqInbetweendqdf[j];
      }
      else
      {
        mtkc_dc_dx.block(0,nq*num_qfree,mt_kc_nc[i],nq) += dc_ij*dqInbetweendqd0[j];
        mtkc_dc_dx.block(0,nq*num_qfree+nq,mt_kc_nc[i],nq) += dc_ij*dqInbetweendqdf[j];
      }
    }
    memcpy(G+nG_cum,mtkc_dc_dx.data(),sizeof(double)*mtkc_dc_dx.size());
    nf_cum += mt_kc_nc[i];
    nG_cum += mt_kc_nc[i]*nq*(num_qfree+num_qdotfree);
  }
  for(int i = 0;i<num_mt_lpc;i++)
  {
    nf_cum += mt_lpc_nc[i];
  }
  return 0;
}


static void snoptIKtraj_userfun(const VectorXd &x_vec, VectorXd &c_vec, VectorXd &G_vec)
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
  char *cu = NULL;
  snopt::integer lencu = 0;
  snopt::integer *iu = NULL;
  snopt::integer leniu = 0;
  snopt::doublereal *ru = NULL;
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

static void snoptIKtraj_fevalfun(const VectorXd &x, VectorXd &c)
{
  VectorXd G;
  snoptIKtraj_userfun(x,c,G);
}

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE>
void inverseKinBackend(RigidBodyManipulator* model, const int mode, const int nT, const double* t, const MatrixBase<DerivedB> &q_seed, const MatrixBase<DerivedB> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<DerivedC> &q_sol, MatrixBase<DerivedD> &qdot_sol, MatrixBase<DerivedE> &qddot_sol, int* INFO, const IKoptions &ikoptions)
{
  nq = model->num_dof;
  if(nT == 0)
  {
    nT = 1;
    t = NULL;
  }
  if(q_seed.rows() != nq || q_seed.cols() != nT || q_nom.rows() != nq || q_nom.cols() != nT)
  {
    cerr<<"Drake:inverseKinBackend: q_seed and q_nom must be of size nq x nT"<<endl;
  }
  num_st_kc = 0;
  num_mt_kc = 0;
  num_st_lpc = 0;
  num_mt_lpc = 0;
  int num_qsc = 0;
  st_kc_array = new SingleTimeKinematicConstraint*[num_constraints];
  mt_kc_array = new MultipleTimeKinematicConstraint*[num_constraints];
  SingleTimeLinearPostureConstraint** st_lpc_array = new SingleTimeLinearPostureConstraint*[num_constraints];
  MultipleTimeLinearPostureConstraint** mt_lpc_array = new MultipleTimeLinearPostureConstraint*[num_constraints];
  qsc_ptr = NULL;
  MatrixXd joint_limit_min(nq,nT);
  MatrixXd joint_limit_max(nq,nT);
  for(int i = 0;i<nT;i++)
  {
    memcpy(joint_limit_min.data()+i*nq,model->joint_limit_min,sizeof(double)*nq);
    memcpy(joint_limit_max.data()+i*nq,model->joint_limit_max,sizeof(double)*nq);
  }
  for(int i = 0;i<num_constraints;i++)
  {
    RigidBodyConstraint* constraint =  constraint_array[i];
    int constraint_category = constraint->getCategory();
    if(constraint_category == RigidBodyConstraint::SingleTimeKinematicConstraintCategory)
    {
      st_kc_array[num_st_kc] = static_cast<SingleTimeKinematicConstraint*>(constraint);
      num_st_kc++;
    }
    else if(constraint_category == RigidBodyConstraint::MultipleTimeKinematicConstraintCategory)
    {
      mt_kc_array[num_mt_kc] = static_cast<MultipleTimeKinematicConstraint*>(constraint);
      num_mt_kc++;
    }
    else if(constraint_category == RigidBodyConstraint::QuasiStaticConstraintCategory)
    {
      num_qsc++;
      if(num_qsc>1)
      {
        cerr<<"Drake:inverseKinBackend:current implementation supports at most one QuasiStaticConstraint"<<endl;
      }
      qsc_ptr = static_cast<QuasiStaticConstraint*>(constraint);
    }
    else if(constraint_category == RigidBodyConstraint::PostureConstraintCategory)
    {
      double* joint_min = new double[nq];
      double* joint_max = new double[nq];
      PostureConstraint* pc = static_cast<PostureConstraint*>(constraint);
      for(int j = 0;j<nT;j++)
      {
        pc->bounds(&t[j],joint_min,joint_max);
        for(int k = 0;k<nq;k++)
        {
          joint_limit_min(k,j) = (joint_limit_min(k,j)>joint_min[k]? joint_limit_min(k,j):joint_min[k]);
          joint_limit_max(k,j) = (joint_limit_max(k,j)<joint_max[k]? joint_limit_max(k,j):joint_max[k]);
          if(joint_limit_min(k,j)>joint_limit_max(k,j))
          {
            cerr<<"Drake:inverseKinBackend:BadInputs Some posture constraint has lower bound larger than the upper bound of other posture constraint for joint "<<k<< " at "<<j<<"'th time "<<endl;
          }
        }
      }
      delete[] joint_min;
      delete[] joint_max;
    }
    else if(constraint_category == RigidBodyConstraint::MultipleTimeLinearPostureConstraintCategory)
    {
      mt_lpc_array[num_mt_lpc] = static_cast<MultipleTimeLinearPostureConstraint*>(constraint);
      num_mt_lpc++;
    }
    else if(constraint_category == RigidBodyConstraint::SingleTimeLinearPostureConstraintCategory)
    {
      st_lpc_array[num_st_lpc] = static_cast<SingleTimeLinearPostureConstraint*>(constraint);
      num_st_lpc++;
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
  ikoptions.getQ(Q);
  snopt::integer SNOPT_MajorIterationsLimit = static_cast<snopt::integer>(ikoptions.getMajorIterationsLimit());
}

template void inverseKinBackend(RigidBodyManipulator* model, const int mode, const int nT, const double* t, const MatrixBase<Map<MatrixXd>> &q_seed, const MatrixBase<Map<MatrixXd>> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<Map<MatrixXd>> &q_sol, MatrixBase<Map<MatrixXd>> &qdot_sol, MatrixBase<Map<MatrixXd>> &qddot_sol, int* INFO, const IKoptions &ikoptions);
template void inverseKinBackend(RigidBodyManipulator* model, const int mode, const int nT, const double* t, const MatrixBase<MatrixXd> &q_seed, const MatrixBase<MatrixXd> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<MatrixXd> &q_sol, MatrixBase<MatrixXd> &qdot_sol, MatrixBase<MatrixXd> &qddot_sol, int* INFO, const IKoptions &ikoptions);
template void inverseKinBackend(RigidBodyManipulator* model, const int mode, const int nT, const double* t, const MatrixBase<Map<VectorXd>> &q_seed, const MatrixBase<Map<VectorXd>> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<Map<VectorXd>> &q_sol, MatrixBase<Map<VectorXd>> &qdot_sol, MatrixBase<Map<VectorXd>> &qddot_sol, int* INFO, const IKoptions &ikoptions);
template void inverseKinBackend(RigidBodyManipulator* model, const int mode, const int nT, const double* t, const MatrixBase<VectorXd> &q_seed, const MatrixBase<VectorXd> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<VectorXd> &q_sol, MatrixBase<VectorXd> &qdot_sol, MatrixBase<VectorXd> &qddot_sol, int* INFO, const IKoptions &ikoptions);
