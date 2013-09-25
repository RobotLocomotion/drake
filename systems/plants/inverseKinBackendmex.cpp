#include <mex.h>
#include <math.h>
#include <float.h>

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
VectorXd q0;
VectorXd qdot0;
bool quasiStaticFlag;
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
double* ti = NULL;
int num_st_kc;
int num_mt_kc;
int* mtkc_nc;
int nT;
int num_qsc_pts;
// The following variables are used in inverseKinSequence only
int* q_idx;
int* qdotf_idx;

MatrixXd velocity_mat;
MatrixXd accel_mat;
MatrixXd accel_mat_qd0;
MatrixXd accel_mat_qdf;


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
  if(quasiStaticFlag)
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
  if(quasiStaticFlag)
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

void IKtraj_cost_fun(double* x,double &J,double* dJ)
{
  MatrixXd dJ_vec = MatrixXd::Zero(1,nq*nT);
  VectorXd qdotf(nq);
  for(int i = 0;i<nq;i++)
  {
    qdotf(i) = x[qdotf_idx[i]];
  }
  MatrixXd q(nq*nT,1);
  MatrixXd qdot(nq*(nT-1),1);
  MatrixXd qddot(nq*nT,1);
  memcpy(q.data(),q0.data(),sizeof(double)*nq);
  for(int i = 0;i<nq*(nT-1);i++)
  {
    q(nq+i) = x[q_idx[i]];
  }
  qdot.block(0,0,nq*(nT-2),1) = velocity_mat*q;
  qdot.block(nq*(nT-2),0,nq,1) = qdotf;
  qddot = accel_mat*q+accel_mat_qd0*qdot0+accel_mat_qdf*qdotf;
  q.resize(nq,nT);
  qdot.resize(nq,nT-1);
  qddot.resize(nq,nT);
  MatrixXd q_diff = q.block(0,1,nq,nT-1)-q_nom.block(0,1,nq,nT-1);
  MatrixXd tmp1 = 0.5*Qa*qddot;
  MatrixXd tmp2 = tmp1.cwiseProduct(qddot);
  J = tmp2.sum();
  MatrixXd tmp3 = 0.5*Qv*qdot;
  MatrixXd tmp4 = tmp3.cwiseProduct(qdot);
  J += tmp4.sum();
  MatrixXd tmp5 = 0.5*Q*q_diff;
  MatrixXd tmp6 = tmp5.cwiseProduct(q_diff);
  J += tmp6.sum();
  MatrixXd dJdqd = 2*tmp3.block(0,0,nq,nT-2);
  dJdqd.resize(1,nq*(nT-2));
  dJ_vec.block(0,0,1,nq*(nT-1)) = dJdqd*velocity_mat.block(0,nq,(nT-2)*nq,nq*(nT-1));
  MatrixXd dJdqdiff = 2*tmp5;
  dJdqdiff.resize(1,nq*(nT-1));
  dJ_vec.block(0,0,1,nq*(nT-1)) += dJdqdiff;
  MatrixXd dJdqdd = 2.0*tmp1;
  dJdqdd.resize(1,nq*nT);
  dJ_vec.block(0,0,1,nq*(nT-1)) += dJdqdd*accel_mat.block(0,nq,nq*nT,nq*(nT-1));
  MatrixXd dJdqdotf;
  dJdqdotf = dJdqdd*accel_mat_qdf+Qv*qdotf;
  dJdqdotf.resize(1,nq);
  dJ_vec.block(0,nq*(nT-1),1,nq) = dJdqdotf;
  memcpy(dJ,dJ_vec.data(),sizeof(double)*nq*nT);
}

int snoptIKtrajfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *neF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *neG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru)
{
  IKtraj_cost_fun(x,F[0],G);
  int nf_cum = 1;
  int nG_cum = nq*nT;
  MatrixXd q(nq,nT-1);
  for(int i = 1;i<nT;i++)
  {
    double* qi;
    if(quasiStaticFlag)
    {
      qi = x+(i-1)*(nq+num_qsc_pts);
    }
    else
    {
      qi = x+(i-1)*nq;
    }
    memcpy(q.data()+nq*(i-1),qi,sizeof(double)*nq);
    model->doKinematics(qi);
    ti = &t[i];
    IK_constraint_fun(qi,F+nf_cum,G+nG_cum);
    nf_cum += nc_array[i];
    nG_cum += nG_array[i];
  }
  for(int i = 0;i<num_mt_kc;i++)
  {
    VectorXd mtkc_c(mtkc_nc[i]);
    MatrixXd mtkc_dc(mtkc_nc[i],nq*(nT-1));
    mt_kc_array[i]->eval(t+1,nT-1,q,mtkc_c,mtkc_dc);
    memcpy(F+nf_cum,mtkc_c.data(),sizeof(double)*mtkc_nc[i]);
    memcpy(G+nG_cum,mtkc_dc.data(),sizeof(double)*mtkc_dc.size());
    nf_cum += mtkc_nc[i];
    nG_cum += mtkc_nc[i]*nq*(nT-1);
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
  IKtraj_cost_fun(x,F[0],G);
  int nf_cum = 1;
  int nG_cum = nq*nT;
  MatrixXd q(nq,nT-1);
  for(int i = 1;i<nT;i++)
  {
    double* qi;
    if(quasiStaticFlag)
    {
      qi = x+(i-1)*(nq+num_qsc_pts);
    }
    else
    {
      qi = x+(i-1)*nq;
    }
    memcpy(q.data()+nq*(i-1),qi,sizeof(double)*nq);
    model->doKinematics(qi);
    ti = &t[i];
    IK_constraint_fun(qi,F+nf_cum,G+nG_cum);
    nf_cum += nc_array[i];
    nG_cum += nG_array[i];
  }
  for(int i = 0;i<num_mt_kc;i++)
  {
    VectorXd mtkc_c(mtkc_nc[i]);
    MatrixXd mtkc_dc(mtkc_nc[i],nq*(nT-1));
    mt_kc_array[i]->eval(t+1,nT-1,q,mtkc_c,mtkc_dc);
    memcpy(F+nf_cum,mtkc_c.data(),sizeof(double)*mtkc_nc[i]);
    memcpy(G+nG_cum,mtkc_dc.data(),sizeof(double)*mtkc_dc.size());
    nf_cum += mtkc_nc[i];
    nG_cum += mtkc_nc[i]*nq*(nT-1);
  }
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
            mexErrMsgIdAndTxt("Drake:inverseKinBackendmex:BadInputs","Posture constraint has lower bound larger than upper bound");
          }
        }
      }
      delete[] joint_min;
      delete[] joint_max;
    }
  }
  if(qsc_ptr == NULL)
  {
    quasiStaticFlag = false;
    num_qsc_pts = 0;
  }
  else
  {
    quasiStaticFlag = qsc_ptr->isActive(); 
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
  VectorXd* iCfun_array = new VectorXd[nT];
  VectorXd* jCvar_array = new VectorXd[nT];
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
  VectorXd* iAfun_array = new VectorXd[nT];
  VectorXd* jAvar_array = new VectorXd[nT];
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
        VectorXd iCfun_append(nc);
        VectorXd jCvar_append(nc);
        for(int k = 0;k<nc;k++)
        {
          iCfun_append(k) = nc_array[i]+k+1;
        }
        for(int k = 0;k<nq;k++)
        {
          iCfun_array[i].segment(nG_array[i]+k*nc,nc) = iCfun_append;
          jCvar_append = VectorXd::Constant(nc,k+1);
          jCvar_array[i].segment(nG_array[i]+k*nc,nc) = jCvar_append;
        }
        nc_array[i] = nc_array[i]+nc;
        nG_array[i] = nG_array[i]+nq*nc;
        if(debug_mode)
        {
          vector<string> constraint_name;
          st_kc_array[j]->name(&t[i],constraint_name);
          for(int l = 0;l<nc;l++)
          {
            Cname_array[i].push_back(constraint_name[l]);
          }
        }
      }
    }
    if(quasiStaticFlag)
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
      iAfun_array[i].tail(num_qsc_pts) = VectorXd::Constant(num_qsc_pts,nc_array[i]+num_qsc_cnst);
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
      if(!quasiStaticFlag)
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
      double* xlow = new double[nx];
      double* xupp = new double[nx];
      memcpy(xlow,joint_limit_min.col(i).data(),sizeof(double)*nq);
      memcpy(xupp,joint_limit_max.col(i).data(),sizeof(double)*nq);
      if(quasiStaticFlag)
      {
        for(int k = 0;k<num_qsc_pts;k++)
        {
          xlow[nq+k] = 0.0;
          xupp[nq+k] = 1.0;
        }
      }
      double* Flow = new double[nF];
      double* Fupp = new double[nF];
      Flow[0] = -mxGetInf();
      Fupp[0] = mxGetInf();
      memcpy(&Flow[1],Cmin_array[i].data(),sizeof(double)*nc_array[i]);
      memcpy(&Fupp[1],Cmax_array[i].data(),sizeof(double)*nc_array[i]);
      ti = &t[i];
      q_nom_i = q_nom.col(i);
      double* x = new double[nx];
      if(!sequentialSeedFlag)
      {
        memcpy(x,q_seed.col(i).data(),sizeof(double)*nq);
      }
      else
      {
        if(i == 0)
        {
          memcpy(x,q_seed.col(i).data(),sizeof(double)*nq);
        }
        else 
        {
          if(INFO[i-1]>10)
          {
            memcpy(x,q_seed.col(i).data(),sizeof(double)*nq);
          }
          else
          {
            memcpy(x,q_sol.col(i-1).data(),sizeof(double)*nq);
          }
        }
      }
      if(quasiStaticFlag)
      {
        for(int j = 0;j<num_qsc_pts;j++)
        {
          x[nq+j] = 1.0/num_qsc_pts;
        }
      }

      snopt::integer minrw,miniw,mincw;
      snopt::integer lenrw = 500000, leniw = 50000, lencw = 500;
      snopt::doublereal rw[lenrw];
      snopt::integer iw[leniw];
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
    pm = mxGetProperty(prhs[ikoptions_idx],0,"q0");
    q0.resize(nq);
    memcpy(q0.data(),mxGetPr(pm),sizeof(double)*nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"qd0");
    qdot0.resize(nq);
    memcpy(qdot0.data(),mxGetPr(pm),sizeof(double)*nq);
    VectorXd qdf_lb(nq);
    VectorXd qdf_ub(nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"qdf_lb");
    memcpy(qdf_lb.data(),mxGetPr(pm),sizeof(double)*nq);
    pm = mxGetProperty(prhs[ikoptions_idx],0,"qdf_ub");
    memcpy(qdf_ub.data(),mxGetPr(pm),sizeof(double)*nq);
    VectorXd qdf_seed = (qdf_lb+qdf_ub)/2;
    int nSample = nT-1;
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
      double val_tmp3 = 4.0/dt[nT-2];
      double val_tmp4 = 5.0/dt[nT-2];
      accel_mat1((nT-1)*nq+k,(nT-2)*nq+k) = val_tmp1;
      accel_mat1((nT-1)*nq+k,(nT-1)*nq+k) = val_tmp2;
      accel_mat2((nT-1)*nq+k,(nT-2)*nq+k) = val_tmp3;
      accel_mat2((nT-1)*nq+k,(nT-1)*nq+k) = val_tmp4;
    }
    accel_mat.resize(nq*nT,nq*nT);
    accel_mat = accel_mat1+accel_mat2.block(0,nq,nq*nT,nq*(nT-2))*velocity_mat;
    accel_mat_qd0.resize(nq*nT,nq);
    accel_mat_qd0 = accel_mat2.block(0,0,nq*nT,nq);
    accel_mat_qdf.resize(nq*nT,nq);
    accel_mat_qdf = accel_mat2.block(0,(nT-1)*nq,nT*nq,nq);
   
    q_idx = new int[nq*(nT-1)];
    qdotf_idx = new int[nq];
    if(quasiStaticFlag)
    {
      nx= nq*nT+num_qsc_pts*(nT-1);
      for(int j = 0;j<nT-1;j++)
      {
        for(int k = 0;k<nq;k++)
        {
          q_idx[j*nq+k] = j*(nq+num_qsc_pts)+k;
        }
      }
      for(int k = 0;k<nq;k++)
      {
        qdotf_idx[k] = (nT-1)*(nq+num_qsc_pts)+k;
      } 
    }
    else
    {
      nx = nq*nT;
      for(int j = 0;j<(nT-1)*nq;j++)
      {
        q_idx[j] = j;
      }
      for(int j = 0;j<nq;j++)
      {
        qdotf_idx[j] = (nT-1)*nq+j;
      }
    }

    double* xlow = new double[nx];
    double* xupp = new double[nx];
    for(int j = 1;j<nT;j++)
    {
      if(quasiStaticFlag)
      {
        memcpy(xlow+(j-1)*(nq+num_qsc_pts),joint_limit_min.data()+j*nq,sizeof(double)*nq);
        memcpy(xupp+(j-1)*(nq+num_qsc_pts),joint_limit_max.data()+j*nq,sizeof(double)*nq);
        for(int k = 0;k<num_qsc_pts;k++)
        {
          xlow[(j-1)*(nq+num_qsc_pts)+nq+k] = 0.0;
          xupp[(j-1)*(nq+num_qsc_pts)+nq+k] = 1.0;
        }
      }
      else
      {
        memcpy(xlow+(j-1)*nq,joint_limit_min.col(j).data(),sizeof(double)*nq);
        memcpy(xupp+(j-1)*nq,joint_limit_max.col(j).data(),sizeof(double)*nq);
      }
    }
    if(quasiStaticFlag)
    {
      memcpy(xlow+(nq+num_qsc_pts)*(nT-1),qdf_lb.data(),sizeof(double)*nq);
      memcpy(xupp+(nq+num_qsc_pts)*(nT-1),qdf_ub.data(),sizeof(double)*nq);
    }
    else
    {
      memcpy(xlow+nq*(nT-1), qdf_lb.data(),sizeof(double)*nq);
      memcpy(xupp+nq*(nT-1), qdf_ub.data(),sizeof(double)*nq);
    }
    
    nF = 1;
    nG = nq*nT;
    snopt::integer lenA = 0;
    for(int j = 1;j<nT;j++)
    {
      nF += nc_array[j]; 
      nG += nG_array[j];
      lenA += nA_array[j];
    } 
    mtkc_nc = new int[num_mt_kc];
    for(int j = 0;j<num_mt_kc;j++)
    {
      mtkc_nc[j] = mt_kc_array[j]->getNumConstraint(t+1,nT-1);
      nF += mtkc_nc[j];
      nG += mtkc_nc[j]*nq*(nT-1);
    }
    double* Flow = new double[nF];
    double* Fupp = new double[nF];
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
    Fname[0] = string("Objective");
    for(int j = 0;j<nq*(nT-1);j++)
    {
      iGfun[j] = 1;
      jGvar[j] = q_idx[j]+1;//C interface uses 1 index
    }
    for(int j = 0;j<nq;j++)
    {
      iGfun[j+(nT-1)*nq] = 1;
      jGvar[j+(nT-1)*nq] = qdotf_idx[j]+1;//C interface uses 1 index
    }
    snopt::integer nf_cum = 1;
    snopt::integer nG_cum = nq*nT;
    snopt::integer nA_cum = 0;
    int x_start_idx = 0;
    for(int j = 1;j<nT;j++)
    {
      memcpy(Flow+nf_cum,Cmin_array[j].data(),sizeof(double)*nc_array[j]);
      memcpy(Fupp+nf_cum,Cmax_array[j].data(),sizeof(double)*nc_array[j]);
      for(int k = 0;k<Cname_array[j].size();k++)
      {
        Fname[nf_cum+k] = Cname_array[j][k];
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
      if(quasiStaticFlag)
      {
        x_start_idx += nq+num_qsc_pts;
      }
      else
      {
        x_start_idx += nq;
      }
    }

    for(int j = 0;j<num_mt_kc;j++)
    {
      VectorXd mtkc_lb(mtkc_nc[j]);
      VectorXd mtkc_ub(mtkc_nc[j]);
      mt_kc_array[j]->bounds(t+1,nT-1,mtkc_lb,mtkc_ub);
      memcpy(Flow+nf_cum,mtkc_lb.data(),sizeof(double)*mtkc_nc[j]);
      memcpy(Fupp+nf_cum,mtkc_ub.data(),sizeof(double)*mtkc_nc[j]);
      vector<string> mtkc_name;
      mt_kc_array[j]->name(t+1,nT-1,mtkc_name);
      for(int k = 0;k<mtkc_nc[j];k++)
      {
        Fname[nf_cum+k] = mtkc_name[k];
      }
      for(int l = 0;l<nq*(nT-1);l++)
      {
        for(int k = 0;k<mtkc_nc[j];k++)
        {
          iGfun[nG_cum+l*mtkc_nc[j]+k] = nf_cum+k+1;
          jGvar[nG_cum+l*mtkc_nc[j]+k] = q_idx[l]+1;
        }
      }
      nf_cum += mtkc_nc[j];
      nG_cum += mtkc_nc[j]*nq*(nT-1);
    }
    double* x = new double[nx];
    if(quasiStaticFlag)
    {
      for(int j = 1;j<nT;j++)
      {
        memcpy(x+(j-1)*(nq+num_qsc_pts),q_seed.data()+j*nq,sizeof(double)*nq);
        for(int k = 0;k<num_qsc_pts;k++)
        {
          x[(j-1)*(nq+num_qsc_pts)+nq+k] = 1.0/num_qsc_pts;
        }
      }
      memcpy(x+(nq+num_qsc_pts)*(nT-1),qdf_seed.data(),sizeof(double)*nq);
    }
    else
    {
      memcpy(x,q_seed.data()+nq,sizeof(double)*nq*(nT-1));
      memcpy(x+nq*(nT-1), qdf_seed.data(),sizeof(double)*nq);
    }
    
    snopt::integer minrw,miniw,mincw;
    snopt::integer lenrw = 100000, leniw = 100000, lencw = 5000;
    snopt::doublereal rw[lenrw];
    snopt::integer iw[leniw];
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
    mexPrintf("got iAfun jAvar A\n");*/ 
   
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
      /* mxArray* df_ptr = mxCreateDoubleMatrix(nF,nx,mxREAL);
      memcpy(mxGetPr(df_ptr),df_userfun.data(),sizeof(double)*nF*nx);
      mxSetCell(plhs[0],11,df_ptr);*/
    }
    MatrixXd q(nq*nT,1);
    VectorXd qdotf(nq);
    q.block(0,0,nq,1) = q0;
    //memcpy(q.data()+nq,x,sizeof(double)*nq*(nT-1));
    for(int j = 0;j<nq*(nT-1);j++)
    {
      q(j+nq) = x[q_idx[j]];
    }
    //memcpy(qdotf.data(),x+nq*(nT-1),sizeof(double)*nq);
    for(int j = 0;j<nq;j++)
    {
      qdotf(j) = x[qdotf_idx[j]];
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
    delete[] xmul; delete[] xstate; delete[] xnames; 
    delete[] F; delete[] Fmul; delete[] Fstate; delete[] Fnames;
    delete[] iGfun;  delete[] jGvar;
    if(lenA>0)
    {
      delete[] iAfun;  delete[] jAvar;  delete[] A;
    }
    delete[] x; delete[] xlow; delete[] xupp; delete[] Flow; delete[] Fupp; delete[] Fname;
    delete[] dt; delete[] dt_ratio;
  } 
  delete[] INFO; delete[] INFO_tmp;
  delete[] iAfun_array; delete[] jAvar_array; delete[] A_array;
  if(mode == 2)
  {
    delete[] q_idx; delete[] qdotf_idx;
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


