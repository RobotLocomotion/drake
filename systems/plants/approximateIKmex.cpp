
/*
 * A c++ version of @RigidBodyManipulator/approximateIK.m
 */

#include <math.h>
#include <set>
#include <mex.h>
#include <Eigen/Dense>
#include <gurobi_c++.h>
#include <drakeUtil.h>
#include "RigidBodyManipulator.h"
#include "constraint/Constraint.h"

#define CHECK_GUROBI_ERRORS

#ifndef CHECK_GUROBI_ERRORS
#define CGE( call, env ) {call ;}
#else


void PGE(GRBenv *env) {
 mexPrintf("Gurobi error %s\n", GRBgeterrormsg(env)); 
}

#define CGE( call, env) {int gerror; gerror = call; if (gerror) PGE (env);}

#endif

#define _USE_MATH_DEFINES

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs<5)
  {
    mexErrMsgIdAndTxt("Drake:approximateIKmex:NotEnoughInputs","Usage approximateIKmex(model_ptr,q_seed,q_nom,Q,constraint1,constraint2,...)");
  }
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  double* t = NULL;
  int nq = model->num_dof;
  Map<VectorXd> q_seed(mxGetPr(prhs[1]),nq);
  Map<VectorXd> q_nom(mxGetPr(prhs[2]),nq);
  Map<MatrixXd> Q(mxGetPr(prhs[3]),nq,nq);
  int num_constraint = nrhs-4;
  int num_kc = 0;
  KinematicConstraint** kc_array = new KinematicConstraint*[num_constraint];
  double* joint_limit_min = new double[nq];
  double* joint_limit_max = new double[nq];
  memcpy(joint_limit_min,model->joint_limit_min,sizeof(double)*nq);
  memcpy(joint_limit_max,model->joint_limit_max,sizeof(double)*nq);
  for(int i = 0;i<num_constraint;i++)
  {
    Constraint* constraint = (Constraint*) getDrakeMexPointer(prhs[4+i]);
    DrakeConstraintType constraint_type = constraint->getType();
    if(constraint_type == DrakeConstraintType::KinematicConstraintType)
    {
      kc_array[num_kc] = (KinematicConstraint*) constraint;
      num_kc++;
    }
    else if(constraint_type == DrakeConstraintType::PostureConstraintType)
    {
      double* joint_min = new double[nq];
      double* joint_max = new double[nq];
      PostureConstraint* pc = (PostureConstraint*) constraint;
      pc->bounds(t,joint_min,joint_max);
      for(int j = 0;j<nq;j++)
      {
        joint_limit_min[j] = (joint_limit_min[j]<joint_min[j]? joint_min[j]:joint_limit_min[j]);
        joint_limit_max[j] = (joint_limit_max[j]>joint_max[j]? joint_max[j]:joint_limit_max[j]);
        if(joint_limit_min[j]>joint_limit_max[j])
        {
          mexErrMsgIdAndTxt("Drake:approximateIKmex:BadInputs","posture constraint has lower bound larger than upper bound");
        }
      }
      delete[] joint_min; delete[] joint_max;
    }
  }
  int error;
  GRBenv *grb_env = NULL;
  GRBmodel *grb_model = NULL;  
  
  VectorXd qtmp = -2*Q*q_nom;
  model->qtmp = qtmp;

  { // set up gurobi
		// create gurobi environment
		error = GRBloadenv(&grb_env,NULL);

		// set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
		error = GRBsetintparam(grb_env,"outputflag",0);
		/*error = GRBsetintparam(grb_env,"method",2);
		error = GRBsetintparam(grb_env,"presolve",0);
 		error = GRBsetintparam(grb_env,"bariterlimit",20);
		error = GRBsetintparam(grb_env,"barhomogenous",0);
 		error = GRBsetdblparam(grb_env,"barconvtol",1e-4);*/
		error = GRBnewmodel(grb_env,&grb_model,"ApproximateIK",nq,qtmp.data(),joint_limit_min,joint_limit_max,NULL,NULL);
  }
  // set up cost function
    //cost: (q-q_nom)'*Q*(q-q_nom)
    error = GRBsetdblattrarray(grb_model,"Obj",0,nq,model->qtmp.data());
     
    for(int i = 0;i<nq;i++)
    {
      for(int j = 0;j<nq;j++)
      {
        if(abs(Q(i,j))>1e-5)
        {
          error = GRBaddqpterms(grb_model,1,&i,&j,&Q(i,j));
        }
      }
    }
  

  int* allIndsData = new int[nq];
  for(int j = 0;j<nq;j++)
  {
    allIndsData[j] = j;
  }  
  model->doKinematics(q_seed.data());
  int kc_idx,c_idx;
  for(kc_idx = 0;kc_idx<num_kc;kc_idx++)
  {
    int nc = kc_array[kc_idx]->getNumConstraint(t);
    VectorXd lb(nc);
    VectorXd ub(nc);
    VectorXd c(nc);
    MatrixXd dc(nc,nq);
    kc_array[kc_idx]->bounds(t,lb,ub);
    kc_array[kc_idx]->eval(t,c,dc);
    for(c_idx = 0; c_idx < nc; c_idx++)
    {
      VectorXd rowVec = dc.row(c_idx);
      double *Jrow = rowVec.data();
      double c_seed= c(c_idx)-dc.row(c_idx)*q_seed;
      double rhs_row;
      if(mxIsInf(-lb(c_idx)))
      {
        rhs_row = ub(c_idx)-c_seed;
        CGE(GRBaddconstr(grb_model,nq,allIndsData,Jrow,GRB_LESS_EQUAL,rhs_row,NULL),grb_env);
      }
      else if(mxIsInf(ub(c_idx)))
      {
        if(mxIsInf(lb(c_idx)))
        {
          mexErrMsgIdAndTxt("Drake:approximateIKmex:BadInputs","lb and ub cannot be both infinity, check the getConstraintBnds output of the KinematicConstraint\n");
        }
        rhs_row = lb(c_idx)-c_seed;
        CGE(GRBaddconstr(grb_model,nq,allIndsData,Jrow,GRB_GREATER_EQUAL,rhs_row,NULL),grb_env);
      }
      else if(ub(c_idx)-lb(c_idx)< 1e-10)
      {
        rhs_row = lb(c_idx)-c_seed;
        CGE(GRBaddconstr(grb_model, nq, allIndsData, Jrow, GRB_EQUAL, rhs_row, NULL), grb_env);
      }
      else
      {
        double rhs_row1 = ub(c_idx)-c_seed;
        CGE(GRBaddconstr(grb_model,nq,allIndsData,Jrow,GRB_LESS_EQUAL,rhs_row1,NULL),grb_env);
        double rhs_row2 = lb(c_idx)-c_seed;
        CGE(GRBaddconstr(grb_model,nq,allIndsData,Jrow,GRB_GREATER_EQUAL,rhs_row2,NULL),grb_env);
      }
    } 
  }
  CGE(GRBupdatemodel(grb_model), grb_env);
  
  CGE(GRBoptimize(grb_model), grb_env);

  plhs[0] = mxCreateDoubleMatrix(nq,1,mxREAL);
  error = GRBgetdblattrarray(grb_model, GRB_DBL_ATTR_X, 0, nq,mxGetPr(plhs[0]));

  if (nlhs>1) {
  	int status;
  	error = GRBgetintattr(grb_model, GRB_INT_ATTR_STATUS, &status);
        if(status == 2)
        {
          status = 0;
        }
        else
        {
          status = 1;
        }
  	plhs[1] = mxCreateDoubleScalar(status);
  }
  //debug only
  /*GRBwrite(grb_model,"gurobi_approximateIK.lp");
  int num_gurobi_cnst;
  GRBgetintattr(grb_model,GRB_INT_ATTR_NUMCONSTRS,&num_gurobi_cnst);
  MatrixXd J(num_gurobi_cnst,nq);
  VectorXd rhs(num_gurobi_cnst);
  for(int i = 0;i<num_gurobi_cnst;i++)
  {
    for(int j = 0;j<nq;j++)
    {
      GRBgetcoeff(grb_model,i,j,&J(i,j));
    }
    GRBgetdblattrarray(grb_model,GRB_DBL_ATTR_RHS,0,num_gurobi_cnst,rhs.data());
  }
  plhs[2] = mxCreateDoubleMatrix(num_gurobi_cnst,nq,mxREAL);
  memcpy(mxGetPr(plhs[2]),J.data(),sizeof(double)*num_gurobi_cnst*nq);
  plhs[3] = mxCreateDoubleMatrix(num_gurobi_cnst,1,mxREAL);
  memcpy(mxGetPr(plhs[3]),rhs.data(),sizeof(double)*num_gurobi_cnst);
  plhs[4] = mxCreateDoubleMatrix(nq,1,mxREAL);
  GRBgetdblattrarray(grb_model,GRB_DBL_ATTR_LB,0,nq,mxGetPr(plhs[4]));
  plhs[5] = mxCreateDoubleMatrix(nq,1,mxREAL);
  GRBgetdblattrarray(grb_model,GRB_DBL_ATTR_UB,0,nq,mxGetPr(plhs[5]));
  char* sense = new char[num_gurobi_cnst];
  GRBgetcharattrarray(grb_model,GRB_CHAR_ATTR_SENSE,0,num_gurobi_cnst,sense);
  plhs[6] = mxCreateString(sense);*/

  GRBfreemodel(grb_model);
  GRBfreeenv(grb_env);
  delete[] joint_limit_min;
  delete[] joint_limit_max;
  delete[] allIndsData;
  delete[] kc_array;
  return;
}
