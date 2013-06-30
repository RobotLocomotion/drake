
/*
 * A c++ version of @RigidBodyManipulator/approximateIK.m
 */
#include <math.h>
#include <set>
#include <mex.h>
#include <Eigen/Dense>
#include <gurobi_c++.h>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

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

void angleDiff(VectorXd phi1, VectorXd phi2, VectorXd* d) {
  *d = phi2 - phi1;
  
  for (int i = 0; i < phi1.size(); i++) {
    if ((*d)(i) < -M_PI) {
      (*d)(i) = fmod((*d)(i) + M_PI,2*M_PI) + M_PI;
    } else {
      (*d)(i) = fmod((*d)(i) + M_PI, 2*M_PI) - M_PI;
    }
  }
}

void angleDiff(MatrixXd phi1, MatrixXd phi2, MatrixXd* d) {
  *d = phi2 - phi1;
  
  for (int i = 0; i < phi1.rows(); i++) {
    for (int j = 0; j < phi1.cols(); j++) {
      if ((*d)(i,j) < -M_PI) {
        (*d)(i,j) = fmod((*d)(i,j) + M_PI, 2*M_PI) + M_PI;
      } else {
        (*d)(i,j) = fmod((*d)(i,j) + M_PI, 2*M_PI) - M_PI;
      }
    }
  }
}

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs < 3) {
    mexErrMsgIdAndTxt("Drake:approximateIKmex:NotEnoughInputs","Usage approximateIKmex(model_ptr,q0,q_nom,Q,...)");
  }
  
  if (nlhs<1) return;

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int i,j,error,nq = model->num_dof;

  GRBenv *grb_env = NULL;
  GRBmodel *grb_model = NULL;  

  { // set up gurobi
		// create gurobi environment
		error = GRBloadenv(&grb_env,NULL);

		// set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
		error = GRBsetintparam(grb_env,"outputflag",0);
		error = GRBsetintparam(grb_env,"method",1);
		error = GRBsetintparam(grb_env,"presolve",0);
// 		error = GRBsetintparam(grb_env,"bariterlimit",20);
// 		error = GRBsetintparam(grb_env,"barhomogenous",0);
// 		error = GRBsetdblparam(grb_env,"barconvtol",0.0001);
		error = GRBnewmodel(grb_env,&grb_model,"ApproximateIK",nq,NULL,model->joint_limit_min,model->joint_limit_max,NULL,NULL);
  }

  { // set up cost function
  	// cost:  (q-q_nom)'*Q*(q-q_nom)  \equiv q'*Q*q - 2*q_nom'*Q*q  (const term doesn't matter)
  	Map<VectorXd> q_nom(mxGetPr(prhs[2]),nq);
  	Map<MatrixXd> Q(mxGetPr(prhs[3]),nq,nq);
   	model->qtmp = -2*Q*q_nom;

  	error = GRBsetdblattrarray(grb_model,"Obj",0,nq,model->qtmp.data());

  	for (i=0; i<nq; i++)
  		for (j=0; j<nq; j++)
  			if (abs(Q(i,j))>1e-5)
  				error = GRBaddqpterms(grb_model,1,&i,&j,&Q(i,j));
  }
  
  int allIndsData[nq];
  
  for (j = 0; j < nq; j++) {
   allIndsData[j] = j; 
  }
  
  double *q0 = mxGetPr(prhs[1]);
  model->doKinematics(q0);
  
  VectorXd q0vec = Map<VectorXd>(q0,nq);

  double zero_vec[3] = {0.0, 0.0, 0.0};  
  double zero_hvec[4] = {0.0, 0.0, 0.0, 1.0};  
  i=4;

  while (i<nrhs) {
//     Map<MatrixXd>* body_pos = NULL;
    MatrixXd body_pos;
    Map<MatrixXd>* world_pos = NULL;
    int rows;
  	int body_ind = mxGetScalar(prhs[i]) - 1;
    mxArray *min = NULL;
    mxArray *max = NULL;
    int n_pts;
    if (body_ind==-1) {
      int n_pts = mxGetN(prhs[i+1]);
//       body_pos = new Map<MatrixXd>(zero_hvec,4,1);
      body_pos.resize(4,1);
      body_pos << 0,0,0,1;
      world_pos = new Map<MatrixXd>(mxGetPr(prhs[i+1]),3,n_pts);
      
      rows = 3;
      i+=2;
    } else {
      if (mxIsClass(prhs[i+2],"struct"))  {//isstruct(worldpos)
        min = mxGetField(prhs[i+2],0,"min");
        max = mxGetField(prhs[i+2],0,"max");
        
        if (min == NULL || max == NULL) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "if world_pos is a struct, it must have fields .min and .max");
        }
        
        rows = mxGetM(min);
        
        if (rows != 3 && rows != 6) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "world_pos.min must have 3 or 6 rows");
        }
        
        if (mxGetM(max) != rows) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "world_pos.max must have the same number of rows as world_pos.min");
        }
      } else {
        rows = mxGetM(prhs[i+2]);
        int n_pts = mxGetN(prhs[i+2]);
        world_pos = new Map<MatrixXd>(mxGetPr(prhs[i+2]),rows,n_pts);
      }
      
      if (mxIsClass(prhs[i+1],"char") || mxGetM(prhs[i+1]) == 1) {
        mexErrMsgIdAndTxt("Drake:approximateIKMex:NotImplemented", "collision group not implemented in mex (mposa)");
        
//         set<int> body_idx;
//         body_idx.insert(body_ind);
//         int n_contactpts = model->getNumContacts(body_idx);
//         MatrixXd contact_pos = MatrixXd::Zero(3,n_contactpts);
//         model->getContactPositions(contact_pos,body_idx);
//         body_pos = contact_pos.rowwise().mean();
//         body_pos = new Map<MatrixXd>(contact_pos.rowwise().mean(),3,1);
//         std::cout << contact_pos.rowwise().mean() << std::endl;;
      } else {
        if (mxGetM(prhs[i+1]) !=3) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "bodypos must be 3xmi");
        }
        n_pts = mxGetN(prhs[i+1]);
        
        Map<MatrixXd> pts_tmp(mxGetPr(prhs[i+1]), 3, n_pts);                    
        body_pos.resize(4,n_pts);
        body_pos << pts_tmp, MatrixXd::Ones(1,n_pts);
        
//         body_pos = new Map<MatrixXd>(mxGetPr(prhs[i+1]),3,n_pts);       
      }
      i += 3;
    }   
    
    MatrixXd x;
    MatrixXd J;
    if (body_ind == -1) {
      x = VectorXd::Zero(3,1);
      Vector3d x_com;// = Map<Vector3d>(x.data(),3);
      
      model->getCOM(x_com);      
      model->getCOMJac(J);     
      x.resize(3,1);
      x << x_com;      
    } else {         
      
      J.resize(rows*n_pts,nq);
      
      model->forwardKin(body_ind, body_pos, (int) rows==6,x);
      model->forwardJac(body_ind, body_pos, (int) rows==6,J);
      
      if (rows == 6 && min == NULL && max == NULL) {
        VectorXd delta;
        angleDiff(x.block(3,0,3,n_pts), (*world_pos).block(3,0,3,n_pts), &delta);
        (*world_pos).block(3,0,3,n_pts) = x.block(3,0,3,n_pts)+delta;
      }
    }
      
    
        
//   plhs[0] = mxCreateDoubleMatrix(nq,1,mxREAL);
//   GRBfreemodel(grb_model);
//   GRBfreeenv(grb_env);
//   return;
//     
    if (max != NULL) {
      double *val = mxGetPr(max);
      
      // add inequality constraint
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN(val[j])) {
          VectorXd rowVec = J.row(j);
          double *Jrow = rowVec.data();
          double rhs = val[j] - x(j) + J.row(j)*q0vec;
          if (mxIsInf(rhs)) {
            if (rhs < 0) {
              mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "RHS of a constraint evaluates to -inf");
            }
          } else {
            CGE(GRBaddconstr(grb_model, nq, allIndsData, Jrow, GRB_LESS_EQUAL, rhs, NULL), grb_env);
          }
//           mexPrintf("rhs = %f\n", );
//           CGE(GRBaddconstr(grb_model, nq, allIndsData, Jrow, GRB_LESS_EQUAL, val[j] - x(j) + J.row(j)*q0vec, NULL), grb_env);
        }
      }
    }
    
    if (min != NULL) {
      double *val = mxGetPr(min);
      // add inequality constraint
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN(val[j])) {
          VectorXd rowVec = J.row(j);
          double *Jrow = rowVec.data();
          double rhs = -(val[j] - x(j) + J.row(j)*q0vec);
          if (mxIsInf(rhs)) {
            if (rhs < 0) {
              mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "RHS of a constraint evaluates to -inf");
            }
          } else {
            CGE(GRBaddconstr(grb_model, nq, allIndsData, Jrow, GRB_LESS_EQUAL, rhs, NULL), grb_env);
          }
        }
      }
    }
    
   if (min == NULL && max == NULL) {
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN((*world_pos)(j))) {
          VectorXd rowVec = J.row(j);
          double *Jrow = rowVec.data();
//         for (int k = 0; k < nq; k++) {
//          mexPrintf("Constr var=%d coef=%f\n",allIndsData[k],Jrow[k]);
//         }
//         cout << endl << endl;
//         mexPrintf("rhs = %f\n", (*world_pos)(j) - x(j) + J.row(j)*q0vec);
          
          CGE(GRBaddconstr(grb_model, nq, allIndsData, Jrow, GRB_EQUAL, (*world_pos)(j) - x(j) + J.row(j)*q0vec, NULL), grb_env);
        }
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
  	plhs[1] = mxCreateDoubleScalar(status);
  }
  
  GRBfreemodel(grb_model);
  GRBfreeenv(grb_env);
  return;
}
