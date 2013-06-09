/*
 * A c++ version of @RigidBodyManipulator/approximateIK.m
 */
#include <math.h>
#include <set>
#include <mex.h>
#include <Eigen/Dense>
#include "RigidBodyManipulator.h"
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

#define USE_EIQUADPROG_BACKUP 1

#if USE_EIQUADPROG_BACKUP
#include "eiquadprog.hpp"
#endif

#define _USE_MATH_DEFINES

#define MAX_CONSTRS 1000
#define MAX_STATE   1000
#define MAX_ITER    20
using namespace std;

int fastQP(const VectorXd Qdiag, const VectorXd f, const MatrixXd Aeq, const VectorXd beq, const MatrixXd Ain, const VectorXd bin, VectorXd* active, VectorXd* x) {
  int fail = 0;
  int iterCnt = 0;
  
  int M_in = bin.size();
  int M = Aeq.rows();
  int N = Aeq.cols();
  int n_active = (*active).size();
  
  MatrixXd Aact = MatrixXd(n_active, N);
  VectorXd bact = VectorXd(n_active);
  
            
  int i;
  
  for (i=0; i<n_active; i++) {
    Aact.row(i) = Ain.row((*active)(i));
    bact(i) = bin((*active)(i));
  }
 
  // calculate a bunch of stuff that is constant during each iteration
  VectorXd Qdiag_mod = Qdiag + VectorXd::Constant(N,1e-8);
    
  VectorXd QinvDiag = Qdiag_mod.cwiseInverse();
    
  MatrixXd QinvAteq = QinvDiag.asDiagonal()*Aeq.transpose(); 
  VectorXd minusQinvf = -QinvDiag.cwiseProduct(f);
  
  MatrixXd A;
  VectorXd b;
  MatrixXd QinvAt;
  VectorXd lam, lamIneq;
  VectorXd violated(M_in);
  VectorXd violation;
  
//       cout << "nactive" << n_active << endl;
//   return 0;
  
  while(1) {
    A.resize(Aeq.rows() + Aact.rows(),N);
    b.resize(beq.size() + bact.size());
    A << Aeq,Aact;
    b << beq,bact;
    
//     cout << "dbg: " << A << endl;
//     cout << "dbg: " << b << endl;
    
    if (A.rows() > 0) {
      //Solve H * [x;lam] = [-f;b] using Schur complements
      QinvAt.resize(QinvAteq.rows(), QinvAteq.cols() + Aact.rows());
      QinvAt << QinvAteq, QinvDiag.asDiagonal()*Aact.transpose();
      
//     cout << "dbg: " << f.transpose()*QinvAt << endl;
//         cout << "dbg: " << A*QinvAt << endl;
//         cout << "dbg: " << b + f.transpose()*QinvAt << endl;
      
      lam.resize(QinvAt.cols());
      lam = -(A*QinvAt).ldlt().solve(b + (f.transpose()*QinvAt).transpose());
//     cout << "dbgl: " << lam << endl;
      
      *x = minusQinvf - QinvAt*lam;
      lamIneq = lam.tail(lam.size() - M);
    } else {
      *x = minusQinvf;
      lamIneq.resize(0);
    }
    
//     cout << "dbg: " << *x << endl;
//     return 0;
    
    
    if(Ain.rows() == 0) {
      (*active).resize(0);
      break;
    }
//     cout << "dbg: " << A << endl;
    
    
    violation = Ain*(*x) - bin;
    
//     cout << "violation: " << violation << endl;
    
    violated.conservativeResize(M_in);
    int violCount = 0;
    for(i=0; i < M_in; i++) {
      if (violation(i) >= 1e-6) {
        violated(violCount) = i;
        violCount++;
      }
    }
    
//     cout << "lamIneq: " << lamIneq << endl;
    if (violCount == 0 && lamIneq.all()) {
      break;
    }    
    violated.conservativeResize(violCount);
//     cout << "violated: " << violated << endl;
    int prevActiveCount = (*active).size();
//     cout << "prevActive: " << *active << endl;
    (*active).conservativeResize(M_in);
    
    int newActiveCount = 0;
    
    
    
    for (i=0; i < prevActiveCount; i++) {
     if (lamIneq(i) < 0 ) {
      if(violCount > 0) {
        violCount--;
        (*active)(newActiveCount) = violated(violCount);
        newActiveCount++;
      }
     } else {
      newActiveCount++; 
     }
    }
    
    while(violCount > 0) {
      violCount--;
      (*active)(newActiveCount) = violated(violCount);
      newActiveCount++;
    }
    
    (*active).conservativeResize(newActiveCount);
    
//     cout << "newActive: " << *active << endl;
    
    n_active = newActiveCount;
    Aact.resize(n_active,N);
    bact.resize(n_active);
    
    for (i=0; i<n_active; i++) {
      Aact.row(i) = Ain.row((*active)(i));
      bact(i) = bin((*active)(i));
    }
    iterCnt++;

    if (iterCnt > MAX_ITER) {
      //Default to calling this method
      cout << "FastQP max iter reached." << endl;
//       mexErrMsgIdAndTxt("Drake:approximateIKmex:Error", "Max iter reached. Problem is likely infeasible");
      fail = 1;
      return fail;
    }
  }  
  return fail;
}


void angleDiff(VectorXd phi1, VectorXd phi2, VectorXd* d) {
  *d = phi2 - phi1;
  
  for (int i = 0; i < phi1.size(); i++) {
    if ((*d)(i) < -M_PI) {
      (*d)(i) = fmod((*d)(i) + M_PI, 2*M_PI) + M_PI;
    } else {
      (*d)(i) = fmod((*d)(i) + M_PI, 2*M_PI) - M_PI;
    }
  }
}

void angleDiff(MatrixXd phi1, MatrixXd phi2, MatrixXd* d) {
  *d = phi2 - phi1;
  
  for (int i = 0; i < phi1.rows(); i++) {
    for (int j = 0; j < phi1.cols(); j++) {
      if ((*d)(i, j) < -M_PI) {
        (*d)(i, j) = fmod((*d)(i, j) + M_PI, 2*M_PI) + M_PI;
      } else {
        (*d)(i, j) = fmod((*d)(i, j) + M_PI, 2*M_PI) - M_PI;
      }
    }
  }
}

using namespace std;

/**
 * Use Frank's fastQP code (mexed)
 * [q,info] = approximateIKEIQPmex(objgetMexModelPtr, q0, q_nom, Q, varargin)
 * info = 0 on success, 1 on failure
 **/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 4) {
    mexErrMsgIdAndTxt("Drake:approximateIKmex:NotEnoughInputs", "Usage approximateIKmex(model_ptr,q0,q_nom,Q,...)");
  }
  
  if (nlhs<1) return;
  
  RigidBodyManipulator *model=NULL;
  
  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "the first argument should be the model_ptr");
  memcpy(&model, mxGetData(prhs[0]), sizeof(model));
  int i, j, error, nq = model->num_dof;
  
  //Constraint preallocation
  static double Aeqdata[MAX_CONSTRS*MAX_STATE];
  static double Aindata[MAX_CONSTRS*MAX_STATE];
  static double beqdata[MAX_CONSTRS];
  static double bindata[MAX_CONSTRS];
  static RigidBodyManipulator* lastModel=NULL;
  static int lastNumJointLimits = 0;
  
  int equality_ind = 0;
  int inequality_ind = 0;
  
  VectorXd beq = Map<VectorXd>(beqdata, MAX_CONSTRS);
  VectorXd bin = Map<VectorXd>(bindata, MAX_CONSTRS);
  //Matrices are actually transposed
  MatrixXd Aeq = Map<MatrixXd>(Aeqdata, nq, MAX_CONSTRS);
  MatrixXd Ain = Map<MatrixXd>(Aindata, nq, MAX_CONSTRS);
  
  //Add joint limits
  if (lastModel != model || true) {
    lastModel = model;
    for (i=0;i<nq;i++) {
      if(!mxIsInf(model->joint_limit_max[i])) {
        bin(inequality_ind) = model->joint_limit_max[i];
        Ain.col(inequality_ind) = VectorXd::Zero(nq);
        Ain(i, inequality_ind) = 1;
        
//         cout << bin(inequality_ind) << endl << endl;
        inequality_ind++;
      }
      
      if(!mxIsInf(model->joint_limit_min[i])) {
        bin(inequality_ind) = -model->joint_limit_min[i];
        Ain.col(inequality_ind) << VectorXd::Zero(nq);
        Ain(i, inequality_ind) = -1;
//         cout << bin(inequality_ind) << endl << endl;
        inequality_ind++;
      }
    }
    lastNumJointLimits = inequality_ind;
  } else {
    inequality_ind = lastNumJointLimits;
  }
  
  // cost:  (q-q_nom)'*Q*(q-q_nom)  \equiv q'*Q*q - 2*q_nom'*Q*q  (const term doesn't matter)
  Map<VectorXd> q_nom(mxGetPr(prhs[2]), nq);
//   Map<MatrixXd> Q(mxGetPr(prhs[3]), nq, nq);
  MatrixXd Q =  Map<MatrixXd>(mxGetPr(prhs[3]), nq, nq);
  VectorXd c = -Q*q_nom;
  
  double *q0 = mxGetPr(prhs[1]);
  model->doKinematics(q0);
  
  VectorXd q0vec = Map<VectorXd>(q0, nq);
  
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
      body_pos.resize(4, 1);
      body_pos << 0, 0, 0, 1;
      world_pos = new Map<MatrixXd>(mxGetPr(prhs[i+1]), 3, n_pts);
      
      rows = 3;
      i+=2;
    } else {
      if (mxIsClass(prhs[i+2], "struct"))  {//isstruct(worldpos)
        min = mxGetField(prhs[i+2], 0, "min");
        max = mxGetField(prhs[i+2], 0, "max");
        
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
        world_pos = new Map<MatrixXd>(mxGetPr(prhs[i+2]), rows, n_pts);
      }
      
      if (mxIsClass(prhs[i+1], "char") || mxGetM(prhs[i+1]) == 1) {
        mexErrMsgIdAndTxt("Drake:approximateIKMex:NotImplemented", "collision group not implemented in mex (mposa)");
      } else {
        if (mxGetM(prhs[i+1]) !=3) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "bodypos must be 3xmi");
        }
        n_pts = mxGetN(prhs[i+1]);
        
        Map<MatrixXd> pts_tmp(mxGetPr(prhs[i+1]), 3, n_pts);
        body_pos.resize(4, n_pts);
        body_pos << pts_tmp, MatrixXd::Ones(1, n_pts);
      }
      i += 3;
    }
    
    MatrixXd x;
    MatrixXd J;
    if (body_ind == -1) {
      x = VectorXd::Zero(3, 1);
      Vector3d x_com;// = Map<Vector3d>(x.data(),3);
      
      model->getCOM(x_com);
      model->getCOMJac(J);
      x.resize(3, 1);
      x << x_com;
    } else {
      
      J.resize(rows*n_pts, nq);
      
      model->forwardKin(body_ind, body_pos, (int) rows==6, x);
      model->forwardJac(body_ind, body_pos, (int) rows==6, J);
      
      if (rows == 6 && min == NULL && max == NULL) {
        VectorXd delta;
        angleDiff(x.block(3, 0, 3, n_pts), (*world_pos).block(3, 0, 3, n_pts), &delta);
      }
    }
    
    if (max != NULL) {
      double *val = mxGetPr(max);
      
      // add inequality constraint
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN(val[j])) {
          VectorXd rowVec = J.row(j);
          double rhs = val[j] - x(j) + J.row(j)*q0vec;
          if (mxIsInf(rhs)) {
            if (rhs < 0) {
              mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "RHS of a constraint evaluates to -inf");
            }
          } else {
            Ain.col(inequality_ind) << J.row(j).transpose();
            bin(inequality_ind) = rhs;
            inequality_ind++;
          }
        }
      }
    }
    
    if (min != NULL) {
      double *val = mxGetPr(min);
      // add inequality constraint
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN(val[j])) {
          VectorXd rowVec = J.row(j);
          double rhs = -(val[j] - x(j) + J.row(j)*q0vec);
          if (mxIsInf(rhs)) {
            if (rhs < 0) {
              mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs", "RHS of a constraint evaluates to -inf");
            }
          } else {
            Ain.col(inequality_ind) << J.row(j).transpose();
            bin(inequality_ind) = rhs;
            inequality_ind++;
          }
        }
      }
    }
    
    if (min == NULL && max == NULL) {
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN((*world_pos)(j))) {
          double rhs =  (*world_pos)(j) - x(j) + J.row(j)*q0vec;
          
          Aeq.col(equality_ind) << J.row(j).transpose();
          beq(equality_ind) = rhs;
          equality_ind++;
        }
      }
    }
  }
  
  Aeq.conservativeResize(nq, equality_ind);
  Ain.conservativeResize(nq, inequality_ind);
  beq.conservativeResize(equality_ind);
  bin.conservativeResize(inequality_ind);
  
//   cout << Aeq << endl << endl;
//   cout << beq << endl << endl;
// 
//   cout << Ain << endl << endl;
//   cout << bin << endl << endl;
//   
//   cout << Q << endl << endl;
//   
//   cout << c << endl << endl;
  VectorXd q = VectorXd::Zero(nq);
//   double result = solve_quadprog(Q, c, -Aeq, beq, -Ain, bin, q);
  
  VectorXd active = VectorXd::Zero(0);
  double result = fastQP(Q.diagonal(), c, Aeq.transpose(), beq, Ain.transpose(), bin, &active, &q);  
  
  //Enable the block below to use stephens' qp code
  #if USE_EIQUADPROG_BACKUP
  if(result == 1) {
    Q += 1e-8*MatrixXd::Identity(nq,nq);
    result = solve_quadprog(Q, c, -Aeq, beq, -Ain, bin, q);
    
    if (mxIsInf(result)) {
      result = 1;
    } else {
      result = 0;
    }
  }
  #endif  
  
//   cout << result << endl << endl;
//   cout << q << endl << endl;
  plhs[0] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPr(plhs[0]), q.data(), sizeof(double)*nq);
  
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleScalar(result);
  }
  
  return;
}
