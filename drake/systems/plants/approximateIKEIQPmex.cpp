/*
 * A c++ version of @RigidBodyTree/approximateIK.m
 */

#include <mex.h>

#include <math.h>
#include <set>
#include <Eigen/Dense>
#include "drake/systems/plants/RigidBodyTree.h"
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "drake/util/drakeUtil.h"
#include "drake/solvers/fast_qp.h"

#define MAX_CONSTRS 1000
#define MAX_ITER 20
using namespace std;

void angleDiff(VectorXd phi1, VectorXd phi2, VectorXd *d) {
  *d = phi2 - phi1;

  for (int i = 0; i < phi1.size(); i++) {
    if ((*d)(i) < -M_PI) {
      (*d)(i) = fmod((*d)(i) + M_PI, 2 * M_PI) + M_PI;
    } else {
      (*d)(i) = fmod((*d)(i) + M_PI, 2 * M_PI) - M_PI;
    }
  }
}

void angleDiff(MatrixXd phi1, MatrixXd phi2, MatrixXd *d) {
  *d = phi2 - phi1;

  for (int i = 0; i < phi1.rows(); i++) {
    for (int j = 0; j < phi1.cols(); j++) {
      if ((*d)(i, j) < -M_PI) {
        (*d)(i, j) = fmod((*d)(i, j) + M_PI, 2 * M_PI) + M_PI;
      } else {
        (*d)(i, j) = fmod((*d)(i, j) + M_PI, 2 * M_PI) - M_PI;
      }
    }
  }
}

using namespace std;

/**
 * Use Frank's fastQP code (mexed)
 * [q, info] = approximateIKEIQPmex(objgetMexModelPtr, q0, q_nom, Q, varargin)
 * info = 0 on success, 1 on failure
 **/
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 4) {
    mexErrMsgIdAndTxt("Drake:approximateIKmex:NotEnoughInputs",
                      "Usage approximateIKmex(model_ptr, q0, q_nom, Q,...)");
  }

  if (nlhs < 1) return;

  // first get the model_ptr back from matlab
  RigidBodyTree *model = (RigidBodyTree *)getDrakeMexPointer(prhs[0]);

  int i, j, error, nq = model->get_num_positions();

  static RigidBodyTree *lastModel = NULL;
  static int lastNumJointLimits = 0;

  int equality_ind = 0;
  int inequality_ind = 0;

  // Constraint preallocation
  Matrix<double, -1, 1, 0, MAX_CONSTRS, 1> beq(MAX_CONSTRS);
  Matrix<double, -1, 1, 0, MAX_CONSTRS, 1> bin(MAX_CONSTRS);
  Matrix<double, -1, -1, RowMajor, MAX_CONSTRS, -1> Aeq(MAX_CONSTRS, nq);
  Matrix<double, -1, -1, RowMajor, MAX_CONSTRS, -1> Ain(MAX_CONSTRS, nq);

  // Add joint limits
  if (lastModel != model || true) {
    lastModel = model;
    for (i = 0; i < nq; i++) {
      if (!mxIsInf(model->joint_limit_max[i])) {
        bin(inequality_ind) = model->joint_limit_max[i];
        Ain.row(inequality_ind) = MatrixXd::Zero(1, nq);
        Ain(inequality_ind, i) = 1;

        //         cout << bin(inequality_ind) << endl << endl;
        inequality_ind++;
      }

      if (!mxIsInf(model->joint_limit_min[i])) {
        bin(inequality_ind) = -model->joint_limit_min[i];
        Ain.row(inequality_ind) << MatrixXd::Zero(1, nq);
        Ain(inequality_ind, i) = -1;
        //         cout << bin(inequality_ind) << endl << endl;
        inequality_ind++;
      }
    }
    lastNumJointLimits = inequality_ind;
  } else {
    inequality_ind = lastNumJointLimits;
  }

  // cost:  (q-q_nom)'*Q*(q-q_nom)  \equiv q'*Q*q - 2*q_nom'*Q*q  (const term
  // doesn't matter)
  Map<VectorXd> q_nom(mxGetPrSafe(prhs[2]), nq);
  Map<MatrixXd> Q(mxGetPrSafe(prhs[3]), nq, nq);
  VectorXd c = -Q * q_nom;

  double *q0 = mxGetPrSafe(prhs[1]);
  model->doKinematics(q0);

  VectorXd q0vec = Map<VectorXd>(q0, nq);

  i = 4;
  while (i < nrhs) {
    MatrixXd body_pos;
    Map<MatrixXd> *world_pos = NULL;
    int rows;
    int body_ind = mxGetScalar(prhs[i]) - 1;
    mxArray *min = NULL;
    mxArray *max = NULL;
    int n_pts;
    if (body_ind == -1) {
      int n_pts = mxGetN(prhs[i + 1]);
      body_pos.resize(4, 1);
      body_pos << 0, 0, 0, 1;
      world_pos = new Map<MatrixXd>(mxGetPrSafe(prhs[i + 1]), 3, n_pts);

      rows = 3;
      i += 2;
    } else {
      if (mxIsClass(prhs[i + 2], "struct")) {  // isstruct(worldpos)
        min = mxGetField(prhs[i + 2], 0, "min");
        max = mxGetField(prhs[i + 2], 0, "max");

        if (min == NULL || max == NULL) {
          mexErrMsgIdAndTxt(
              "Drake:approximateIKMex:BadInputs",
              "if world_pos is a struct, it must have fields .min and .max");
        }

        rows = mxGetM(min);

        if (rows != 3 && rows != 6) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs",
                            "world_pos.min must have 3 or 6 rows");
        }

        if (mxGetM(max) != rows) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs",
                            "world_pos.max must have the same number of rows "
                            "as world_pos.min");
        }
      } else {
        rows = mxGetM(prhs[i + 2]);
        int n_pts = mxGetN(prhs[i + 2]);
        world_pos = new Map<MatrixXd>(mxGetPrSafe(prhs[i + 2]), rows, n_pts);
      }

      if (mxIsClass(prhs[i + 1], "char") || mxGetM(prhs[i + 1]) == 1) {
        mexErrMsgIdAndTxt("Drake:approximateIKMex:NotImplemented",
                          "collision group not implemented in mex (mposa)");
      } else {
        if (mxGetM(prhs[i + 1]) != 3) {
          mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs",
                            "bodypos must be 3xmi");
        }
        n_pts = mxGetN(prhs[i + 1]);

        Map<MatrixXd> pts_tmp(mxGetPrSafe(prhs[i + 1]), 3, n_pts);
        body_pos.resize(4, n_pts);
        body_pos << pts_tmp, MatrixXd::Ones(1, n_pts);
      }
      i += 3;
    }

    MatrixXd x;
    MatrixXd J;
    if (body_ind == -1) {
      x = VectorXd::Zero(3, 1);
      Vector3d x_com;

      model->getCOM(x_com);
      model->getCOMJac(J);
      x.resize(3, 1);
      x << x_com;
    } else {
      J.resize(rows * n_pts, nq);

      model->forwardKin(body_ind, body_pos, (int)rows == 6, x);
      model->forwardJac(body_ind, body_pos, (int)rows == 6, J);

      if (rows == 6 && min == NULL && max == NULL) {
        VectorXd delta;
        angleDiff(x.block(3, 0, 3, n_pts), (*world_pos).block(3, 0, 3, n_pts),
                  &delta);
        (*world_pos).block(3, 0, 3, n_pts) = x.block(3, 0, 3, n_pts) + delta;
      }
    }

    if (max != NULL) {
      double *val = mxGetPrSafe(max);

      // add inequality constraint
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN(val[j])) {
          VectorXd rowVec = J.row(j);
          double rhs = val[j] - x(j) + J.row(j) * q0vec;
          if (mxIsInf(rhs)) {
            if (rhs < 0) {
              mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs",
                                "RHS of a constraint evaluates to -inf");
            }
          } else {
            Ain.row(inequality_ind) << J.row(j);
            bin(inequality_ind) = rhs;
            inequality_ind++;
          }
        }
      }
    }

    if (min != NULL) {
      double *val = mxGetPrSafe(min);
      // add inequality constraint
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN(val[j])) {
          VectorXd rowVec = J.row(j);
          double rhs = -(val[j] - x(j) + J.row(j) * q0vec);
          if (mxIsInf(rhs)) {
            if (rhs < 0) {
              mexErrMsgIdAndTxt("Drake:approximateIKMex:BadInputs",
                                "RHS of a constraint evaluates to -inf");
            }
          } else {
            Ain.row(inequality_ind) << J.row(j);
            bin(inequality_ind) = rhs;
            inequality_ind++;
          }
        }
      }
    }

    if (min == NULL && max == NULL) {
      for (j = 0; j < rows; j++) {
        if (!mxIsNaN((*world_pos)(j))) {
          double rhs = (*world_pos)(j)-x(j) + J.row(j) * q0vec;

          Aeq.row(equality_ind) << J.row(j);
          beq(equality_ind) = rhs;
          equality_ind++;
        }
      }
    }
    delete world_pos;
    if (min) mxDestroyArray(min);
    if (max) mxDestroyArray(max);
  }

  Aeq.conservativeResize(equality_ind, nq);
  Ain.conservativeResize(inequality_ind, nq);
  beq.conservativeResize(equality_ind);
  bin.conservativeResize(inequality_ind);

  cout << "c is " << c.rows() << endl;
  cout << "Aeq is " << Aeq.rows() << " by " << Aeq.cols() << endl;

  VectorXd q = model->getZeroConfiguration();
  //   double result = solve_quadprog(Q, c, -Aeq, beq, -Ain, bin, q);

  VectorXd Qdiag = Q.diagonal();
  vector<MatrixBase<VectorXd> > blkQ;
  blkQ.push_back(Qdiag);
  set<int> active;
  double result = fastQP(blkQ, c, Aeq, beq, Ain, bin, active, q);

  plhs[0] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPrSafe(plhs[0]), q.data(), sizeof(double) * nq);

  if (nlhs > 1) {
    plhs[1] = mxCreateDoubleScalar(result);
  }

  return;
}
