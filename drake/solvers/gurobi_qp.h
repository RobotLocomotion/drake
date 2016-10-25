#pragma once

#include <vector>
#include <set>

#include <Eigen/Dense>

#include <gurobi_c++.h>

#include "drake/common/drake_export.h"

#define CGE(call, env)                                                  \
  {                                                                     \
    int gerror;                                                         \
    gerror = call;                                                      \
    if (gerror)                                                         \
      std::cerr << "Gurobi error " << GRBgeterrormsg(env) << std::endl; \
  }

DRAKE_EXPORT GRBmodel* gurobiQP(
  GRBenv* env, std::vector<Eigen::MatrixXd*> QblkDiag,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  Eigen::VectorXd& f,
  const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq,
  const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  Eigen::VectorXd& lb, Eigen::VectorXd& ub, std::set<int>& active,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  Eigen::VectorXd& x,
  double active_set_slack_tol = 1e-4);

// template <typename tA, typename tB, typename tC, typename tD, typename tE>
// GRBmodel* gurobiQP(GRBenv *env, std::vector< Eigen::Map<tA> > QblkDiag,
// Eigen::VectorXd& f, const Eigen::MatrixBase<tB>& Aeq, const
// Eigen::MatrixBase<tC>& beq, const Eigen::MatrixBase<tD>& Ain, const
// Eigen::MatrixBase<tE>& bin, Eigen::VectorXd& lb, Eigen::VectorXd& ub,
// std::set<int>& active, Eigen::VectorXd& x);

DRAKE_EXPORT GRBmodel* gurobiActiveSetQP(
  GRBenv* env, std::vector<Eigen::MatrixXd*> QblkDiag,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  Eigen::VectorXd& f,
  const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq,
  const Eigen::MatrixXd& Ain, const Eigen::VectorXd& bin,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  Eigen::VectorXd& lb, Eigen::VectorXd& ub,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  int*& vbasis, int vbasis_len,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  int*& cbasis, int cbasis_len,
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  Eigen::VectorXd& x);
